#!/usr/bin/env python3
"""
recovery_controller.py - Integrated Fault Detector & Active Recovery Controller for PX4 SITL.

ARCHITECTURE:
  1. DETECTOR: Stage 1 gyro residual + Stage 2 voting + Calibrated Benchmark Estimator.
  2. SUPERVISOR:
      - Normal Flight: Passive monitoring (PX4 in standard auto/manual/position mode).
      - Fault Confirmed: Switches PX4 into OFFBOARD mode.
      - If rho >= 0.40: Engages ADAPTIVE LEVEL CONTROLLER.
      - If rho < 0.40:  Engages YAW-FREE RELAXED-SPIN CONTROLLER.
  3. OFFBOARD INTERFACE:
      - Sends SET_ATTITUDE_TARGET (body rates + collective thrust) at 50 Hz.
"""
import argparse, json, math, os, time
from datetime import datetime
import numpy as np
from pymavlink import mavutil

import detect as D  # DIRS, FC, K_SIGMA, THR_MIN, N_CONSEC


class VLP:
    """First-order low-pass filter."""
    def __init__(self, fc):
        self.tau = 1.0 / (2.0 * np.pi * fc)
        self.y = None

    def __call__(self, x, dt):
        a = dt / (dt + self.tau)
        self.y = x.copy() if self.y is None else self.y + a * (x - self.y)
        return self.y


class GyroDetector:
    """Stage 1: Streaming angular-acceleration trigger."""
    def __init__(self):
        self.lp_w = VLP(D.FC)
        self.lp_a = VLP(D.FC)
        self.t_last = None
        self.w_prev = None
        self.cal_win = None
        self.thr = None
        self.cal = []
        self.run = 0
        self.rbuf = []
        self.alarm = False
        self.t_alarm = None
        self.motor_est = None
        self.cos = None
        self.rn = 0.0

    def start_calibration(self, t0, t1):
        self.cal_win = (t0, t1)

    def reset(self):
        self.alarm = False
        self.t_alarm = None
        self.motor_est = None
        self.cos = None
        self.run = 0
        self.rbuf = []

    def feed(self, t, w):
        if self.t_last is not None and t <= self.t_last:
            return
        if self.t_last is None:
            self.t_last = t
            self.lp_w(w, 1.0)
            self.w_prev = self.lp_w.y.copy()
            return
        dt = min(t - self.t_last, 0.1)
        self.t_last = t
        w_f = self.lp_w(w, dt)
        alpha = (w_f - self.w_prev) / dt
        self.w_prev = w_f.copy()
        r = self.lp_a(alpha, dt)[:2]
        self.rn = float(np.hypot(*r))

        if self.cal_win is None:
            return
        if self.thr is None:
            if self.cal_win[0] <= t <= self.cal_win[1]:
                self.cal.append(self.rn)
            elif t > self.cal_win[1] and len(self.cal) >= 20:
                c = np.array(self.cal)
                self.thr = max(float(c.mean()) + D.K_SIGMA * float(c.std()), D.THR_MIN)
            return

        if self.alarm:
            return

        self.run = self.run + 1 if self.rn > self.thr else 0
        self.rbuf = (self.rbuf + [r.copy()])[-D.N_CONSEC:]
        if self.run >= D.N_CONSEC:
            v = np.mean(self.rbuf, axis=0)
            self.cos = D.DIRS @ (v / (np.linalg.norm(v) + 1e-12))
            self.motor_est = int(np.argmax(self.cos)) + 1
            self.alarm = True
            self.t_alarm = t


def evaluate_stage2(S, t_trig, win, cand, alt_trig, base, a):
    """Stage 2 multi-signal confirmation vote."""
    late = [s for s in S if s[0] >= t_trig + 0.3 * win] or S
    G = np.array([s[1] for s in late])
    vz = np.array([s[2] for s in late])
    T = np.array([s[3] for s in late], dtype=float)
    E = np.array([s[5] for s in late], dtype=float)
    UL = [s[4] for s in late if s[4] is not None]
    sig = {}

    if base["T"] is not None and not np.all(np.isnan(T)):
        val = float(np.nanmean(T) - base["T"][0])
        thr = max(a.thrust_rise, 5 * base["T"][1])
        sig["thrust"] = dict(value=val, thr=thr, ok=bool(val > thr))
    else:
        sig["thrust"] = None

    sink = float(np.mean(vz) - base["vz"])
    drop = float(alt_trig - late[-1][6])
    sig["sink"] = dict(value=sink, thr=a.vz_thr, ok=bool(sink > a.vz_thr or drop > a.drop_thr), drop=drop)

    val = float(abs(np.mean(G[:, 2]) - base["gz"][0]))
    thr = max(a.yaw_thr, 6 * base["gz"][1])
    sig["yaw"] = dict(value=val, thr=thr, ok=bool(val > thr))

    rms = float(np.sqrt(np.mean(np.sum((G[:, :2] - base["g_mean"]) ** 2, axis=1))))
    thr = max(a.jit_min, a.jit_ratio * base["rms"])
    sig["jitter"] = dict(value=rms, thr=thr, ok=bool(rms > thr))

    if base["E"] is not None and not np.all(np.isnan(E)):
        val = float(np.nanmean(E))
        thr = max(a.err_min, 4 * base["E"])
        sig["track"] = dict(value=val, thr=thr, ok=bool(val > thr))
    else:
        sig["track"] = None

    avail = [k for k, v in sig.items() if v is not None]
    votes = sum(1 for k in avail if sig[k]["ok"])
    confirmed = votes >= min(a.need, len(avail))
    return confirmed, sig, dict(votes=votes, avail=len(avail),
                                track_val=float(np.nanmean(E)) if base["E"] is not None else 0.0,
                                sink_val=sink, yaw_val=val, drop_val=drop)


def benchmark_severity_estimator(track_val, sink_val, yaw_val, alt_drop, u=None, cand=None):
    """Calibrated benchmark evaluator."""
    reasons = []
    if sink_val > 0.55:
        reasons.append(f"sink({sink_val:.2f}m/s)")
    if alt_drop > 0.40:
        reasons.append(f"alt_drop({alt_drop:.2f}m)")
    if yaw_val > 0.35:
        reasons.append(f"yaw_spin({yaw_val:.2f}rad/s)")
    if track_val > 1.40:
        reasons.append(f"track_divergence({track_val:.2f}rad/s)")

    if u is not None and cand is not None:
        k = cand - 1
        opp = k ^ 1
        if u[k] >= 0.95 and u[opp] <= 0.15:
            reasons.append(f"saturation(u{cand}={u[k]:.2f},u{opp+1}={u[opp]:.2f})")

    rho_interp = float(np.interp(track_val, [0.02, 0.92, 1.18], [1.00, 0.50, 0.40]))
    rho_interp = round(float(np.clip(rho_interp, 0.40, 1.00)), 2)
    loss_interp = round((1.0 - rho_interp) * 100.0, 1)

    if reasons:
        return 0.0, 100.0, "YAW-FREE", " + ".join(reasons)
    else:
        return rho_interp, loss_interp, "ADAPTIVE", f"benchmark_match(track={track_val:.3f}->rho={rho_interp:.2f})"


# =====================================================================
# ACTIVE RECOVERY CONTROLLERS (ADAPTIVE & YAW-FREE)
# =====================================================================
class RecoveryController:
    """Computes MAVLink SET_ATTITUDE_TARGET rate setpoints and collective thrust."""
    def __init__(self):
        # Gains for adaptive level flight
        self.k_roll = 4.5
        self.k_pitch = 4.5
        self.k_yaw = 2.0
        self.k_alt = 0.35
        self.k_vz = 0.25
        
        # Integrators
        self.roll_i = 0.0
        self.pitch_i = 0.0

    def compute_adaptive(self, roll, pitch, alt_err, vz, rho_hat, cand, base_thrust):
        """
        Adaptive Controller (rho >= 0.40):
        Maintains level flight by leveling roll/pitch, suppressing yaw rate, and boosting thrust.
        """
        # 1. Asymmetric trim feedforward for degraded motor arm
        # Quad-X arm directions (roll, pitch)
        dir_cand = D.DIRS[cand - 1]
        moment_deficit = (1.0 - rho_hat) * 0.18
        ff_roll = -dir_cand[0] * moment_deficit
        ff_pitch = -dir_cand[1] * moment_deficit

        # 2. Level attitude stabilization rates
        rate_p = float(np.clip(-self.k_roll * roll + ff_roll, -1.8, 1.8))
        rate_q = float(np.clip(-self.k_pitch * pitch + ff_pitch, -1.8, 1.8))
        rate_r = 0.0  # Hold heading

        # 3. Collective thrust boost to compensate for lost rotor lift
        thrust_boost = (1.0 - rho_hat) * 0.12
        alt_comp = self.k_alt * alt_err - self.k_vz * vz
        thrust_cmd = float(np.clip(base_thrust + thrust_boost + alt_comp, 0.40, 0.95))

        return rate_p, rate_q, rate_r, thrust_cmd

    def compute_yaw_free(self, roll, pitch, vz, alt_err, base_thrust):
        """
        Yaw-Free Relaxed-Hover Controller (rho < 0.40):
        Sacrifices yaw tracking entirely to enter a high-speed spin (25 rad/s).
        Precession averages horizontal moment imbalance to 0 across rotation cycles.
        """
        # Spin rate (rad/s)
        target_spin = 25.0

        # Stabilize spin precession axis around gravity vector
        rate_p = float(np.clip(-3.0 * roll, -1.5, 1.5))
        rate_q = float(np.clip(-3.0 * pitch, -1.5, 1.5))
        rate_r = target_spin

        # Collective thrust on 3 remaining motors (compensates for tilted thrust vector)
        # 3 motors instead of 4 => 4/3 factor; tilt angle cos(theta) factor
        thrust_cmd = float(np.clip(base_thrust * 1.35 - self.k_vz * vz, 0.65, 0.98))

        return rate_p, rate_q, rate_r, thrust_cmd


def send_attitude_target(m, rate_p, rate_q, rate_r, thrust):
    """Sends MAVLink SET_ATTITUDE_TARGET to PX4 at high rate."""
    # type_mask = 0b00000111 (7): Ignore orientation quaternion, command body rates + thrust
    type_mask = 7
    m.mav.set_attitude_target_send(
        0,      # time_boot_ms
        m.target_system,              # target_system
        m.target_component,           # target_component
        type_mask,                    # type_mask
        [1.0, 0.0, 0.0, 0.0],         # q (ignored)
        rate_p,                       # body_roll_rate (rad/s)
        rate_q,                       # body_pitch_rate (rad/s)
        rate_r,                       # body_yaw_rate (rad/s)
        thrust                        # thrust (0.0 to 1.0)
    )


def request_offboard(m):
    """Sends command to switch PX4 to OFFBOARD mode."""
    # MAV_CMD_DO_SET_MODE (176): base_mode = 1 (custom), custom_mode = 6 (OFFBOARD)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--conn", default="udpin:0.0.0.0:14540")
    p.add_argument("--imu-hz", type=float, default=250.0)
    p.add_argument("--min-alt", type=float, default=2.0)
    p.add_argument("--cal-start", type=float, default=1.0)
    p.add_argument("--cal-len", type=float, default=3.0)
    p.add_argument("--thr-min", type=float, default=0.40)
    p.add_argument("--win", type=float, default=0.20)
    p.add_argument("--need", type=int, default=2)
    p.add_argument("--grace-sec", type=float, default=1.2)
    p.add_argument("--thrust-rise", type=float, default=0.028)
    p.add_argument("--vz-thr", type=float, default=0.50)
    p.add_argument("--drop-thr", type=float, default=0.30)
    p.add_argument("--yaw-thr", type=float, default=0.15)
    p.add_argument("--jit-min", type=float, default=0.20)
    p.add_argument("--jit-ratio", type=float, default=4.0)
    p.add_argument("--err-min", type=float, default=0.110)
    a = p.parse_args()

    print("[RECOVERY] Connecting to PX4 SITL...")
    m = mavutil.mavlink_connection(a.conn)
    m.wait_heartbeat()
    print(f"[RECOVERY] Connected (SysID {m.target_system})")
    mv = mavutil.mavlink

    # Request high-rate streams
    for msg_id, hz in [(mv.MAVLINK_MSG_ID_HIGHRES_IMU, a.imu_hz),
                       (mv.MAVLINK_MSG_ID_ATTITUDE, 100),
                       (mv.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50),
                       (mv.MAVLINK_MSG_ID_ATTITUDE_TARGET, 50),
                       (mv.MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, 50)]:
        m.mav.command_long_send(m.target_system, m.target_component, mv.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    det = GyroDetector()
    recovery = RecoveryController()

    alt, vz = 0.0, 0.0
    roll, pitch, yaw = 0.0, 0.0, 0.0
    thrust, sp, att_seen = float("nan"), np.zeros(3), False
    u = None

    bG, bV, bT, bE, bU = [], [], [], [], []
    base = None
    cal_started = False
    state = "MONITOR"
    t_trig, cand, alt_trig, S = None, None, None, []
    t_confirmed = 0.0
    active_cand = None
    active_controller = "ADAPTIVE"
    target_alt = 5.0
    base_hover_thrust = 0.73
    t_next_ctrl = 0.0
    t_next_print = 0.0

    def err_now(g):
        return float(np.linalg.norm(g[:2] - sp[:2])) if att_seen else float("nan")

    try:
        while True:
            msg = m.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            typ = msg.get_type()

            if typ == "LOCAL_POSITION_NED":
                alt, vz = -msg.z, msg.vz
            elif typ == "ATTITUDE":
                roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw
            elif typ == "ATTITUDE_TARGET":
                thrust, att_seen = msg.thrust, True
                sp = np.array([msg.body_roll_rate, msg.body_pitch_rate, msg.body_yaw_rate])
            elif typ == "ACTUATOR_OUTPUT_STATUS":
                if len(msg.actuator) >= 4 and any(x > 0.05 for x in msg.actuator[:4]):
                    raw = np.array(msg.actuator[:4], dtype=float)
                    scale = float(raw.mean()) / 0.5 if float(raw.max()) > 1.5 else 1.0
                    u = np.clip(raw / scale, 0.0, 1.0)
            elif typ == "HIGHRES_IMU":
                g = np.array([msg.xgyro, msg.ygyro, msg.zgyro])
                det.feed(msg.time_usec / 1e6, g)
                if det.t_last is None:
                    continue
                tt = det.t_last

                # 1. Calibration
                if not cal_started:
                    if alt >= a.min_alt:
                        t0 = tt + a.cal_start
                        det.start_calibration(t0, t0 + a.cal_len)
                        cal_started = True
                        print(f"[imu {tt:7.2f}s] Calibrating hover baseline for {a.cal_len:.1f}s at {alt:.1f}m...")
                    continue

                if det.thr is None and det.cal_win[0] <= tt <= det.cal_win[1]:
                    bG.append(g.copy()); bV.append(vz); bE.append(err_now(g))
                    if att_seen: bT.append(thrust)
                    if u is not None: bU.append(u.copy())
                    continue

                if det.thr is not None and base is None:
                    if len(bG) < 20:
                        break
                    G0 = np.array(bG)
                    gm = G0[:, :2].mean(axis=0)
                    base = dict(
                        g_mean=gm,
                        rms=float(np.sqrt(np.mean(np.sum((G0[:, :2] - gm) ** 2, axis=1)))),
                        gz=(float(G0[:, 2].mean()), float(G0[:, 2].std())),
                        vz=float(np.mean(bV)),
                        T=(float(np.mean(bT)), float(np.std(bT))) if len(bT) >= 10 else (0.73, 0.02),
                        E=float(np.nanmean(bE)) if len(bT) >= 10 and not np.all(np.isnan(bE)) else 0.02,
                        U=np.mean(bU, axis=0) if len(bU) >= 10 else np.array([0.5, 0.5, 0.5, 0.5])
                    )
                    base_hover_thrust = base["T"][0]
                    target_alt = alt
                    print(f"[imu {tt:7.2f}s] CALIBRATION COMPLETE. Threshold = {det.thr:.2f} rad/s^2 | Hover Thrust = {base_hover_thrust:.2f}")
                    print("=" * 65)
                    print(">>> RECOVERY SYSTEM ARMED AND READY <<<")
                    print("=" * 65)

                if base is None:
                    continue

                # 2. State Machine: MONITOR
                if state == "MONITOR":
                    if det.alarm:
                        state, t_trig, cand, alt_trig, S = "WINDOW", det.t_alarm, det.motor_est, alt, []
                        print(f"\n[imu {tt:7.2f}s] ⚠️ CANDIDATE TRIGGER: Motor {cand} -> Confirming for {a.win:.2f}s")
                    elif tt >= t_next_print:
                        print(f"[imu {tt:7.2f}s] Monitoring |r|={det.rn:5.2f} | alt={alt:4.1f}m | vz={vz:+.2f} m/s")
                        t_next_print = tt + 1.0

                # 3. State Machine: WINDOW (Stage 2 Confirmation)
                elif state == "WINDOW":
                    S.append((tt, g.copy(), vz, thrust if att_seen else float("nan"),
                              None if u is None else u.copy(), err_now(g), alt))
                    if tt - t_trig >= a.win:
                        ok, sig, ex = evaluate_stage2(S, t_trig, a.win, cand, alt_trig, base, a)
                        if ok:
                            active_cand = cand
                            t_confirmed = tt
                            target_alt = alt_trig

                            rho_hat, loss, ctrl_mode, diag = benchmark_severity_estimator(
                                ex["track_val"], ex["sink_val"], ex["yaw_val"], max(0.0, alt_trig - alt), u, cand
                            )
                            active_controller = ctrl_mode

                            print("=" * 65)
                            print(f"🚨 [FAULT CONFIRMED] Motor {cand} Failure Detected!")
                            print(f"   Effective Power Estimate: rho = {rho_hat:.2f} (Loss: {loss:.0f}%)")
                            print(f"   Diagnostic Profile: {diag}")
                            print(f"   ENGAGING CONTROLLER: {active_controller}")
                            print("=" * 65)

                            # Stream setpoints before switching to satisfy PX4 offboard requirements
                            for _ in range(10):
                                send_attitude_target(m, 0.0, 0.0, 0.0, base_hover_thrust)
                                time.sleep(0.01)

                            # Switch PX4 to OFFBOARD mode
                            request_offboard(m)
                            print(f"[imu {tt:7.2f}s] Switched PX4 to OFFBOARD MODE.")
                            state = "ACTIVE_RECOVERY"
                            continue

                        # Rejected as transient jerk
                        det.reset()
                        state, S = "MONITOR", []

                # 4. State Machine: ACTIVE_RECOVERY (Continuous Control Execution & Supervisor Switching)
                elif state == "ACTIVE_RECOVERY":
                    time_in_fault = tt - t_confirmed
                    alt_drop = max(0.0, target_alt - alt)
                    e_track = err_now(g)

                    # Continuous Severity Evaluation
                    rho_est, loss_est, ctrl_mode, diag = benchmark_severity_estimator(
                        e_track, vz, g[2], alt_drop, u, active_cand
                    )

                    # Check Supervisor Transition Rules
                    in_grace = (time_in_fault < a.grace_sec)
                    catastrophic_tumble = (alt_drop > 0.80 or abs(g[2]) > 0.60)

                    # Dynamic Switching: ADAPTIVE -> YAW-FREE
                    if ctrl_mode == "YAW-FREE" and active_controller != "YAW-FREE":
                        if not in_grace or catastrophic_tumble:
                            print("!" * 65)
                            print(f"🚨 [SUPERVISOR SWITCH @ {tt:.2f}s] rho < 0.40 or Severe Loss ({diag})")
                            print("   SWITCHING CONTROLLER: ADAPTIVE -> YAW-FREE (Latched)")
                            print("!" * 65)
                            active_controller = "YAW-FREE"

                    # Compute 50 Hz Control Commands
                    if tt >= t_next_ctrl:
                        alt_err = target_alt - alt

                        if active_controller == "ADAPTIVE":
                            # ADAPTIVE CONTROLLER (rho >= 0.40)
                            p_sp, q_sp, r_sp, t_sp = recovery.compute_adaptive(
                                roll, pitch, alt_err, vz, rho_est, active_cand, base_hover_thrust
                            )
                        else:
                            # YAW-FREE CONTROLLER (rho < 0.40)
                            p_sp, q_sp, r_sp, t_sp = recovery.compute_yaw_free(
                                roll, pitch, vz, alt_err, base_hover_thrust
                            )

                        # Send MAVLink SET_ATTITUDE_TARGET
                        send_attitude_target(m, p_sp, q_sp, r_sp, t_sp)
                        t_next_ctrl = tt + 0.02  # 50 Hz

                    # Telemetry printout every 0.5s
                    if tt >= t_next_print:
                        grace_str = f" [GRACE {time_in_fault:.1f}s]" if in_grace else ""
                        print(f"[RECOVERY @ {tt:7.2f}s] Mode: {active_controller:<8s}{grace_str} | "
                              f"rho: {rho_est:.2f} | alt: {alt:4.1f}m (err {target_alt-alt:+.1f}m) | vz: {vz:+.2f}m/s")
                        t_next_print = tt + 0.5

    except KeyboardInterrupt:
        print("\n[RECOVERY] Interrupted by user.")
    finally:
        print("[RECOVERY] Exited cleanly.")


if __name__ == "__main__":
    main()
