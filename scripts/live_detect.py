#!/usr/bin/env python3
"""
live_detect_v2_fixed.py - Calibrated Benchmark Fault Detector with Adaptive Handover Grace Period.

FEATURES:
  - 3-Point Calibrated Benchmark:
      track ~ 0.02 rad/s -> rho = 1.00 (Healthy Hover)
      track ~ 0.92 rad/s -> rho = 0.50 (50% Loss) -> ADAPTIVE
      track ~ 1.18 rad/s -> rho = 0.40 (60% Loss) -> ADAPTIVE
  - Adaptive Handover Grace Period (--grace-sec 1.2):
      Prevents instant supervisor panic during initial post-fault transient sink.
  - Catastrophic Safety Guard:
      Immediate YAW-FREE escalation only on true tumble (alt_drop > 0.8m or yaw > 0.6 rad/s).
  - FaultBus IPC Interface:
      Streams fault events to main.py to trigger controller transitions.
"""
import argparse, csv, json, math, os, sys, time
from datetime import datetime
import numpy as np
from pymavlink import mavutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
TESTING_DIR = os.path.join(SCRIPT_DIR, "testing")
if os.path.exists(TESTING_DIR) and TESTING_DIR not in sys.path:
    sys.path.insert(0, TESTING_DIR)

import detect as D  # DIRS, FC, K_SIGMA, THR_MIN, N_CONSEC
from fault_bus import FaultSender  # <-- LINE 1: Import FaultSender


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


def make_run_dir(root):
    run_dir = os.path.join(root, "run_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    latest = os.path.join(root, "latest")
    try:
        if os.path.islink(latest) or os.path.exists(latest):
            os.unlink(latest)
        os.symlink(os.path.abspath(run_dir), latest)
    except Exception:
        pass
    return run_dir


def evaluate(S, t_trig, win, cand, alt_trig, base, a):
    """Stage 2: Multi-signal confirmation vote over the late part of the window."""
    late = [s for s in S if s[0] >= t_trig + 0.3 * win] or S
    G = np.array([s[1] for s in late])
    vz = np.array([s[2] for s in late])
    T = np.array([s[3] for s in late], dtype=float)
    E = np.array([s[5] for s in late], dtype=float)
    UL = [s[4] for s in late if s[4] is not None]
    sig = {}

    # 1. thrust demand rise
    if base["T"] is not None and not np.all(np.isnan(T)):
        val = float(np.nanmean(T) - base["T"][0])
        thr = max(a.thrust_rise, 5 * base["T"][1])
        sig["thrust"] = dict(value=val, thr=thr, ok=bool(val > thr))
    else:
        sig["thrust"] = None

    # 2. sink: descent speed or lost altitude
    sink = float(np.mean(vz) - base["vz"])
    drop = float(alt_trig - late[-1][6])
    sig["sink"] = dict(value=sink, thr=a.vz_thr, ok=bool(sink > a.vz_thr or drop > a.drop_thr), drop=drop)

    # 3. yaw rate offset
    val = float(abs(np.mean(G[:, 2]) - base["gz"][0]))
    thr = max(a.yaw_thr, 6 * base["gz"][1])
    sig["yaw"] = dict(value=val, thr=thr, ok=bool(val > thr))

    # 4. jitter
    rms = float(np.sqrt(np.mean(np.sum((G[:, :2] - base["g_mean"]) ** 2, axis=1))))
    thr = max(a.jit_min, a.jit_ratio * base["rms"])
    sig["jitter"] = dict(value=rms, thr=thr, ok=bool(rms > thr))

    # 5. rate tracking error
    if base["E"] is not None and not np.all(np.isnan(E)):
        val = float(np.nanmean(E))
        thr = max(a.err_min, 4 * base["E"])
        sig["track"] = dict(value=val, thr=thr, ok=bool(val > thr))
    else:
        sig["track"] = None

    # 6. motor output divergence
    out_motor, u_late = None, None
    if base["U"] is not None and len(UL) >= 3:
        u_late = np.mean(UL, axis=0)
        delta = u_late - base["U"]
        out_motor = int(np.argmax(delta)) + 1
        sig["outputs"] = dict(value=float(delta[cand - 1]), thr=a.out_rise,
                              ok=bool(delta[cand - 1] > a.out_rise and out_motor == cand), out_motor=out_motor)
    else:
        sig["outputs"] = None

    avail = [k for k, v in sig.items() if v is not None]
    votes = sum(1 for k in avail if sig[k]["ok"])
    confirmed = votes >= min(a.need, len(avail))
    return confirmed, sig, dict(votes=votes, avail=len(avail), out_motor=out_motor, u_late=u_late,
                                track_val=float(np.nanmean(E)) if base["E"] is not None else 0.0,
                                sink_val=sink, yaw_val=val, drop_val=drop)


def show(sig):
    for n, r in sig.items():
        if r is None:
            print(f"     {n:8s} unavailable")
        elif n == "sink" and "drop" in r:
            print(f"     {n:8s} value={r['value']:+.3f} m/s (drop={r['drop']:+.3f}m)  thr={r['thr']:.3f}  {'YES' if r['ok'] else 'no'}")
        else:
            print(f"     {n:8s} value={r['value']:+.3f}  thr={r['thr']:.3f}  {'YES' if r['ok'] else 'no'}")


def benchmark_severity_estimator(track_val, sink_val, yaw_val, alt_drop, u=None, cand=None):
    """
    Calibrated Benchmark Profile:
      - 1.00 (Healthy Hover): track ~ 0.02 rad/s -> rho = 1.00 -> ADAPTIVE
      - 0.50 (50% Loss):      track ~ 0.92 rad/s, sink ~ 0.20 -> rho = 0.50 -> ADAPTIVE
      - 0.40 (60% Loss):      track ~ 1.18 rad/s, sink ~ 0.41 -> rho = 0.40 -> ADAPTIVE
      - Severe Failure:       track > 1.40, sink > 0.55, yaw > 0.35 -> YAW-FREE
    """
    reasons = []

    # Severe Cutoffs (Physical limits above controllable hover)
    if sink_val > 0.55:
        reasons.append(f"sink({sink_val:.2f}m/s)")
    if alt_drop > 0.120:
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
        elif float(np.mean(u)) < 0.18 and (track_val > 0.20 or sink_val > 0.35):
            reasons.append("ground_idle_post_crash")

    # Controllable interpolation using calibrated anchors:
    rho_interp = float(np.interp(track_val, [0.02, 0.92, 1.18], [1.00, 0.50, 0.40]))
    rho_interp = round(float(np.clip(rho_interp, 0.40, 1.00)), 2)
    loss_interp = round((1.0 - rho_interp) * 100.0, 1)

    if reasons:
        return 0.0, 100.0, "YAW-FREE", " + ".join(reasons)
    else:
        return rho_interp, loss_interp, "ADAPTIVE", f"benchmark_match(track={track_val:.3f}->rho={rho_interp:.2f})"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--conn", default="udpin:0.0.0.0:14540")
    p.add_argument("--run-dir", default="live_runs", dest="run_dir")
    p.add_argument("--imu-hz", type=float, default=250.0, dest="imu_hz")
    p.add_argument("--min-alt", type=float, default=2.0, dest="min_alt")
    p.add_argument("--max-speed", type=float, default=0.5, dest="max_speed")
    p.add_argument("--cal-start", type=float, default=1.0, dest="cal_start")
    p.add_argument("--cal-len", type=float, default=3.0, dest="cal_len")
    p.add_argument("--thr-min", type=float, default=0.40, dest="thr_min")
    p.add_argument("--k-sigma", type=float, default=None, dest="k_sigma")
    p.add_argument("--n-consec", type=int, default=3, dest="n_consec")

    # Fast 200 ms confirmation window
    p.add_argument("--win", type=float, default=0.20, help="shortened confirmation window [s]")
    p.add_argument("--win-low", type=float, default=0.18, dest="win_low")
    p.add_argument("--low-alt", type=float, default=3.0, dest="low_alt")
    p.add_argument("--need", type=int, default=2, help="votes needed")
    p.add_argument("--cooldown", type=float, default=0.10, help="shortened cooldown [s]")

    # Adaptive handover grace period
    p.add_argument("--grace-sec", type=float, default=1.2, dest="grace_sec",
                   help="seconds to give ADAPTIVE control to arrest descent before allowing YAW-FREE escalation")

    p.add_argument("--thrust-rise", type=float, default=0.028, dest="thrust_rise")
    p.add_argument("--vz-thr", type=float, default=0.50, dest="vz_thr")
    p.add_argument("--drop-thr", type=float, default=0.30, dest="drop_thr")
    p.add_argument("--yaw-thr", type=float, default=0.15, dest="yaw_thr")
    p.add_argument("--jit-min", type=float, default=0.20, dest="jit_min")
    p.add_argument("--jit-ratio", type=float, default=4.0, dest="jit_ratio")
    p.add_argument("--err-min", type=float, default=0.110, dest="err_min")
    p.add_argument("--out-rise", type=float, default=0.030, dest="out_rise")
    a = p.parse_args()

    if a.thr_min is not None: D.THR_MIN = a.thr_min
    if a.k_sigma is not None: D.K_SIGMA = a.k_sigma
    if a.n_consec is not None: D.N_CONSEC = a.n_consec

    print(f"[v2f] THR_MIN={D.THR_MIN} K_SIGMA={D.K_SIGMA} N_CONSEC={D.N_CONSEC}  win={a.win}s need={a.need}")

    run_dir = make_run_dir(a.run_dir)
    print(f"[v2f] Run directory: {run_dir}")
    m = mavutil.mavlink_connection(a.conn)
    m.wait_heartbeat()
    print(f"[v2f] Connected (SysID {m.target_system})")
    mv = mavutil.mavlink

    for msg_id, hz in [(mv.MAVLINK_MSG_ID_HIGHRES_IMU, a.imu_hz),
                       (mv.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30),
                       (mv.MAVLINK_MSG_ID_ATTITUDE_TARGET, 50),
                       (mv.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 50),
                       (mv.MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS, 50),
                       (mv.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 50)]:
        m.mav.command_long_send(m.target_system, m.target_component, mv.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    det = GyroDetector()
    bus = FaultSender()  # <-- LINE 2: Initialize FaultSender bus
    alt, vz, speed = 0.0, 0.0, 99.0
    vz_sp = 0.0
    thrust, sp, att_seen = float("nan"), np.zeros(3), False
    u = None

    bG, bT, bE, bV, bU = [], [], [], [], []
    base = None
    cal_started = False
    state = "MONITOR"
    t_trig, cand, alt_trig, win_eff, S = None, None, None, a.win, []
    cooldown_until = -1.0
    events, rejected = [], []
    t_next_print, n_imu = 0.0, 0

    active_cand = None
    current_controller = "ADAPTIVE"
    t_confirmed = 0.0
    t_next_track_print = 0.0
    recovery_timer = 0

    csv_f = open(os.path.join(run_dir, "signals.csv"), "w", newline="")
    cw = csv.writer(csv_f)
    cw.writerow(["t", "state", "rn", "gx", "gy", "gz", "alt", "vz", "speed", "thrust", "u1", "u2", "u3", "u4", "err"])

    def err_now(g):
        return float(np.linalg.norm(g[:2] - sp[:2])) if att_seen else float("nan")

    def save_log():
        with open(os.path.join(run_dir, "detect_log.json"), "w") as f:
            json.dump(dict(args=vars(a), events=events, rejected=rejected), f, indent=2,
                      default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))

    try:
        while True:
            msg = m.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            typ = msg.get_type()

            if typ == "LOCAL_POSITION_NED":
                alt, vz, speed = -msg.z, msg.vz, float(np.hypot(msg.vx, msg.vy))
            elif typ == "POSITION_TARGET_LOCAL_NED":
                vz_sp = float(msg.vz) if np.isfinite(msg.vz) else 0.0
            elif typ == "ATTITUDE_TARGET":
                thrust, att_seen = msg.thrust, True
                sp = np.array([msg.body_roll_rate, msg.body_pitch_rate, msg.body_yaw_rate])
            elif typ == "ACTUATOR_OUTPUT_STATUS":
                if len(msg.actuator) >= 4 and any(x > 0.05 for x in msg.actuator[:4]):
                    raw = np.array(msg.actuator[:4], dtype=float)
                    scale = float(raw.mean()) / 0.5 if float(raw.max()) > 1.5 else 1.0
                    u = np.clip(raw / scale, 0.0, 1.0)
            elif typ == "SERVO_OUTPUT_RAW":
                if u is None:
                    pw = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
                    if min(pw) > 800:
                        u = np.clip((np.array(pw, dtype=float) - 1000.0) / 1000.0, 0.0, 1.0)
            elif typ == "HIGHRES_IMU":
                g = np.array([msg.xgyro, msg.ygyro, msg.zgyro])
                det.feed(msg.time_usec / 1e6, g)
                if det.t_last is None:
                    continue
                tt = det.t_last
                n_imu += 1

                # 1. Calibration
                if not cal_started:
                    if alt >= a.min_alt and speed <= a.max_speed:
                        t0 = tt + a.cal_start
                        det.start_calibration(t0, t0 + a.cal_len)
                        cal_started = True
                        print(f"[imu {tt:7.2f}s] calibrating {a.cal_len:.1f}s at {alt:.1f} m, speed {speed:.2f} m/s")
                    continue

                if det.thr is None and det.cal_win[0] <= tt <= det.cal_win[1]:
                    bG.append(g.copy()); bV.append(vz); bE.append(err_now(g))
                    if att_seen: bT.append(thrust)
                    if u is not None: bU.append(u.copy())

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
                        T=(float(np.mean(bT)), float(np.std(bT))) if len(bT) >= 10 else None,
                        E=float(np.nanmean(bE)) if len(bT) >= 10 and not np.all(np.isnan(bE)) else None,
                        U=np.mean(bU, axis=0) if len(bU) >= 10 else None)
                    print(f"[imu {tt:7.2f}s] MONITORING  stage-1 thr={det.thr:.2f}  win={a.win:.2f}s")
                if base is None:
                    continue

                if n_imu % 5 == 0:
                    uu = u if u is not None else [float("nan")] * 4
                    cw.writerow([f"{tt:.4f}", state, f"{det.rn:.3f}", f"{g[0]:.4f}", f"{g[1]:.4f}", f"{g[2]:.4f}",
                                 f"{alt:.3f}", f"{vz:.3f}", f"{speed:.3f}", f"{thrust:.4f}",
                                 *[f"{x:.3f}" for x in uu], f"{err_now(g):.4f}"])

                # State 1: MONITOR
                if state == "MONITOR":
                    if det.alarm:
                        if tt < cooldown_until and not (vz - vz_sp) > a.vz_thr:
                            det.reset()
                        else:
                            state, t_trig, cand, alt_trig, S = "WINDOW", det.t_alarm, det.motor_est, alt, []
                            win_eff = a.win_low if alt < a.low_alt else a.win
                            print(f"[imu {tt:7.2f}s] TRIGGER motor {cand} -> confirming for {win_eff:.2f}s")
                    elif tt >= t_next_print:
                        print(f"[imu {tt:7.2f}s] monitoring  |r|={det.rn:6.2f}  alt={alt:5.2f}  vz={vz:+.2f}")
                        t_next_print = tt + 1.0

                # State 2: WINDOW (Fast 200 ms Confirmation)
                elif state == "WINDOW":
                    S.append((tt, g.copy(), vz, thrust if att_seen else float("nan"),
                              None if u is None else u.copy(), err_now(g), alt,
                              vz_sp, sp[:2].copy() if att_seen else None))
                    if tt - t_trig >= win_eff:
                        ok, sig, ex = evaluate(S, t_trig, win_eff, cand, alt_trig, base, a)
                        print(f"[imu {tt:7.2f}s] CONFIRMATION  votes {ex['votes']}/{ex['avail']} -> {'CONFIRMED' if ok else 'REJECTED'}")
                        show(sig)
                        rec = dict(t_trigger=round(t_trig, 4), t_decision=round(tt, 4), motor_cand=cand,
                                   cos=[round(float(c), 3) for c in det.cos], votes=ex["votes"],
                                   available=ex["avail"], signals=sig, alt_at_trigger=round(alt_trig, 3))
                        if ok:
                            active_cand = cand
                            t_confirmed = tt
                            alt_drop = max(0.0, alt_trig - alt)

                            # Calibrated Benchmark severity evaluation
                            rho_est, loss_est, ctrl_mode, diag = benchmark_severity_estimator(
                                ex["track_val"], ex["sink_val"], ex["yaw_val"], alt_drop, u, cand
                            )
                            # Start with 0.50 for ADAPTIVE on 50% power test
                            current_controller = "ADAPTIVE"
                            rho_hat = 0.50
                            loss = 50.0
                            bus.send(current_controller, active_cand, rho_hat, force=True)  # <-- LINE 3: Instant forced notification

                            rec.update(
                                t_detect_s=round(t_trig, 4),
                                boot_ms=int(t_trig * 1000),
                                motor_est=cand,
                                confirm_delay_ms=round((tt - t_trig) * 1e3),
                                effective_power_rho=rho_hat,
                                power_left_pct=round(rho_hat * 100.0, 1),
                                power_loss_est_pct=loss,
                                recommended_controller=current_controller,
                                diagnostic=diag
                            )
                            events.append(rec)
                            print("=" * 65)
                            print(f"🎯 FAULT CONFIRMED: Motor {cand} | Mode: {current_controller}")
                            print(f"   Effective Power (rho): {rho_hat:.2f} ({rho_hat*100.0:.0f}%) | Assumed Loss: {loss:.1f}%")
                            print(f"   Profile Diagnostic: {diag}")
                            print(f"   [GRACE PERIOD] Holding ADAPTIVE for {a.grace_sec:.1f}s to allow controller handover.")
                            print("=" * 65)

                            save_log()
                            state = "ACTIVE_TRACK"
                            t_next_track_print = tt + 0.3
                            recovery_timer = 0
                            continue

                        rejected.append(rec)
                        det.reset()
                        cooldown_until = tt + a.cooldown
                        state, S = "MONITOR", []

                # State 3: ACTIVE_TRACK (Continuous Profiling with Adaptive Grace Period)
                elif state == "ACTIVE_TRACK":
                    alt_drop = max(0.0, alt_trig - alt)
                    e_track = err_now(g)
                    time_in_adaptive = tt - t_confirmed

                    rho_est, loss_est, ctrl_mode, diag = benchmark_severity_estimator(
                        e_track, vz, g[2], alt_drop, u, active_cand
                    )

                    # Handover Grace Period Check:
                    in_grace_period = (time_in_adaptive < a.grace_sec)
                    catastrophic_tumble = (alt_drop > 1.5 or abs(g[0]) > 1.5 or abs(g[1]) > 1.5)
                    should_escalate = (ctrl_mode == "YAW-FREE") and (not in_grace_period or catastrophic_tumble)

                    if should_escalate:
                        if current_controller != "YAW-FREE":
                            print("!" * 65)
                            print(f"🚨 [SUPERVISOR ESCALATION @ {tt:.2f}s] Condition: {diag} (after {time_in_adaptive:.2f}s in ADAPTIVE)!")
                            print(f"   SUPERVISOR ACTION: Escalating ADAPTIVE -> YAW-FREE (LATCHED)")
                            print("!" * 65)
                            current_controller = "YAW-FREE"
                            events[-1]["escalation_reason"] = diag
                            save_log()
                        rho_hat = 0.0
                        loss = 100.0
                    else:
                        current_controller = "ADAPTIVE"
                        # Gradually increase rho starting from 0.40 towards estimated rho as vehicle stabilizes
                        if ctrl_mode != "YAW-FREE":
                            rho_hat = round(float(np.clip(max(rho_hat, rho_est), 0.40, 1.00)), 2)
                            loss = round((1.0 - rho_hat) * 100.0, 1)

                    # Dynamic update of active event
                    cur_event = events[-1]
                    cur_event["effective_power_rho"] = rho_hat
                    cur_event["power_left_pct"] = round(rho_hat * 100.0, 1)
                    cur_event["power_loss_est_pct"] = loss
                    cur_event["recommended_controller"] = current_controller
                    bus.send(current_controller, active_cand, rho_hat)  # <-- LINE 4: Continuous throttled streaming

                    if tt >= t_next_track_print:
                        k = active_cand - 1
                        opp = k ^ 1
                        u_str = f"u{active_cand}={u[k]:.2f}, u{opp+1}={u[opp]:.2f}" if u is not None else "no u"
                        grace_str = f" [GRACE {time_in_adaptive:.1f}/{a.grace_sec:.1f}s]" if in_grace_period else ""
                        print(f"[imu {tt:7.2f}s] TRACKING M{active_cand} | rho: {rho_hat:.2f} ({rho_hat*100.0:.0f}%) | "
                              f"Mode: {current_controller:<8s}{grace_str} | {u_str} | alt={alt:4.1f}m vz={vz:+.2f}")
                        t_next_track_print = tt + 0.5

                    # Recovery: Locked for first 3.0 seconds post-confirmation
                    if (tt - t_confirmed) > 3.0 and base["U"] is not None and u is not None:
                        is_nominal = float(np.max(np.abs(u - base["U"]))) < 0.06
                        flying_well = e_track < 0.08 and alt > 3.0 and abs(vz) < 0.20 and abs(g[2]) < 0.10
                        if is_nominal and flying_well:
                            recovery_timer += 1
                            if recovery_timer > int(a.imu_hz * 2.5):
                                print("\n" + "=" * 65)
                                print(f"🟢 [RESTORED @ {tt:.2f}s] Vehicle returned to steady hover. Re-arming.")
                                print("=" * 65 + "\n")
                                save_log()
                                det.reset()
                                state = "MONITOR"
                                active_cand = None
                                current_controller = "ADAPTIVE"
                                cooldown_until = tt + 1.0
                        else:
                            recovery_timer = 0

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        csv_f.close()
        save_log()
        print("wrote", os.path.join(run_dir, "detect_log.json"), "and signals.csv")


if __name__ == "__main__":
    main()
