#!/usr/bin/env python3
"""detect.py - motor-failure detection + isolation from a PX4 .ulg (IMU gyro + torque setpoint).

Usage:  python3 detect.py LOG.ulg [inject_log.json] [--out detect_log.json]
        (pip install pyulog numpy).  Tested offline on PX4 SITL logs (hover); the level estimators
        mirror run_simulation.print_level_estimates (see estimate_level).

IDEA (frame: PX4 body FRD, x fwd, y right, z down; motors 1 FR, 2 RL, 3 FL, 4 RR)
  The rate controller asks for body torque tau_sp. A healthy vehicle answers with roll/pitch
  angular acceleration  alpha ~ g * tau_sp.  A weak/dead motor i removes thrust Delta, so the
  vehicle accelerates differently from what was asked:
        residual r = alpha_measured - g * tau_sp   (roll, pitch)
        r ~ Delta * (y_i, -x_i)/J      -> direction of r points at the faulty motor's quadrant
  1. alpha_measured = d/dt of low-passed gyro (causal), tau_sp low-passed the same way.
  2. g (rad/s^2 per unit torque_sp) is fitted by least squares on a healthy window, one value
     per axis. If the fit is poor (R^2 < R2_MIN) g := 0, i.e. plain angular-acceleration
     threshold. The mode used is printed.
  3. Decision: |r| above  mu + K_SIGMA*sigma  (healthy-window stats, floor THR_MIN) for
     N_CONSEC consecutive samples -> failure declared at the last of those samples.
  4. Isolation: motor with the largest cosine between mean r (over those samples) and
     DIRS[i]. 4 motors map to 4 distinct quadrants of (roll, pitch).
NOT done: severity (stop vs partial) and root cause - needs identified J and thrust gain.
"""
import argparse, json, sys
import numpy as np

# direction of (actual - commanded) accel in (roll, pitch) when motor i is weak, FRD frame
DIRS = np.array([[+1, -1], [-1, +1], [-1, -1], [+1, +1]], float) / np.sqrt(2)

FC = 30.0         # Hz, cut-off of the causal low-passes
TAU_M = 0.02      # s, motor lag applied to tau_sp (model of actuator response)
K_SIGMA = 8.0     # threshold = mu + K_SIGMA * sigma of the healthy residual norm
THR_MIN = 2.0     # rad/s^2, floor for the threshold
N_CONSEC = 5      # samples above threshold in a row
R2_MIN = 0.2      # below this the g-fit is rejected (g := 0)
DT = 0.004        # s, common resampling grid (250 Hz)
# Level estimate constants (gz x500: from memory, VERIFY in Tools/simulation/gz/models/x500_base/model.sdf)
MASS, JXX, ARM = 2.0, 0.02167, 0.248   # kg, kg m^2 (roll/pitch), centre-to-motor distance [m]
ADAPTIVE_MIN = 0.5   # estimated thrust left needed to choose the spin-free (adaptive) controller.
                     # sim: adaptive survives at >=0.4 kept, crashes at <=0.3; +0.1 margin for the ~0.11-0.13 PX4 estimate error
SIM_ADAPTIVE_OK = 0.4  # true level from which adaptive works in run_simulation.py (used only to SCORE the choice)
DT_EST = 0.06     # s after fault onset at which the tilt rise / motor commands are read (0.06 or 0.08; same read time as run_simulation.print_level_estimates)


def _lp(x, dt, fc):
    """Causal first-order low-pass along axis 0."""
    a = dt / (dt + 1.0 / (2 * np.pi * fc))
    y = np.empty_like(x)
    y[0] = x[0]
    for k in range(1, len(x)):
        y[k] = y[k - 1] + a * (x[k] - y[k - 1])
    return y


def detect(t, omega, tau_sp, cal_win, fc=FC, tau_m=TAU_M, k_sigma=K_SIGMA,
           thr_min=THR_MIN, n_consec=N_CONSEC, verbose=True):
    """t [s] uniform; omega (N,3) gyro rad/s; tau_sp (N,3) torque setpoint; cal_win=(t0,t1)
    healthy window [s]. Returns dict (detected False if nothing found)."""
    dt = float(np.median(np.diff(t)))
    w_f = _lp(omega, dt, fc)                                   # LP #1 on gyro
    alpha = np.vstack([np.zeros((1, 3)), np.diff(w_f, axis=0) / dt])
    alpha_f = _lp(alpha, dt, fc)                               # LP #2 on the derivative
    tau_f = _lp(_lp(tau_sp, dt, fc), dt, fc)                   # same two LPs -> same delay
    tau_f = _lp(tau_f, dt, 1.0 / (2 * np.pi * tau_m))          # + motor lag

    cal = (t >= cal_win[0]) & (t <= cal_win[1])
    if cal.sum() < 20:
        raise ValueError("calibration window has too few samples")
    g = np.zeros(2)
    for ax in range(2):                                        # roll, pitch
        a, u = alpha_f[cal, ax], tau_f[cal, ax]
        den = float(u @ u)
        if den > 1e-12:
            gi = float(a @ u) / den
            ss_res = float(np.sum((a - gi * u) ** 2))
            ss_tot = float(np.sum((a - a.mean()) ** 2)) + 1e-12
            if 1.0 - ss_res / ss_tot >= R2_MIN:
                g[ax] = gi
    mode = "model (alpha - g*tau_sp)" if np.any(g != 0) else "plain angular-acceleration threshold"

    r = alpha_f[:, :2] - g * tau_f[:, :2]
    rn = np.hypot(r[:, 0], r[:, 1])
    mu, sd = float(rn[cal].mean()), float(rn[cal].std())
    thr = max(mu + k_sigma * sd, thr_min)
    if verbose:
        print(f"mode: {mode}; g = {g}; healthy |r| mu={mu:.3f} sd={sd:.3f} -> threshold {thr:.2f} rad/s^2")

    above = rn > thr
    start = int(np.searchsorted(t, cal_win[1]))
    run = 0
    for k in range(start, len(t)):
        run = run + 1 if above[k] else 0
        if run >= n_consec:
            v = r[k - n_consec + 1:k + 1].mean(axis=0)
            i_on = k - n_consec + 1                          # onset = walk back to where r was still small
            while i_on > 0 and rn[i_on - 1] > 0.2 * thr:
                i_on -= 1
            cosv = DIRS @ (v / (np.linalg.norm(v) + 1e-12))
            m = int(np.argmax(cosv))
            return dict(detected=True, t_detect=float(t[k]), t_onset=float(t[i_on]), motor_est=m + 1,
                        cos_to_each_motor=[round(float(c), 3) for c in cosv],
                        residual_roll_pitch=[float(v[0]), float(v[1])],
                        threshold=thr, mode=mode, g=[float(x) for x in g])
    return dict(detected=False, threshold=thr, mode=mode, g=[float(x) for x in g])


# ---------------------------------------------------------------- level (thrust effectiveness) estimate
def estimate_level(t, tilt, motors, t_onset, motor_est, mass=MASS, jxx=JXX, arm=ARM,
                   dt_est=DT_EST, win=None, u_max=1.0, sat=0.95):
    """Estimate rho = actual thrust / commanded thrust of the motor the detector named.
    Same two estimators as run_simulation.print_level_estimates():
    E1 (tilt, dt): early on the missing thrust is a ~constant torque, tilt ~ 0.5*alpha*dt^2
                   -> alpha_eq = 2*tilt/dt^2, rho1 = 1 - J*alpha_eq/(arm*f_pre).
                   f_pre = thrust of the failed motor just before the fault. In the sim this is the
                   commanded thrust; here it is mg * u_d/sum(u) from the pre-fault motor commands
                   (share of the weight carried by that motor, assumes thrust ~ command).
                   Tilt is taken relative to its pre-fault value (~0 in the sim).
    E2 (opposite motor): all four ACTUAL thrusts are equal when torques are ~0, so
                   rho2 = u_opposite / u_failed, commands averaged over `win` (default: dt_est
                   +/- 24 ms, as in the sim). Invalid if the failed motor's command is saturated.
    Known limit (PX4 SITL, see results.csv): the PX4 rate loop counters the tilt within ~60-80 ms,
    so alpha_eq comes out ~10x lower than the constant-torque value and E1 is biased HIGH."""
    out = dict(alpha_eq=None, level_est1=None, level_est2=None, t_est1=None, t_est2=None,
               u_d_est1=None, u_d_est2=None, est2_note=None)
    m = motor_est - 1
    o = m ^ 1                                            # opposite motor: 0<->1, 2<->3
    i0 = int(np.searchsorted(t, t_onset))
    ie = int(np.searchsorted(t, t_onset + dt_est))
    if ie < len(t):
        base = float(tilt[max(i0 - 5, 0):i0 + 1].mean())
        theta = max(float(tilt[ie]) - base, 0.0)
        a_eq = 2.0 * theta / dt_est ** 2
        pre = np.nanmean(motors[max(i0 - 50, 0):max(i0 - 1, 1), :], axis=0)   # ~0.2 s before onset
        share = float(pre[m] / pre.sum()) if np.all(np.isfinite(pre)) and pre.sum() > 1e-6 else float("nan")
        if not (0.05 < share < 0.6):                                     # NaN/odd commands -> hover share
            share = 0.25
        f_pre = mass * 9.81 * share                                      # = mg/4 if all four equal
        out.update(alpha_eq=a_eq, level_est1=float(np.clip(1.0 - jxx * a_eq / (arm * f_pre), 0.0, 1.0)),
                   t_est1=float(t[ie]), u_d_est1=float(motors[ie, m]))
    if win is None:
        win = (dt_est - 0.024, dt_est + 0.024)
    if t[-1] >= t_onset + win[1]:
        w0, w1 = (int(x) for x in np.searchsorted(t, [t_onset + win[0], t_onset + win[1]]))
        if w1 - w0 >= 5:
            ud, uo = float(motors[w0:w1, m].mean()), float(motors[w0:w1, o].mean())
            if ud > 1e-6 and ud < sat * u_max:
                out.update(level_est2=float(np.clip(uo / ud, 0.0, 1.0)),
                           t_est2=float(t[(w0 + w1) // 2]), u_d_est2=ud)
            else:
                out["est2_note"] = "failed motor command saturated or ~0"
        else:
            out["est2_note"] = "window too short"
    # Reported power (thrust) loss of the failed motor = 1 - level, from the OPPOSITE-MOTOR
    # estimate only (the tilt estimate is not used: it is biased high in PX4, see results).
    out["power_loss"] = None if out["level_est2"] is None else 1.0 - out["level_est2"]
    out["power_left"] = out["level_est2"]                       # fraction of commanded thrust still delivered
    # Controller choice. Wrongly picking adaptive at <=0.3 crashes; wrongly picking yaw-free at >=0.4
    # only costs a spin and more altitude loss -> default to yaw-free unless the estimate is valid and high.
    out["controller"] = "adaptive" if (out["power_left"] is not None and out["power_left"] >= ADAPTIVE_MIN) else "yaw-free"
    out["power_loss_class"] = ("estimated" if out["power_loss"] is not None else
                               "severe? (failed-motor command saturated or ~0, no ratio)"
                               if out["est2_note"] else "not available")
    return out


def true_level(ftype, level, ramp, t_inj_s, t_s, cmd):
    """Effective thrust/command ratio of the injected motor at time t_s. For the SIM_GZ_EC_MAX
    emulation the cap limits thrust to `cap` (fraction of max), so rho = min(1, cap / command):
    a cap above the commanded value has NO effect. complete -> 0; toggle -> undefined (None)."""
    if ftype == "complete":
        return 0.0
    if ftype == "prop":
        cap = level
    elif ftype == "power":
        cap = 1.0 - (1.0 - level) * min(max((t_s - t_inj_s) / ramp, 0.0), 1.0)
    else:
        return None
    return float(min(1.0, cap / max(cmd, 1e-6)))


# ---------------------------------------------------------------- ulog front end
def _resample(ts_us, xyz, grid_us):
    return np.column_stack([np.interp(grid_us, ts_us, xyz[:, i]) for i in range(3)])


def load_ulog(path):
    from pyulog import ULog
    u = ULog(path)
    def topic(name):
        return next(d for d in u.data_list if d.name == name)
    w, tq, lp = topic("vehicle_angular_velocity"), topic("vehicle_torque_setpoint"), topic("vehicle_local_position")
    att, mot = topic("vehicle_attitude"), topic("actuator_motors")
    grid = np.arange(max(w.data["timestamp"][0], tq.data["timestamp"][0]),
                     min(w.data["timestamp"][-1], tq.data["timestamp"][-1]), int(DT * 1e6))
    om = _resample(w.data["timestamp"], np.column_stack([w.data[f"xyz[{i}]"] for i in range(3)]), grid)
    ta = _resample(tq.data["timestamp"], np.column_stack([tq.data[f"xyz[{i}]"] for i in range(3)]), grid)
    alt = np.interp(grid, lp.data["timestamp"], -lp.data["z"])      # NED z -> altitude
    q = np.column_stack([np.interp(grid, att.data["timestamp"], att.data[f"q[{i}]"]) for i in range(4)])
    tilt = np.arccos(np.clip(1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1.0, 1.0))   # angle of body z to vertical
    motors = np.column_stack([np.interp(grid, mot.data["timestamp"], mot.data[f"control[{i}]"]) for i in range(4)])
    return grid, om, ta, alt, tilt, motors


def analyze(ulg, inject_json=None, cal_alt=4.0, cal_start=1.0, cal_len=3.0,
            thr_min=THR_MIN, verbose=True, mass=MASS, jxx=JXX, arm=ARM, dt_est=DT_EST):
    """Run the detector on one log. Returns the result dict (adds latency/isolation if
    inject_json is given). Raises ValueError if the log cannot be analysed."""
    grid, om, ta, alt, tilt, motors = load_ulog(ulg)
    t = grid / 1e6
    idx = np.where(alt > cal_alt)[0]
    if len(idx) == 0:
        raise ValueError(f"altitude never exceeded {cal_alt} m; lower --cal-alt")
    t0 = t[idx[0]] + cal_start
    res = detect(t, om, ta, (t0, t0 + cal_len), thr_min=thr_min, verbose=verbose)
    out = dict(log=ulg, cal_window_s=[t0, t0 + cal_len], monitored_s=float(t[-1] - (t0 + cal_len)), **res)
    if res["detected"]:
        out["t_detect_us"] = int(round(res["t_detect"] * 1e6))
        out.update(estimate_level(t, tilt, motors, res["t_onset"], res["motor_est"],
                                  mass=mass, jxx=jxx, arm=arm, dt_est=dt_est))
    if inject_json:
        j = json.load(open(inject_json))
        ev = next(e for e in j["events"] if e["event"] == "inject")
        out["true_motor"], out["t_inject_us"] = ev["motor"], ev["boot_us"]
        if res["detected"]:
            out["latency_ms"] = (out["t_detect_us"] - ev["boot_us"]) / 1e3
            out["isolation_correct"] = res["motor_est"] == ev["motor"]
            ar = j["args"]
            for k in ("1", "2"):                          # true effective level at each estimate's time
                if out.get(f"level_est{k}") is not None:
                    out[f"level_true{k}"] = true_level(ar["type"], ar["level"], ar["ramp"],
                                                       j["t_inject_us"] / 1e6, out[f"t_est{k}"],
                                                       out[f"u_d_est{k}"])
            if out.get("level_true2") is not None:
                out["power_loss_true"] = 1.0 - out["level_true2"]
            lt = out.get("level_true2") if out.get("level_true2") is not None else (
                0.0 if ar["type"] == "complete" else None)
            if lt is not None:
                right = "adaptive" if lt >= SIM_ADAPTIVE_OK else "yaw-free"
                out["controller_right"] = right
                out["controller_ok"] = out["controller"] == right
                out["controller_unsafe"] = out["controller"] == "adaptive" and right == "yaw-free"
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ulg")
    ap.add_argument("inject_json", nargs="?")
    ap.add_argument("--cal-alt", type=float, default=4.0, help="altitude [m] that starts the healthy window")
    ap.add_argument("--cal-start", type=float, default=1.0, help="s after reaching cal-alt")
    ap.add_argument("--cal-len", type=float, default=3.0, help="healthy window length [s]")
    ap.add_argument("--thr-min", type=float, default=THR_MIN, help="threshold floor [rad/s^2]")
    ap.add_argument("--out", default="detect_log.json")
    a = ap.parse_args()
    try:
        out = analyze(a.ulg, a.inject_json, a.cal_alt, a.cal_start, a.cal_len, a.thr_min)
    except ValueError as e:
        sys.exit(str(e))
    if out["detected"]:
        print(f"DETECTED at boot t = {out['t_detect']:.3f} s, estimated motor {out['motor_est']}, "
              f"cos = {out['cos_to_each_motor']}")
    else:
        print("NO DETECTION (threshold", round(out["threshold"], 2), ")")
    if out.get("level_est1") is not None or out.get("level_est2") is not None:
        f = lambda v: "n/a" if v is None else f"{v:.2f}"
        print(f"level estimate: tilt-based {f(out.get('level_est1'))} (true {f(out.get('level_true1'))}), "
              f"opposite-motor {f(out.get('level_est2'))} (true {f(out.get('level_true2'))})"
              + (f"  [{out['est2_note']}]" if out.get("est2_note") else ""))
    if out["detected"]:
        pl = out.get("power_left")
        print(f"POWER LEFT in motor {out['motor_est']}: " + ("n/a (no valid ratio)" if pl is None else f"{100 * pl:.0f} %")
              + f"  -> use {out['controller'].upper()}"
              + (f"   (right choice: {out['controller_right']}, {'OK' if out['controller_ok'] else 'WRONG'}"
                 + (", UNSAFE" if out.get("controller_unsafe") else "") + ")" if "controller_ok" in out else ""))
    if out.get("power_loss") is not None:
        pt = out.get("power_loss_true")
        print(f"ESTIMATED POWER LOSS of motor {out['motor_est']}: {100 * out['power_loss']:.0f} %"
              + ("" if pt is None else f"   (true {100 * pt:.0f} %)"))
    elif out["detected"]:
        print(f"power loss: {out.get('power_loss_class')}")
    if "latency_ms" in out:
        print(f"true motor {out['true_motor']}; latency {out['latency_ms']:.1f} ms; "
              f"isolation {'CORRECT' if out['isolation_correct'] else 'WRONG'}")
        if out["latency_ms"] < 0:
            print("WARNING: negative latency = false alarm before injection (threshold too low)")
    json.dump(out, open(a.out, "w"), indent=1)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
