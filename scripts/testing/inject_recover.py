#!/usr/bin/env python3
"""px4_live_inject_detect_recover.py -- PX4 SITL + Gazebo (gz): hover -> inject partial motor loss
-> detect IN THE LOOP -> health-scaled (adaptive) reallocation with the GIVEN level.   UNTESTED on PX4.

Put in the same folder as inject.py and detect.py (both are imported, not modified).

RUN ORDER
  1. start PX4 SITL + Gazebo as usual, take off and hover above --min-alt (default 5 m)
     (inject.py's gate waits for this; e.g. in pxh>  commander takeoff   or  QGC takeoff)
  2. python3 px4_live_inject_detect_recover.py --check          # verify the param names exist
  3. python3 px4_live_inject_detect_recover.py --motors 1 --level 0.5
  Screen-record the Gazebo window for the video; the console prints each stage live.

WHAT HAPPENS (one loop, one MAVLink connection)
  * HIGHRES_IMU gyro is streamed at ~250 Hz into a streaming copy of detect.py's detector
    (same LP chain FC=30 Hz, K_SIGMA, THR_MIN, N_CONSEC, same DIRS isolation). It calibrates on a
    healthy window first. Only gyro is available over MAVLink, so it runs detect.py's fallback mode
    'plain angular-acceleration threshold' (g = 0, no torque-setpoint term).
  * After calibration + random delay, the fault is injected exactly as inject.py does it
    (--type prop: SIM_GZ_EC_MAXn scaled by sqrt(level)). Reuses inject.Link, so the same code path.
  * On alarm, RECOVERY = scale CA_ROTOR{k}_CT of the ISOLATED motor (not ground truth) by --level
    (given, not estimated). PX4's control allocator rebuilds its effectiveness matrix with that
    column scaled = same idea as run_simulation.Controller.adaptive (pinv(B * health)).
  * Everything is restored in 'finally' (params persist in SITL!).
  * Markers INJECT / DETECT / ADAPT go into the .ulg (needs 'debug_vect 0' in logger_topics.txt).
Caveat: PX4's allocator is not literally your Python adaptive controller; compare the .ulg
(detect.py LOG.ulg inject_log.json) with the sim numbers rather than assuming they match.
"""
import argparse, json, math, random, time
import numpy as np
import detect as D                      # DIRS, FC, K_SIGMA, THR_MIN, N_CONSEC

CT_PARAM = "CA_ROTOR{}_CT"              # 0-based rotor index in PX4 (motor 1 -> CA_ROTOR0_CT)


class VLP:
    """Causal first-order low-pass for non-uniform sample times."""
    def __init__(self, fc):
        self.tau, self.y = 1.0 / (2 * np.pi * fc), None

    def __call__(self, x, dt):
        a = dt / (dt + self.tau)
        self.y = x.copy() if self.y is None else self.y + a * (x - self.y)
        return self.y


class GyroDetector:
    """Streaming detect.detect() in 'plain angular-acceleration' mode (g = 0)."""
    def __init__(self):
        self.lp_w, self.lp_a = VLP(D.FC), VLP(D.FC)
        self.t_last = None
        self.w_prev = None
        self.cal_win = None
        self.cal = []
        self.thr = None
        self.run, self.rbuf = 0, []
        self.alarm, self.t_alarm, self.motor_est, self.cos = False, None, None, None
        self.rn = 0.0

    def start_calibration(self, t0, t1):
        self.cal_win = (t0, t1)

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
        a_f = self.lp_a(alpha, dt)
        r = a_f[:2]
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
            self.motor_est = int(np.argmax(self.cos)) + 1          # 1-based
            self.alarm, self.t_alarm = True, t


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--conn", default="udpin:0.0.0.0:14550")
    p.add_argument("--motors", type=int, nargs=1, default=[1])
    p.add_argument("--level", type=float, default=0.5, help="thrust left; also the GIVEN health")
    p.add_argument("--imu-hz", type=float, default=250.0)
    p.add_argument("--min-alt", type=float, default=5.0, dest="min_alt")
    p.add_argument("--cal-start", type=float, default=1.0, dest="cal_start")
    p.add_argument("--cal-len", type=float, default=3.0, dest="cal_len")
    p.add_argument("--tmin", type=float, default=3.0)
    p.add_argument("--tmax", type=float, default=8.0)
    p.add_argument("--run-s", type=float, default=15.0, dest="run_s", help="seconds to fly after recovery")
    p.add_argument("--detect-timeout", type=float, default=10.0, dest="det_to")
    p.add_argument("--no-recover", action="store_true", help="inject+detect only (baseline for comparison)")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--out", default="live_log.json")
    p.add_argument("--check", action="store_true")
    a = p.parse_args()
    m = a.motors[0]
    assert 1 <= m <= 4 and 0.0 < a.level <= 1.0

    import inject as I                                   # needs pymavlink
    L = I.Link(a.conn)
    L.m.mav.command_long_send(L.m.target_system, L.m.target_component, 511, 0,   # SET_MESSAGE_INTERVAL
                              105, int(1e6 / a.imu_hz), 0, 0, 0, 0, 0)            # HIGHRES_IMU
    L.m.mav.command_long_send(L.m.target_system, L.m.target_component, 511, 0,
                              32, int(1e6 / 20), 0, 0, 0, 0, 0)                   # LOCAL_POSITION_NED 20 Hz

    det = GyroDetector()
    orig_pump = L.pump
    def pump(timeout=1.0):                               # every message, even during blocking
        msg = orig_pump(timeout)                         # param calls, feeds the detector
        if msg is not None and msg.get_type() == "HIGHRES_IMU":
            det.feed(msg.time_usec / 1e6, np.array([msg.xgyro, msg.ygyro, msg.zgyro]))
        return msg
    L.pump = pump

    if a.check:
        for n in ["SIM_GZ_EC_MAX%d" % m] + [CT_PARAM.format(k) for k in range(4)]:
            try:
                print(n, L.get_param(n)[0])
            except RuntimeError:
                print(n, "MISSING  <- fix CT_PARAM / name for your PX4 version")
        t_end = time.time() + 2
        while time.time() < t_end:
            L.pump(0.2)
        print("IMU samples seen:", "yes" if det.t_last is not None else "NO (HIGHRES_IMU not streaming)")
        return

    seed = a.seed if a.seed is not None else random.randrange(10**6)
    rng = random.Random(seed)
    max_name = f"SIM_GZ_EC_MAX{m}"
    base_max = L.get_param(max_name)
    base_ct = [L.get_param(CT_PARAM.format(k)) for k in range(4)]     # (value, type) each
    res = dict(args=vars(a), seed=seed)
    injected = recovered = False
    try:
        wall = time.time() + 180
        while L.alt < a.min_alt or det.t_last is None:                # wait for hover altitude
            L.pump()
            if time.time() > wall:
                raise RuntimeError("never reached --min-alt / no IMU stream")
        t0 = det.t_last + a.cal_start
        det.start_calibration(t0, t0 + a.cal_len)
        print(f"[imu {det.t_last:7.2f}s] HOVER  calibrating detector for {a.cal_len:.1f}s")
        while det.thr is None:
            L.pump()
        t_inj = det.t_last + rng.uniform(a.tmin, a.tmax)
        print(f"[imu {det.t_last:7.2f}s] HOVER  monitoring, threshold {det.thr:.2f} rad/s^2, injection planned t={t_inj:.2f}")
        t_stop, t_next_print = None, 0.0
        while True:
            L.pump()
            t = det.t_last
            if t >= t_next_print:
                print(f"[imu {t:7.2f}s] alt={L.alt:5.2f} m  |r|={det.rn:6.2f}  alarm={det.alarm}")
                t_next_print = t + 1.0
            if not injected and t >= t_inj:
                t_send = t
                L.set_param(max_name, round(base_max[0] * math.sqrt(a.level)), base_max[1])
                L.mark("INJECT", m, "prop", a.level)
                injected = True
                res.update(t_inject_send=t_send, t_inject_ack=det.t_last, injected_motor=m)
                print(f"[imu {t_send:7.2f}s] FAULT INJECTED  motor {m} -> {100*a.level:.0f}% thrust left")
            if injected and det.alarm and not recovered:
                k = det.motor_est
                lat = (det.t_alarm - res["t_inject_send"]) * 1e3
                print(f"[imu {det.t_alarm:7.2f}s] DETECTED  motor {k} ({'CORRECT' if k == m else 'WRONG'}), "
                      f"latency {lat:.0f} ms from injection command (cos={np.round(det.cos, 2).tolist()})")
                res.update(t_detect=det.t_alarm, latency_ms=lat, motor_est=k, isolation_correct=(k == m))
                if not a.no_recover:
                    name = CT_PARAM.format(k - 1)
                    L.set_param(name, base_ct[k - 1][0] * a.level, base_ct[k - 1][1])
                    L.mark("ADAPT", k, "prop", a.level)
                    res.update(t_adapt=det.t_last, adapt_param=name)
                    print(f"[imu {det.t_last:7.2f}s] ADAPTIVE  {name} x {a.level} (given level) -> allocator reconfigured")
                recovered = True
                t_stop = det.t_last + a.run_s
            if injected and not det.alarm and t > res["t_inject_send"] + a.det_to:
                print("NO DETECTION within timeout")
                res["detected"] = False
                break
            if t_stop is not None and t >= t_stop:
                break
    finally:                                                       # ALWAYS restore: params persist
        for name, (v, ty) in [(max_name, base_max)] + [(CT_PARAM.format(k), base_ct[k]) for k in range(4)]:
            try:
                L.set_param(name, v, ty)
            except Exception as e:
                print("RESTORE FAILED", name, e)
        try:
            L.mark("RESTORE", m, "prop", 1.0)
        except Exception:
            pass
        json.dump(res, open(a.out, "w"), indent=1)
        print("wrote", a.out)


if __name__ == "__main__":
    main()
