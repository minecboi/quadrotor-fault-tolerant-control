#!/usr/bin/env python3
"""inject.py - random-time motor fault injection for PX4 SITL (Gazebo gz). UNTESTED.

Fault types (motor numbers are 1-based, as in PX4):
  complete : motor off for good            -> MAV_CMD_INJECT_FAILURE (PX4 native)
  toggle   : motor off/on repeatedly       -> MAV_CMD_INJECT_FAILURE off/ok
  power    : thrust ramps down to --level  -> cap SIM_GZ_EC_MAXn (emulated)
  prop     : thrust steps down to --level  -> cap SIM_GZ_EC_MAXn (emulated)
--level = remaining thrust fraction (0.5 = 50 % thrust left).
Thrust ~ omega^2, so the speed cap is scaled by sqrt(level).
All times are SIM time (from time_boot_ms), so a slow Gazebo GUI does not matter.
Ground truth (injection sim time, in us = ulog timestamp scale) goes to a JSON file.
The same events are also written INTO the .ulg as DEBUG_VECT markers (uORB debug_vect):
  name INJECT/STEP/RESTORE, x = motor (1-4), y = fault code (complete 0, toggle 1, power 2,
  prop 3), z = remaining thrust level.  Needs 'debug_vect 0' in logger_topics.txt (SITL:
  build/px4_sitl_default/rootfs/fs/microsd/etc/logging/logger_topics.txt), restart SITL.
"""
import argparse, json, math, random, struct, time
from pymavlink import mavutil

CMD_INJECT = 420            # MAV_CMD_INJECT_FAILURE
UNIT_MOTOR = 101            # FAILURE_UNIT_SYSTEM_MOTOR (PX4 console: "motor (101)"; 100 = battery)
F_OK, F_OFF = 0, 1          # FAILURE_TYPE_OK / FAILURE_TYPE_OFF
INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
FAULT_CODE = {"complete": 0, "toggle": 1, "power": 2, "prop": 3}   # y field of the ulog marker


class Link:
    """MAVLink connection that tracks sim time and altitude from the stream."""
    def __init__(self, conn):
        self.m = mavutil.mavlink_connection(conn)
        self.m.wait_heartbeat()
        self.t = 0.0        # sim seconds since boot
        self.alt = 0.0      # metres above start (-z in NED)

    def pump(self, timeout=1.0):
        msg = self.m.recv_match(blocking=True, timeout=timeout)
        if msg is None:
            return None
        ms = getattr(msg, "time_boot_ms", None)
        if ms is not None:
            self.t = ms / 1000.0
        if msg.get_type() == "LOCAL_POSITION_NED":
            self.alt = -msg.z
        return msg

    def wait_until(self, t_end):
        while self.t < t_end:
            self.pump()

    def mark(self, label, motor, ftype, level):
        """Marker into the .ulg: DEBUG_VECT -> uORB topic debug_vect (x=motor, y=fault code,
        z=level). Needs the line 'debug_vect 0' in logger_topics.txt, see README note."""
        self.m.mav.debug_vect_send(label.encode()[:10], int(self.t * 1e6),
                                   float(motor or 0), float(FAULT_CODE[ftype]), float(level))

    # PX4 packs int32 params as raw bits inside the float field -> bit-cast.
    def get_param(self, name):
        self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component,
                                           name.encode(), -1)
        end = time.time() + 3
        while time.time() < end:
            msg = self.pump(0.5)
            if msg and msg.get_type() == "PARAM_VALUE" and msg.param_id == name:
                v = msg.param_value
                if msg.param_type == INT32:
                    v = struct.unpack("<i", struct.pack("<f", v))[0]
                return v, msg.param_type
        raise RuntimeError(f"param {name} not found (wrong name/version?)")

    def set_param(self, name, value, ptype):
        raw = struct.unpack("<f", struct.pack("<i", int(value)))[0] if ptype == INT32 else float(value)
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                  name.encode(), raw, ptype)
        for _ in range(10):                           # read back to verify; PX4 may answer with
            got, _t = self.get_param(name)            # the previous value for a moment -> retry
            if abs(got - value) <= 1e-3 * max(1, abs(value)):
                return
            time.sleep(0.2)
        raise RuntimeError(f"{name}: wrote {value}, read back {got}")

    def failure(self, motor, ftype):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     CMD_INJECT, 0, UNIT_MOTOR, ftype, motor, 0, 0, 0, 0)
        end = time.time() + 3
        while time.time() < end:
            msg = self.pump(0.5)
            if msg and msg.get_type() == "COMMAND_ACK" and msg.command == CMD_INJECT:
                if msg.result != 0:
                    raise RuntimeError(f"injection rejected, ack result={msg.result}")
                return msg.result
        raise RuntimeError("no COMMAND_ACK for injection")


def profile(a):
    """List of (dt, action) for ONE motor, dt measured from that motor's start time."""
    if a.type == "complete":
        return [(0.0, ("off", None))]
    if a.type == "toggle":
        steps = []
        for c in range(a.cycles):
            t0 = c * (a.t_off + a.t_on)
            steps += [(t0, ("off", None)), (t0 + a.t_off, ("ok", None))]
        return steps
    if a.type == "prop":
        return [(0.0, ("scale", a.level))]
    n = 10                                           # power: linear ramp in 10 steps
    return [(a.ramp * k / n, ("scale", 1 - (1 - a.level) * k / n)) for k in range(1, n + 1)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--conn", default="udpin:0.0.0.0:14550")
    p.add_argument("--motors", type=int, nargs="+", default=[1])   # i  or  i j
    p.add_argument("--type", choices=["complete", "toggle", "power", "prop"], default="complete")
    p.add_argument("--level", type=float, default=0.5)             # remaining thrust fraction
    p.add_argument("--ramp", type=float, default=5.0)              # power: ramp time [s]
    p.add_argument("--t-off", type=float, default=2.0, dest="t_off")
    p.add_argument("--t-on", type=float, default=2.0, dest="t_on")
    p.add_argument("--cycles", type=int, default=3)
    p.add_argument("--gap", type=float, default=0.0)               # delay of 2nd motor [s]
    p.add_argument("--tmin", type=float, default=5.0)              # random delay window after
    p.add_argument("--tmax", type=float, default=15.0)             # reaching --min-alt [sim s]
    p.add_argument("--min-alt", type=float, default=5.0, dest="min_alt")
    p.add_argument("--run-s", type=float, default=20.0, dest="run_s")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--out", default="inject_log.json")
    p.add_argument("--check", action="store_true", help="only read params, inject nothing")
    a = p.parse_args()

    assert 1 <= len(a.motors) <= 2 and len(set(a.motors)) == len(a.motors)
    assert all(1 <= m <= 4 for m in a.motors) and 0.0 <= a.level <= 1.0
    seed = a.seed if a.seed is not None else random.randrange(10**6)
    rng = random.Random(seed)

    L = Link(a.conn)
    if a.check:
        for n in ["SYS_FAILURE_EN", "CA_FAILURE_MODE"] + \
                 [f"SIM_GZ_EC_{k}{i}" for i in (1, 2, 3, 4) for k in ("MIN", "MAX")]:
            try:
                print(n, L.get_param(n)[0])
            except RuntimeError as e:
                print(n, "MISSING")
        return

    scaled = a.type in ("power", "prop")
    base = {}
    if L.get_param("SYS_FAILURE_EN")[0] != 1:
        L.set_param("SYS_FAILURE_EN", 1, INT32)
    if scaled:
        for m in a.motors:
            base[m] = L.get_param(f"SIM_GZ_EC_MAX{m}")        # (value, type)

    events = []
    def log(kind, motor=None, **kw):
        events.append(dict(event=kind, motor=motor, sim_t=L.t, boot_us=int(L.t * 1e6), **kw))
        print(f"[sim {L.t:7.2f}s] {kind} motor={motor} {kw}")

    try:
        wall_end = time.time() + 180
        while L.alt < a.min_alt:                              # gate: wait for hover altitude
            L.pump()
            if time.time() > wall_end:
                raise RuntimeError("never reached --min-alt (is the drone flying?)")
        t_inj = L.t + rng.uniform(a.tmin, a.tmax)             # random injection time
        log("armed_timer", t_inject_planned=t_inj, seed=seed)

        steps = profile(a)
        timeline = sorted(((t_inj + k * a.gap + dt, mot, act)
                           for k, mot in enumerate(a.motors) for dt, act in steps),
                          key=lambda x: x[0])
        for t, mot, (kind, val) in timeline:
            L.wait_until(t)
            if kind == "off":
                L.failure(mot, F_OFF)
            elif kind == "ok":
                L.failure(mot, F_OK)
            else:                                             # scale thrust to val
                bval, btype = base[mot]
                L.set_param(f"SIM_GZ_EC_MAX{mot}", round(bval * math.sqrt(val)), btype)
            lvl = val if val is not None else (0.0 if kind == "off" else 1.0)
            label = "INJECT" if not any(e["event"] == "inject" for e in events) else "STEP"
            L.mark(label, mot, a.type, lvl)                   # marker inside the .ulg
            log("inject", mot, action=kind, level=val)
        L.wait_until(timeline[-1][0] + a.run_s)
    finally:                                                  # ALWAYS undo: params persist!
        for m in a.motors:
            try:
                if scaled:
                    L.set_param(f"SIM_GZ_EC_MAX{m}", base[m][0], base[m][1])
                else:
                    L.failure(m, F_OK)
                L.mark("RESTORE", m, a.type, 1.0)             # marker inside the .ulg
            except Exception as e:
                print("RESTORE FAILED for motor", m, e)
        first = next((e for e in events if e["event"] == "inject"), None)
        json.dump(dict(args=vars(a), seed=seed,
                       t_inject_us=int(t_inj * 1e6) if first else None,   # fault onset (ramp start)
                       events=events), open(a.out, "w"), indent=1)
        print("wrote", a.out)


if __name__ == "__main__":
    main()
