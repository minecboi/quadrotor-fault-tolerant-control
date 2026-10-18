#!/usr/bin/env python3
"""benchmark.py - Task 2c runner. collect/healthy need a running PX4 SITL; evaluate runs offline on saved runs.

Stage 1  collect   fly + inject (inject.py) and save one folder per run:
                   runs/run_NNN_m<motor>_<type>_<level>_<phase>/{flight.ulg, inject_log.json, meta.json}
Stage 2  evaluate  run detect.py offline on every saved run, for one or several threshold floors
                   (so "tune one thing, before/after" needs NO new flights).
Stage 0  healthy   fly without injection to measure false alarms.

Examples (run from the PX4-Autopilot folder; copy inject.py, detect.py, benchmark.py there):
  python3 benchmark.py collect  --plan "1:complete,2:complete,3:complete,4:complete,1:prop:0.5,1:power:0.3" --reps 3
  python3 benchmark.py healthy  --seconds 60 --reps 3
  python3 benchmark.py evaluate --thr-min 2.0 1.0 0.5

Flying: default is semi-manual (you take off, press Enter). --auto-fly sends arm + takeoff over
MAVLink (experimental; after a crash the simulated drone may refuse to re-arm -> restart SITL).
inject.py and this script both listen on UDP 14550, so the connection here is closed before
inject.py starts.  After each run a force-disarm is sent so PX4 closes the .ulg.
Level estimate: detect.py also estimates thrust/command of the failed motor (tilt-based and
opposite-motor); evaluate reports the error against the true effective level. For prop/power the
true level is min(1, cap/command), because the SIM_GZ_EC_MAX cap does nothing while the command is
below the cap. Constants --mass/--jxx/--arm are from memory for gz x500: check model.sdf.
Only HOVER is covered: other phases (forward, climb, yaw) are flown by you; label them with --phase.
"""
import argparse, csv, glob, json, os, shutil, subprocess, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LOGDIR = "build/px4_sitl_default/rootfs/fs/log"


# ------------------------------------------------------------------ flying helpers
def _mav(conn, fn):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(conn)
    if m.wait_heartbeat(timeout=15) is None:
        raise RuntimeError(f"no heartbeat on {conn}")
    fn(m)
    time.sleep(0.5)
    m.close()


def arm_takeoff(conn):
    def f(m):
        m.mav.command_long_send(m.target_system, m.target_component, 400, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(2)
        nan = float("nan")
        m.mav.command_long_send(m.target_system, m.target_component, 22, 0, nan, 0, 0, nan, nan, nan, nan)
    _mav(conn, f)


def force_disarm(conn):
    _mav(conn, lambda m: m.mav.command_long_send(m.target_system, m.target_component,
                                                 400, 0, 0, 21196, 0, 0, 0, 0, 0))


def newest_ulg(logdir, since):
    files = [f for f in glob.glob(os.path.join(logdir, "*", "*.ulg")) if os.path.getmtime(f) >= since]
    return max(files, key=os.path.getmtime) if files else None


def parse_plan(s):                       # "1:complete,2:prop:0.5"
    out = []
    for item in s.split(","):
        p = item.split(":")
        out.append((int(p[0]), p[1], float(p[2]) if len(p) > 2 else 0.0))
    return out


def new_run_dir(root, tag):
    n = len(glob.glob(os.path.join(root, "run_*"))) + 1
    d = os.path.join(root, f"run_{n:03d}_{tag}")
    os.makedirs(d)
    return d


def start_flight(a, what):
    if a.auto_fly:
        arm_takeoff(a.conn)
        time.sleep(a.takeoff_wait)
    else:
        input(f"[{what}] reset/take off the drone, wait until it hovers, then press Enter... ")


def finish_flight(a, d, t_start, meta):
    force_disarm(a.conn)
    time.sleep(a.flush_s)
    ulg = newest_ulg(a.logdir, t_start)
    if ulg is None:
        meta["error"] = "no new .ulg found"
    else:
        shutil.copy(ulg, os.path.join(d, "flight.ulg"))
    json.dump(meta, open(os.path.join(d, "meta.json"), "w"), indent=1)
    print("  saved", d, meta.get("error", ""))


# ------------------------------------------------------------------ stage 1 / 0
def collect(a):
    os.makedirs(a.out, exist_ok=True)
    for _ in range(a.reps):
        for motor, ftype, level in parse_plan(a.plan):
            t_start = time.time()
            d = new_run_dir(a.out, f"m{motor}_{ftype}_{level:g}_{a.phase}")
            meta = dict(motor=motor, type=ftype, level=level, phase=a.phase, healthy=False)
            start_flight(a, f"motor {motor} {ftype} {level:g} {a.phase}")
            cmd = [sys.executable, os.path.join(HERE, "inject.py"), "--conn", a.conn,
                   "--motors", str(motor), "--type", ftype, "--level", str(level),
                   "--min-alt", str(a.min_alt), "--out", os.path.join(d, "inject_log.json")]
            rc = subprocess.run(cmd).returncode
            if rc != 0:
                meta["error"] = f"inject.py exit code {rc}"
            finish_flight(a, d, t_start, meta)


def healthy(a):
    os.makedirs(a.out, exist_ok=True)
    for _ in range(a.reps):
        t_start = time.time()
        d = new_run_dir(a.out, f"healthy_{a.phase}")
        meta = dict(motor=None, type="healthy", level=1.0, phase=a.phase, healthy=True, seconds=a.seconds)
        start_flight(a, "healthy")
        time.sleep(a.seconds)
        finish_flight(a, d, t_start, meta)


# ------------------------------------------------------------------ stage 2
def evaluate(a):
    import detect
    runs = sorted(glob.glob(os.path.join(a.runs, "run_*")))
    skipped = [d for d in runs if not os.path.exists(os.path.join(d, "meta.json"))]
    if skipped:
        print(f"skipping {len(skipped)} folder(s) without meta.json (incomplete/old runs): "
              + ", ".join(os.path.basename(d) for d in skipped))
    runs = [d for d in runs if d not in skipped]
    if not runs:
        sys.exit(f"no run_* folders in {a.runs}")
    all_rows = []
    for thr in a.thr_min:
        rows = []
        for d in runs:
            meta = json.load(open(os.path.join(d, "meta.json")))
            row = dict(thr_min=thr, run=os.path.basename(d), phase=meta["phase"], type=meta["type"],
                       level=meta["level"], motor=meta["motor"], healthy=meta["healthy"],
                       detected=0, latency_ms=None, motor_est=None, correct=None,
                       false_alarm=0, monitored_s=None, error=meta.get("error"),
                       alpha_eq=None, level_est1=None, level_est2=None, level_true1=None,
                       level_true2=None, level_est=None, level_true=None, level_err=None,
                       power_loss_est=None, power_loss_true=None, power_left=None, controller=None,
                       controller_ok=None, controller_unsafe=None)
            ulg = os.path.join(d, "flight.ulg")
            if row["error"] is None:
                try:
                    ij = None if meta["healthy"] else os.path.join(d, "inject_log.json")
                    r = detect.analyze(ulg, ij, cal_alt=a.cal_alt, thr_min=thr, verbose=False,
                                       mass=a.mass, jxx=a.jxx, arm=a.arm, dt_est=a.dt_est)
                    row["monitored_s"] = r["monitored_s"]
                    if r["detected"]:
                        row["motor_est"] = r["motor_est"]
                        if meta["healthy"]:
                            row["false_alarm"] = 1
                        elif r["latency_ms"] < 0:
                            row["false_alarm"] = 1                 # fired before the injection
                        else:
                            row.update(detected=1, latency_ms=r["latency_ms"], correct=int(r["isolation_correct"]))
                            for k in ("alpha_eq", "level_est1", "level_est2", "level_true1", "level_true2"):
                                row[k] = r.get(k)
                            add_level(row)
                            row["power_loss_est"] = r.get("power_loss")
                            row["power_loss_true"] = r.get("power_loss_true")
                            row["power_left"] = r.get("power_left")
                            row["controller"] = r.get("controller")
                            row["controller_ok"] = r.get("controller_ok")
                            row["controller_unsafe"] = r.get("controller_unsafe")
                except Exception as e:                             # keep going, report the failure
                    row["error"] = f"{type(e).__name__}: {e}"
            rows.append(row)
        all_rows += rows
        summarize(rows, thr, a.out_prefix)
    with open(a.out_prefix + "results.csv", "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(all_rows[0].keys()))
        wr.writeheader()
        wr.writerows(all_rows)
    print("wrote", a.out_prefix + "results.csv")


def add_level(row):
    """Combine the two level estimates (mean of those available) and the matching true values."""
    ok = [(e, t) for e, t in ((row["level_est1"], row["level_true1"]), (row["level_est2"], row["level_true2"]))
          if e is not None and t is not None]
    if ok:
        row["level_est"] = float(np.mean([e for e, _ in ok]))
        row["level_true"] = float(np.mean([t for _, t in ok]))
        row["level_err"] = row["level_est"] - row["level_true"]


def level_summary(det, prefix, thr):
    """Estimated vs true thrust level (thrust/command of the failed motor). toggle has no level."""
    def mae(pairs):
        pairs = [(e, t) for e, t in pairs if e is not None and t is not None]
        return (float(np.mean([abs(e - t) for e, t in pairs])), len(pairs)) if pairs else (None, 0)
    pl = [(r["power_loss_est"], r["power_loss_true"]) for r in det
          if r["power_loss_est"] is not None and r["power_loss_true"] is not None]
    print("estimated POWER LOSS (1 - opposite-motor level), error in percentage points: "
          + ("n/a (no valid estimate)" if not pl else
             f"mean abs {100 * np.mean([abs(e - t) for e, t in pl]):.1f} pp over {len(pl)} run(s); "
             f"runs without a valid estimate: {len(det) - len(pl)}"))
    for r in det:
        if r["power_loss_est"] is not None:
            print(f"    {r['run']}: est {100 * r['power_loss_est']:.0f} %  true "
                  + ("n/a" if r["power_loss_true"] is None else f"{100 * r['power_loss_true']:.0f} %"))
    ch = [r for r in det if r["controller_ok"] is not None]
    if ch:
        print(f"adaptive-vs-yaw-free choice: correct {sum(bool(r['controller_ok']) for r in ch)}/{len(ch)}; "
              f"UNSAFE (adaptive chosen where it would crash) {sum(bool(r['controller_unsafe']) for r in ch)}")
        for r in ch:
            print(f"    {r['run']}: power left "
                  + ("n/a" if r["power_left"] is None else f"{100 * r['power_left']:.0f} %")
                  + f" -> {r['controller']}  ({'OK' if r['controller_ok'] else 'WRONG'})")
    print("level estimate (thrust/command of failed motor), mean absolute error:")
    for name, ke, kt in (("tilt-based      ", "level_est1", "level_true1"),
                         ("opposite-motor  ", "level_est2", "level_true2"),
                         ("combined (mean) ", "level_est", "level_true")):
        m, n = mae([(r[ke], r[kt]) for r in det])
        print(f"  {name}: " + ("n/a (no data)" if m is None else f"{m:.3f}  over {n} run(s)"))
    ref = [r["alpha_eq"] for r in det if r["type"] == "complete" and r["alpha_eq"] is not None]
    if ref:                                              # calibrate E1 on complete failures: no J, m, arm needed
        a_ref = float(np.median(ref))
        pairs = [(float(np.clip(1 - r["alpha_eq"] / a_ref, 0, 1)), r["level_true1"]) for r in det
                 if r["type"] != "complete" and r["alpha_eq"] is not None and r["level_true1"] is not None]
        m, n = mae(pairs)
        print(f"  tilt-based, calibrated on complete runs (alpha_ref={a_ref:.1f} rad/s^2): "
              + ("n/a" if m is None else f"{m:.3f}  over {n} non-complete run(s)"))
    pts = [r for r in det if r["level_est"] is not None]
    if pts:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(5, 5))
        for ty in sorted({r["type"] for r in pts}):
            q = [r for r in pts if r["type"] == ty]
            ax.scatter([r["level_true"] for r in q], [r["level_est"] for r in q], label=ty)
        ax.plot([0, 1], [0, 1], "k--", lw=0.8)
        ax.set_xlabel("true effective level (thrust / command)")
        ax.set_ylabel("estimated level")
        ax.set_title(f"level estimate vs truth (threshold floor {thr})")
        ax.legend(); ax.grid(alpha=0.3)
        fig.tight_layout()
        fn = f"{prefix}level_est_vs_true_thr{thr:g}.png"
        fig.savefig(fn, dpi=110); plt.close(fig)
        print("saved", fn)


def summarize(rows, thr, prefix):
    ok = [r for r in rows if r["error"] is None]
    faults = [r for r in ok if not r["healthy"]]
    hl = [r for r in ok if r["healthy"]]
    det = [r for r in faults if r["detected"]]
    lat = np.array([r["latency_ms"] for r in det], float)
    print(f"\n=== threshold floor {thr} rad/s^2 ===  ({len(rows) - len(ok)} run(s) with errors skipped)")
    if faults:
        print(f"detection rate : {len(det)}/{len(faults)} = {len(det) / len(faults):.2f}")
    if len(det):
        print(f"latency [ms]   : mean {lat.mean():.1f}, worst {lat.max():.1f}")
        print(f"isolation acc. : {sum(r['correct'] for r in det)}/{len(det)}")
    fa = sum(r["false_alarm"] for r in ok)
    mon_min = (sum(r["monitored_s"] for r in hl) + sum(r["monitored_s"] for r in faults)) / 60.0
    print(f"false alarms   : {fa} (healthy flights + pre-injection time), monitored ~{mon_min:.1f} min "
          f"-> {fa / mon_min if mon_min else float('nan'):.2f}/min   [first alarm per log only]")
    if det or faults:
        cm = np.zeros((4, 5), int)                                  # cols: est 1-4, missed
        for r in faults:
            cm[r["motor"] - 1, (r["motor_est"] - 1) if r["detected"] else 4] += 1
        print("confusion matrix (rows true motor 1-4; cols est 1-4, missed):")
        print(cm)
    if det:
        level_summary(det, prefix, thr)
    phases = sorted({r["phase"] for r in det})
    types = sorted({r["type"] for r in det})
    if phases:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 4))
        w = 0.8 / max(len(types), 1)
        for i, ty in enumerate(types):
            m = []
            for ph in phases:
                v = [r["latency_ms"] for r in det if r["phase"] == ph and r["type"] == ty]
                m.append(np.mean(v) if v else np.nan)
            ax.bar(np.arange(len(phases)) + i * w, m, w, label=ty)
        ax.set_xticks(np.arange(len(phases)) + 0.4 - w / 2)
        ax.set_xticklabels(phases)
        ax.set_ylabel("mean detection latency [ms]")
        ax.set_xlabel("flight phase")
        ax.set_title(f"latency vs flight phase (threshold floor {thr})")
        ax.legend()
        ax.grid(alpha=0.3, axis="y")
        fig.tight_layout()
        fn = f"{prefix}latency_vs_phase_thr{thr:g}.png"
        fig.savefig(fn, dpi=110)
        plt.close(fig)
        print("saved", fn)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    def fly_args(p):
        p.add_argument("--conn", default="udpin:0.0.0.0:14550")
        p.add_argument("--out", default="runs")
        p.add_argument("--reps", type=int, default=1)
        p.add_argument("--phase", default="hover")
        p.add_argument("--min-alt", type=float, default=5.0, dest="min_alt")
        p.add_argument("--auto-fly", action="store_true", dest="auto_fly")
        p.add_argument("--takeoff-wait", type=float, default=15.0, dest="takeoff_wait")
        p.add_argument("--flush-s", type=float, default=4.0, dest="flush_s")
        p.add_argument("--logdir", default=LOGDIR)
    c = sub.add_parser("collect"); fly_args(c)
    c.add_argument("--plan", required=True, help='e.g. "1:complete,2:prop:0.5,3:power:0.3"')
    h = sub.add_parser("healthy"); fly_args(h)
    h.add_argument("--seconds", type=float, default=60.0)
    e = sub.add_parser("evaluate")
    e.add_argument("--runs", default="runs")
    e.add_argument("--thr-min", type=float, nargs="+", default=[2.0], dest="thr_min")
    e.add_argument("--cal-alt", type=float, default=4.0, dest="cal_alt")
    e.add_argument("--out-prefix", default="", dest="out_prefix")
    e.add_argument("--mass", type=float, default=2.0, help="vehicle mass [kg] for the tilt-based level estimate")
    e.add_argument("--jxx", type=float, default=0.02167, help="roll/pitch inertia [kg m^2]")
    e.add_argument("--arm", type=float, default=0.248, help="centre-to-motor distance [m]")
    e.add_argument("--dt-est", type=float, default=0.06, dest="dt_est",
                   help="read time after onset for the level estimators [s] (0.06 or 0.08)")
    a = ap.parse_args()
    {"collect": collect, "healthy": healthy, "evaluate": evaluate}[a.cmd](a)


if __name__ == "__main__":
    main()
