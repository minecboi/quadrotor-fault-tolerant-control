#!/usr/bin/env python3
"""
run_simulation.py  --  Task 1d baseline benchmark: quadrotor single-motor failure
==========================================================================
One command:    python3 run_simulation.py         (add --quick for a fast test)
"""
import argparse
import csv
import os
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# =====================================================================
# 1. PARAMETERS
# =====================================================================
G = 9.81
M = 1.2                         # mass [kg]
L = 0.17                        # centre-to-motor distance [m]
D = L / np.sqrt(2)              # motor offset along x and y [m]
J = np.diag([0.0123, 0.0123, 0.0224])     # inertia [kg m^2]
J_INV = np.linalg.inv(J)
KM = 0.016                      # reaction torque / thrust [m]
F_MAX = 3.0 * M * G / 4.0       # max thrust per motor -> TWR = 3
TAU_MOTOR = 0.03                # motor first-order lag [s]
K_DRAG = 0.25                   # linear translational drag [N s/m]
C_YAW = 2.0e-4                  # quadratic yaw aero drag [N m s^2/rad^2]
C_ROLLPITCH = 2.0e-3            # linear roll/pitch damping [N m s/rad]

LEVEL = 0.0   # effectiveness of the failed motor after T_FAIL (0 = dead, 1 = healthy)

POS = np.array([[+D, -D], [-D, +D], [+D, +D], [-D, -D]])     # (x_i, y_i)
SPIN = np.array([-1.0, -1.0, +1.0, +1.0])                   # yaw sign
B = np.vstack([np.ones(4), POS[:, 1], -POS[:, 0], SPIN * KM])  # 4x4 allocation

DT = 0.002          # simulation / control step [s]
T_FAIL = 2.0        # failure time [s]
T_END = 10.0        # total sim time [s]
T_DETECT = 0.08     # assumed detection latency [s]
Z0 = 10.0           # start altitude [m]
SPEEDS = [0.0, 5.0, 10.0]   # forward speed at failure [m/s]

CONTROLLERS = ["traditional", "yaw-free", "INDI", "adaptive"]
COLORS = {
    "traditional": "tab:red", 
    "yaw-free": "tab:blue", 
    "INDI": "tab:green", 
    "adaptive": "tab:orange"
}

# recovery band used by the "time to recover" KPI
BAND_POS_XY, BAND_POS_Z, BAND_TILT_DEG = 0.5, 0.3, 10.0

# controller gains
KP_POS = np.array([3.0, 3.0, 5.0])
KD_VEL = np.array([4.0, 4.0, 5.0])
MAX_TILT = np.deg2rad(35.0)             
KP_ATT, KD_ATT = 200.0, 20.0              
KP_YAW, KD_YAW = 20.0, 6.0                
KP_TILT_FT, KD_TILT_FT = 500.0, 30.0     
KP_TILT_IN, KD_TILT_IN = 500.0, 30.0     
INDI_FC = 40.0                          
FT_POS_SCALE = 0.5                      
FT_MAX_TILT = np.deg2rad(20.0)            
GYRO_COMP = False                       
R_SPIN = 35.0                           
KR_SPIN = 2.0                           

# =====================================================================
# 2. MATH HELPERS
# =====================================================================
def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw*bw - ax*bx - ay*by - az*bz,
                     aw*bx + ax*bw + ay*bz - az*by,
                     aw*by - ax*bz + ay*bw + az*bx,
                     aw*bz + ax*by - ay*bx + az*bw])


def vee(S):
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


# =====================================================================
# 3. PLANT
# =====================================================================
def deriv(s, f_cmd, alive):
    p, v, q, w, f = s[0:3], s[3:6], s[6:10], s[10:13], s[13:17]
    R = quat_to_R(q)
    f_eff = f * alive                             
    u = B @ f_eff                                   
    acc = R @ np.array([0, 0, u[0]]) / M - np.array([0, 0, G]) - K_DRAG * v / M
    tau_drag = -np.array([C_ROLLPITCH * w[0], C_ROLLPITCH * w[1], C_YAW * w[2] * abs(w[2])])
    wdot = J_INV @ (u[1:4] + tau_drag - np.cross(w, J @ w))
    qdot = 0.5 * quat_mul(q, np.array([0.0, *w]))
    fdot = (np.clip(f_cmd, 0.0, F_MAX) - f) / TAU_MOTOR
    return np.concatenate([v, acc, qdot, wdot, fdot])


def rk4(s, f_cmd, alive):
    k1 = deriv(s, f_cmd, alive)
    k2 = deriv(s + 0.5*DT*k1, f_cmd, alive)
    k3 = deriv(s + 0.5*DT*k2, f_cmd, alive)
    k4 = deriv(s + DT*k3, f_cmd, alive)
    sn = s + DT/6.0 * (k1 + 2*k2 + 2*k3 + k4)
    sn[6:10] /= np.linalg.norm(sn[6:10])
    return sn


# =====================================================================
# 4. CONTROLLERS
# =====================================================================
def outer_loop(p, v, p_ref, v_ref, soft=False):
    sc = FT_POS_SCALE if soft else 1.0
    a = sc * (KP_POS * (p_ref - p) + KD_VEL * (v_ref - v)) + (K_DRAG / M) * v
    a[2] += G
    horiz = np.linalg.norm(a[:2])
    max_h = max(a[2], 1.0) * np.tan(FT_MAX_TILT if soft else MAX_TILT)
    if horiz > max_h:
        a[:2] *= max_h / horiz
    a[2] = max(a[2], 0.2 * G)
    return a


class Controller:
    def __init__(self, kind):
        self.kind = kind
        self.f_prev = np.zeros(4)
        self.w_prev = None
        self.alpha_f = np.zeros(3)
        self.f_filt = None
        self.tau_f = np.zeros(2)
        self.health_est = np.ones(4)  # Live estimated effectiveness per motor

    def traditional(self, a_des, R, w):
        zb = a_des / np.linalg.norm(a_des)
        yb = np.cross(zb, [1.0, 0, 0]); yb /= np.linalg.norm(yb)
        xb = np.cross(yb, zb)
        Rd = np.column_stack([xb, yb, zb])
        eR = 0.5 * vee(Rd.T @ R - R.T @ Rd)
        alpha = -np.array([KP_ATT, KP_ATT, KP_YAW]) * eR - np.array([KD_ATT, KD_ATT, KD_YAW]) * w
        tau = J @ alpha + np.cross(w, J @ w)
        T = M * a_des @ R[:, 2]
        return np.linalg.solve(B, np.array([T, *tau]))

    @staticmethod
    def tilt_error(a_des, R):
        n = R[:, 2]
        nd = a_des / np.linalg.norm(a_des)
        return (R.T @ np.cross(n, nd))[:2]

    @staticmethod
    def alloc3(T, tau_z, tau_xy_des, dead):
        idx = [i for i in range(4) if i != dead]
        tau_z = np.clip(tau_z, -KM * T, KM * T)
        A2 = B[[0, 3]][:, idx]                      
        Bxy = B[1:3][:, idx]
        nf = np.linalg.svd(A2)[2][-1]               
        fp = np.linalg.pinv(A2) @ np.array([T, tau_z])
        d = Bxy @ nf
        delta = d @ (tau_xy_des - Bxy @ fp) / (d @ d)
        f = np.zeros(4); f[idx] = fp + delta * nf
        return f

    @staticmethod
    def spin_ref(dead):
        return -SPIN[dead] * R_SPIN

    def yaw_free(self, a_des, R, w, dead):
        e = self.tilt_error(a_des, R)
        alpha_xy = KP_TILT_FT * e - KD_TILT_FT * w[:2]
        tau_xy = J[0, 0] * alpha_xy + (np.cross(w, J @ w)[:2] if GYRO_COMP else 0.0)
        tau_z = J[2, 2] * KR_SPIN * (self.spin_ref(dead) - w[2]) + C_YAW * w[2] * abs(w[2])
        T = max(M * a_des @ R[:, 2], 0.0)
        return self.alloc3(T, tau_z, tau_xy, dead)

    def indi(self, a_des, R, w, f_actual, dead):
        a_lp = DT / (DT + 1.0 / (2*np.pi*INDI_FC))
        if self.w_prev is None:
            self.w_prev, self.f_filt = w.copy(), f_actual.copy()
        alpha_raw = (w - self.w_prev) / DT
        self.w_prev = w.copy()
        self.alpha_f += a_lp * (alpha_raw - self.alpha_f)
        self.f_filt += a_lp * (f_actual - self.f_filt)
        u_f = B @ self.f_filt                          
        e = self.tilt_error(a_des, R)
        alpha_des = KP_TILT_IN * e - KD_TILT_IN * w[:2]
        tau_xy = u_f[1:3] + J[0, 0] * (alpha_des - self.alpha_f[:2])
        a_z_des = KR_SPIN * (self.spin_ref(dead) - w[2])
        tau_z = u_f[3] + J[2, 2] * (a_z_des - self.alpha_f[2]) + 0.0
        T = max(M * a_des @ R[:, 2], 0.0)
        return self.alloc3(T, tau_z, tau_xy, dead)

    # ---- NEW: Adaptive Health-Scaled Pseudoinverse Allocator
    def adaptive(self, a_des, R, w, dead, detected):
        if detected:
            self.health_est[dead] = LEVEL  # Update health estimate upon detection
        
        # Build effective allocation matrix by scaling columns with health estimates
        B_eff = B * self.health_est[None, :]
        
        # Standard full attitude tracking (like traditional)
        zb = a_des / np.linalg.norm(a_des)
        yb = np.cross(zb, [1.0, 0, 0]); yb /= np.linalg.norm(yb)
        xb = np.cross(yb, zb)
        Rd = np.column_stack([xb, yb, zb])
        eR = 0.5 * vee(Rd.T @ R - R.T @ Rd)
        alpha = -np.array([KP_ATT, KP_ATT, KP_YAW]) * eR - np.array([KD_ATT, KD_ATT, KD_YAW]) * w
        tau = J @ alpha + np.cross(w, J @ w)
        T = max(M * a_des @ R[:, 2], 0.0)
        
        # Health-scaled pseudoinverse allocation: handles partial failures smoothly
        wrench_des = np.array([T, *tau])
        f = np.linalg.pinv(B_eff) @ wrench_des
        return f

    def command(self, t, s, p_ref, v_ref, dead, detected):
        p, v, q, w, f_act = s[0:3], s[3:6], s[6:10], s[10:13], s[13:17]
        R = quat_to_R(q)
        soft = detected and self.kind not in ["traditional", "adaptive"]
        a_des = outer_loop(p, v, p_ref, v_ref, soft)
        
        if self.kind == "traditional" or not detected:
            f = self.traditional(a_des, R, w)
            if self.kind == "INDI":
                self.indi(a_des, R, w, f_act, dead)
        elif self.kind == "yaw-free":
            f = self.yaw_free(a_des, R, w, dead)
        elif self.kind == "INDI":
            f = self.indi(a_des, R, w, f_act, dead)
        elif self.kind == "adaptive":
            f = self.adaptive(a_des, R, w, dead, detected)
        return np.clip(f, 0.0, F_MAX)


# =====================================================================
# 5. SIMULATION & ESTIMATES
# =====================================================================
def initial_state(speed):
    theta = np.arctan2(K_DRAG * speed, M * G)            
    q = np.array([np.cos(theta/2), 0, np.sin(theta/2), 0])
    T0 = np.hypot(M * G, K_DRAG * speed)
    s = np.zeros(17)
    s[0:3] = [0, 0, Z0]
    s[3:6] = [speed, 0, 0]
    s[6:10] = q
    s[13:17] = T0 / 4
    return s


def simulate(kind, dead, speed):
    n = int(T_END / DT)
    s = initial_state(speed)
    ctrl = Controller(kind)
    log = {k: [] for k in ["t", "p", "v", "euler", "tilt", "w", "f", "f_cmd", "pref"]}
    crashed, t_crash = False, np.nan
    for k in range(n):
        t = k * DT
        failed = t >= T_FAIL
        detected = t >= T_FAIL + T_DETECT
        alive = np.ones(4)
        if failed:
            alive[dead] = LEVEL
        p_ref = np.array([speed * t, 0.0, Z0])
        v_ref = np.array([speed, 0.0, 0.0])
        f_cmd = ctrl.command(t, s, p_ref, v_ref, dead, detected)
        R = quat_to_R(s[6:10])
        log["t"].append(t); log["p"].append(s[0:3].copy()); log["v"].append(s[3:6].copy())
        roll = np.arctan2(R[2, 1], R[2, 2]); pitch = -np.arcsin(np.clip(R[2, 0], -1, 1))
        yaw = np.arctan2(R[1, 0], R[0, 0])
        log["euler"].append([roll, pitch, yaw])
        log["tilt"].append(np.arccos(np.clip(R[2, 2], -1, 1)))
        log["w"].append(s[10:13].copy()); log["f"].append(s[13:17] * alive); log["f_cmd"].append(f_cmd)
        log["pref"].append(p_ref)
        s = rk4(s, f_cmd, alive)
        if not np.all(np.isfinite(s)) or s[2] <= 0.0:
            crashed, t_crash = True, t
            break
    out = {k: np.array(v) for k, v in log.items()}
    out["euler"][:, 2] = np.unwrap(out["euler"][:, 2])
    out.update(crashed=crashed, t_crash=t_crash, kind=kind, dead=dead, speed=speed)
    return out


def print_level_estimates(r, level, t_after=(0.08, 0.2, 0.5, 1.0)):
    t, d = r["t"], r["dead"]
    o = d ^ 1                                   
    i_fail = int(round(T_FAIL / DT))
    if i_fail - 1 < 0 or i_fail >= len(t):
        return
    f_pre = r["f_cmd"][i_fail - 1, d]           
    print(f"  level estimate, motor {d+1}: actual = {level:.2f}")
    for ta in t_after:
        i = i_fail + int(round(ta / DT))
        if i + 13 >= len(t):
            break
        try:
            alpha = 2.0 * r["tilt"][i] / ta**2
            rho1 = np.clip(1.0 - J[0, 0] * alpha / (L * f_pre), 0.0, 1.0) if f_pre > 1e-6 else np.nan
            w = slice(max(i - 12, i_fail), i + 13)  
            fd = r["f_cmd"][w, d].mean()
            rho2 = np.clip(r["f_cmd"][w, o].mean() / fd, 0.0, 1.0) if fd > 1e-6 else np.nan
            print(f"    t+{ta:4.2f}s  tilt-based = {rho1:5.2f}   opposite-motor = {rho2:5.2f}")
        except Exception:
            pass


# =====================================================================
# 6. KPIs
# =====================================================================
def kpis(r):
    t = r["t"]; post = t >= T_FAIL
    p, pref = r["p"][post], r["pref"][post]
    err = p - pref
    alt_loss = max(0.0, Z0 - np.min(r["p"][:, 2][post]))
    max_tilt = np.rad2deg(np.max(r["tilt"][post]))
    drift = float(np.max(np.hypot(err[:, 0], err[:, 1])))
    if r["crashed"]:
        t_rec, spin = np.nan, np.nan
    else:
        ok = (np.hypot(err[:, 0], err[:, 1]) < BAND_POS_XY) & (np.abs(err[:, 2]) < BAND_POS_Z) \
             & (np.rad2deg(r["tilt"][post]) < BAND_TILT_DEG)
        bad = np.where(~ok)[0]
        tp = t[post]
        if len(bad) == 0:
            t_rec = 0.0
        elif bad[-1] == len(ok) - 1:
            t_rec = np.nan
        else:
            t_rec = float(tp[bad[-1] + 1] - T_FAIL)
        last = t >= T_END - 1.0
        spin = float(np.mean(np.abs(r["w"][last, 2])))
    return dict(controller=r["kind"], motor=r["dead"] + 1, speed=r["speed"],
                crashed=int(r["crashed"]), alt_loss_m=alt_loss, max_tilt_deg=max_tilt,
                drift_m=drift, t_recover_s=t_rec, spin_rad_s=spin)


# =====================================================================
# 7. PLOTS
# =====================================================================
def plot_timeseries(runs, speed, motor, outdir):
    fig, ax = plt.subplots(4, len(CONTROLLERS), figsize=(18, 13), sharex=True)
    for j, kind in enumerate(CONTROLLERS):
        r = runs[(kind, motor, speed)]
        t = r["t"]
        title = f"{kind}" + ("  (CRASHED)" if r["crashed"] else "")
        ax[0, j].set_title(title, color=COLORS[kind], fontweight="bold")
        err = r["p"] - r["pref"]
        ax[0, j].plot(t, err[:, 0], label="x error"); ax[0, j].plot(t, err[:, 1], label="y error")
        ax[0, j].plot(t, r["p"][:, 2] - Z0, label="z - z0")
        ax[1, j].plot(t, np.rad2deg(r["euler"][:, 0]), label="roll")
        ax[1, j].plot(t, np.rad2deg(r["euler"][:, 1]), label="pitch")
        ax[1, j].plot(t, np.rad2deg(r["tilt"]), "k--", label="tilt")
        for i, lab in enumerate(["p (roll rate)", "q (pitch rate)", "r (yaw rate)"]):
            ax[2, j].plot(t, r["w"][:, i], label=lab)
        for i in range(4):
            ax[3, j].plot(t, r["f"][:, i], label=f"motor {i+1}", lw=1.0)
        ax[3, j].axhline(F_MAX, color="gray", ls=":", lw=0.8)
        for a in ax[:, j]:
            a.axvline(T_FAIL, color="k", ls="--", lw=0.8)
            a.grid(alpha=0.3)
        ax[3, j].set_xlabel("time [s]")
    ylabels = ["position error [m]", "angle [deg]", "body rate [rad/s]", "rotor thrust [N]"]
    for i, yl in enumerate(ylabels):
        ax[i, 0].set_ylabel(yl)
        ax[i, len(CONTROLLERS)-1].legend(fontsize=7, loc="upper right")
    ylim_pos = (-6, 6) if speed < 7 else (-12, 12)
    for j in range(len(CONTROLLERS)):
        ax[0, j].set_ylim(*ylim_pos); ax[1, j].set_ylim(-190, 190)
        ax[2, j].set_ylim(-60, 60)
    fig.suptitle(f"Motor {motor} fails at t = {T_FAIL:.1f} s | speed {speed:g} m/s", fontsize=13)
    fig.tight_layout()
    fn = os.path.join(outdir, f"timeseries_motor{motor}_{speed:g}mps.png")
    fig.savefig(fn, dpi=110); plt.close(fig)
    return fn


# =====================================================================
# 8. MAIN
# =====================================================================
def main():
    global LEVEL
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="only motor 1, speeds 0 and 10")
    ap.add_argument("--out", default="results")
    ap.add_argument("--plot-motor", type=int, default=1)
    ap.add_argument("--level", type=float, default=LEVEL)
    ap.add_argument("--estimate", action="store_true")
    args = ap.parse_args()

    LEVEL = args.level
    os.makedirs(args.out, exist_ok=True)
    motors = [0] if args.quick else [0, 1, 2, 3]
    speeds = [0.0, 10.0] if args.quick else SPEEDS

    runs, table = {}, []
    t0 = time.time()
    for speed in speeds:
        for dead in motors:
            for kind in CONTROLLERS:
                r = simulate(kind, dead, speed)
                runs[(kind, dead + 1, speed)] = r
                row = kpis(r); table.append(row)
                print(f"v={speed:4.1f} m/s  motor {dead+1}  {kind:12s} "
                      f"{'CRASH' if r['crashed'] else 'ok   '}  loss={row['alt_loss_m']:6.2f} m  "
                      f"tilt={row['max_tilt_deg']:6.1f} deg  drift={row['drift_m']:6.2f} m  "
                      f"t_rec={row['t_recover_s']:5.2f} s  spin={row['spin_rad_s']:5.1f} rad/s", flush=True)
                
                if args.estimate and kind == "traditional":
                    print_level_estimates(r, args.level)

    print(f"\nsimulated {len(table)} runs in {time.time()-t0:.0f} s")

    with open(os.path.join(args.out, "kpi_table.csv"), "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=list(table[0].keys()))
        wr.writeheader(); wr.writerows(table)
    for speed in speeds:
        plot_motor = args.plot_motor if args.plot_motor - 1 in motors else motors[0] + 1
        print("saved", plot_timeseries(runs, speed, plot_motor, args.out))

    print("saved", os.path.join(args.out, "kpi_table.csv"))


if __name__ == "__main__":
    main()
