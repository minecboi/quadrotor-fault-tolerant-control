import numpy as np
from quadrotor import Quadrotor3D
from mode_controller import ModeController, NOMINAL, ADAPTIVE, YAW_FREE

goal = [0.0, 0.0, -5.0]                       # NED: 5 m up
x0 = [0.3, 0.2, -4.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
mc = ModeController(Quadrotor3D(), initial_state=x0, n_nodes=20)   # compile once

def run(mode, motor=None, rho=None, T=5.0, dt=0.05):
    quad = Quadrotor3D()
    quad.set_state(pos=np.array(x0[0:3], float), angle=np.array(x0[3:7], float),
                   vel=np.array(x0[7:10], float), rate=np.array(x0[10:13], float))
    eff_true = np.ones(4)                     # plant: the failed motor really has rho (YAW-FREE: dead)
    if motor is not None:
        eff_true[motor - 1] = 0.0 if mode == YAW_FREE else rho
    worst_tilt, worst_yaw, t_solve, bad = 0.0, 0.0, 0.0, 0
    for _ in range(int(T / dt)):
        s = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
        t0 = __import__("time").time()
        u, st = mc.solve(s, goal, mode, motor, rho)
        t_solve = max(t_solve, __import__("time").time() - t0)
        bad += st != 0
        quad.update(u * eff_true, dt)         # plant: thrust = u * max_thrust * eff, clipped to [0,1]
        R = quad.angle
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (R[1]**2 + R[2]**2), -1, 1)))
        worst_tilt = max(worst_tilt, tilt)
        worst_yaw = max(worst_yaw, abs(quad.a_rate[2]))
    err = np.linalg.norm(np.array(quad.pos) - np.array(goal))
    print(f"{mode:9s} final pos error={err:.3f} m  max tilt={worst_tilt:5.1f} deg  "
          f"max |yaw rate|={worst_yaw:.2f} rad/s  max solve={t_solve*1e3:.1f} ms  failed solves={bad}")

run(NOMINAL)
run(ADAPTIVE, motor=1, rho=0.5)
run(YAW_FREE, motor=1, rho=0.0)
