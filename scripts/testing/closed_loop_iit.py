import time
import numpy as np
from quadrotor import Quadrotor3D
from controller import Controller

goal = [0.0, 0.0, -5.0]
x0 = [0.3, 0.2, -4.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
DEAD = 2                                             # motor 3 (index 2), as in IIT broken mode

ctrl_ok = Controller(Quadrotor3D(), t_horizon=1, n_nodes=20, initial_state=x0, model_name="unbroken")
ctrl_br = Controller(Quadrotor3D(), t_horizon=1, n_nodes=20, initial_state=x0, broken=True, model_name="broken")

def run(ctrl, failed, T=5.0, dt=0.05):
    quad = Quadrotor3D()
    quad.set_state(pos=np.array(x0[0:3], float), angle=np.array(x0[3:7], float),
                   vel=np.array(x0[7:10], float), rate=np.array(x0[10:13], float))
    eff = np.ones(4)
    if failed:
        eff[DEAD] = 0.0
    worst_tilt, worst_yaw, t_max = 0.0, 0.0, 0.0
    for _ in range(int(T / dt)):
        s = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
        t0 = time.time()
        w_opt, _ = ctrl.run_optimization(initial_state=s, goal=goal, use_model=0)
        t_max = max(t_max, time.time() - t0)
        u = np.clip(w_opt[0:4], 0.0, 1.0)
        quad.update(u * eff, dt)
        R = quad.angle
        worst_tilt = max(worst_tilt, np.degrees(np.arccos(np.clip(1 - 2 * (R[1]**2 + R[2]**2), -1, 1))))
        worst_yaw = max(worst_yaw, abs(quad.a_rate[2]))
    err = np.linalg.norm(np.array(quad.pos) - np.array(goal))
    print(f"{'IIT broken (motor 3 dead)' if failed else 'IIT unbroken (healthy)'}: "
          f"pos error={err:.3f} m  max tilt={worst_tilt:.1f} deg  max |yaw rate|={worst_yaw:.2f}  "
          f"max solve={t_max*1e3:.1f} ms")

run(ctrl_ok, failed=False)
run(ctrl_br, failed=True)
