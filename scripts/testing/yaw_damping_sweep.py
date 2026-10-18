import time
import numpy as np
from quadrotor import Quadrotor3D
from controller import Controller

goal = [0.0, 0.0, -5.0]
x0 = [0.3, 0.2, -4.5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
DEAD = 2

class DampedQuad(Quadrotor3D):
    """Same plant plus a yaw-rate damping torque, the term the sim is missing."""
    def __init__(self, kd=0.0):
        super().__init__()
        self.kd = kd
    def f_rate(self, x, u, t_d):
        out = np.array(super().f_rate(x, u, t_d), dtype=float)
        out[2] += -self.kd * x[3][2] / self.J[2]
        return out

ctrl = Controller(Quadrotor3D(), t_horizon=1, n_nodes=20, initial_state=x0, broken=True, model_name="broken")

def run(kd, T=5.0, dt=0.05):
    quad = DampedQuad(kd)
    quad.set_state(pos=np.array(x0[0:3], float), angle=np.array(x0[3:7], float),
                   vel=np.array(x0[7:10], float), rate=np.array(x0[10:13], float))
    eff = np.ones(4); eff[DEAD] = 0.0
    worst_tilt, worst_yaw, t_max = 0.0, 0.0, 0.0
    for _ in range(int(T / dt)):
        s = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
        t0 = time.time()
        w_opt, _ = ctrl.run_optimization(initial_state=s, goal=goal, use_model=0)
        t_max = max(t_max, time.time() - t0)
        quad.update(np.clip(w_opt[0:4], 0.0, 1.0) * eff, dt)
        R = quad.angle
        worst_tilt = max(worst_tilt, np.degrees(np.arccos(np.clip(1 - 2 * (R[1]**2 + R[2]**2), -1, 1))))
        worst_yaw = max(worst_yaw, abs(quad.a_rate[2]))
    err = np.linalg.norm(np.array(quad.pos) - np.array(goal))
    print(f"kd={kd:<6} pos error={err:8.3f} m  max tilt={worst_tilt:6.1f} deg  "
          f"max |yaw rate|={worst_yaw:6.2f} rad/s  max solve={t_max*1e3:.1f} ms")

for kd in [0.0, 0.005, 0.02, 0.05]:
    run(kd)
