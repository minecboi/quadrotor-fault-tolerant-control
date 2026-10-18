import numpy as np
from quadrotor import Quadrotor3D
from mode_controller import ModeController, NOMINAL, ADAPTIVE, YAW_FREE

quad = Quadrotor3D()
x0 = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
hover = [0, 0, 0]                       # hold the current position

mc = ModeController(quad, initial_state=x0, n_nodes=20)
for mode, motor, rho in [(NOMINAL, None, None), (ADAPTIVE, 1, 0.5), (YAW_FREE, 1, 0.0)]:
    for _ in range(30):                 # let SQP_RTI converge, as it does at 20 Hz
        u, status = mc.solve(x0, hover, mode, motor, rho)
    eff, umax, _ = mc.mode_params(mode, motor, rho)
    print(f"{mode:9s} status={status}  u={np.round(u, 3)}  effective thrust={np.round(u*eff*umax, 3)}")
