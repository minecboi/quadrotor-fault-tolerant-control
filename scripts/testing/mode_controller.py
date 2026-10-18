"""
mode_controller.py - ONE acados MPC for the flight. Compiled once; the fault mode only changes parameters,
so NOMINAL -> ADAPTIVE -> YAW-FREE needs no recompilation.

Model and cost follow controller.py (unbroken), with three additions:
  eff   (4)  per-motor effectiveness in the thrust model
  umax  (4)  per-motor command ceiling (hard constraint)
  yaw_w (1)  yaw-rate weight

Modes:
  NOMINAL : eff = 1,            umax = 1,              yaw_w = 1
  ADAPTIVE: eff[motor] = rho,   umax = 1,              yaw_w = 1     (rho = given level)
  YAW-FREE: eff = 1,            umax[motor] = 0,       yaw_w = 0     (motor off, yaw not controlled)
"""
import os

import casadi as cs
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

from utils import v_dot_q, skew_symmetric, quaternion_error, decompose_quaternion
from fault_bus import NOMINAL, ADAPTIVE, YAW_FREE

N_PARAM = 26   # p_ref(17) = p(3) q(4) v(3) w(3) u_ref(4), eff(4), umax(4), yaw_w(1)


class ModeController:
    def __init__(self, quad, initial_state, n_nodes=20, t_horizon=1.0, u_hover=0.30, name="mode_quad"):
        # u_hover is the reference from controller.py; the physical hover value is about 0.30 (m*g/4 / max_thrust)
        self.quad, self.N, self.u_hover = quad, n_nodes, u_hover

        x = cs.MX.sym("x", 13)
        u = cs.MX.sym("u", 4)
        P = cs.MX.sym("p", N_PARAM)
        p_ref, eff, umax, yaw_w = P[0:17], P[17:21], P[21:25], P[25]

        # ---- dynamics (same equations as controller.py; thrust scaled by eff) ----
        q, v, w = x[3:7], x[7:10], x[10:13]
        J = quad.J
        T = u * quad.max_thrust * eff * umax
        a = v_dot_q(cs.vertcat(0.0, 0.0, -cs.sum1(T)) / quad.mass, q) + cs.vertcat(0.0, 0.0, 9.81)
        Tf = cs.mtimes(T.T, cs.MX(quad.y_f))
        Tx = cs.mtimes(T.T, cs.MX(quad.x_f))
        Tz = cs.mtimes(T.T, cs.MX(quad.z_l_tau))
        wdot = cs.vertcat((Tf + (J[1] - J[2]) * w[1] * w[2]) / J[0],
                          (Tx + (J[2] - J[0]) * w[2] * w[0]) / J[1],
                          (Tz + (J[0] - J[1]) * w[0] * w[1]) / J[2])
        xdot = cs.vertcat(v, 0.5 * cs.mtimes(skew_symmetric(w), q), a, wdot)

        model = AcadosModel()
        model.name = name
        model.x = x
        model.xdot = cs.MX.sym("xdot", 13)
        model.u = u
        model.p = P
        model.f_expl_expr = xdot

        # ---- cost (weights from controller.py q_cost / r_cost; terminal x5) ----
        pe = x[0:3] - p_ref[0:3]
        qxy, _ = decompose_quaternion(quaternion_error(q, p_ref[3:7]))
        qxy_err = qxy[1] ** 2 + qxy[2] ** 2
        ve = v - p_ref[7:10]
        we = w - p_ref[10:13]
        ue = u - p_ref[13:17]
        track = (80 * pe[0] ** 2 + 80 * pe[1] ** 2 + 800 * pe[2] ** 2 + 600 * qxy_err
                 + cs.sumsqr(ve) + 50 * (we[0] ** 2 + we[1] ** 2) + yaw_w * we[2] ** 2)
        model.cost_expr_ext_cost = track + cs.sumsqr(ue) + cs.sumsqr(u)
        model.cost_expr_ext_cost_e = 5 * track

        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = n_nodes
        ocp.dims.N = n_nodes
        ocp.solver_options.tf = t_horizon
        ocp.dims.np = N_PARAM
        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL"

        p0 = np.zeros(N_PARAM)
        p0[3] = 1.0
        p0[13:17] = u_hover
        p0[17:21] = 1.0
        p0[21:25] = 1.0
        p0[25] = 1.0
        ocp.parameter_values = p0

        ocp.constraints.x0 = np.asarray(initial_state, dtype=float)
        ocp.constraints.idxbu = np.arange(4)
        ocp.constraints.lbu = np.zeros(4)
        ocp.constraints.ubu = np.ones(4)

        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.regularize_method = "PROJECT"
        ocp.solver_options.levenberg_marquardt = 1e-2
        ocp.solver_options.qp_tol = 1e-2
        ocp.solver_options.nlp_tol = 1e-2
        ocp.solver_options.nlp_tol_eq = 1e-2
        ocp.solver_options.nlp_tol_ineq = 1e-2

        json_file = os.path.join("./", name + "_ocp.json")
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        self.last_u = np.full(4, u_hover)

    @staticmethod
    def mode_params(mode=NOMINAL, motor=None, rho=None):
        eff, umax, yaw_w = np.ones(4), np.ones(4), 1.0
        if motor is not None:
            k = int(motor) - 1
            if mode == ADAPTIVE:
                eff[k] = float(np.clip(1.0 if rho is None else rho, 0.05, 1.0))
            elif mode == YAW_FREE:
                umax[k], yaw_w = 0.0, 1.0   # keep yaw-rate penalty: limit spin, do not track heading
        return eff, umax, yaw_w

    def solve(self, state, goal, mode=NOMINAL, motor=None, rho=None):
        """Returns (u[4] normalized thrust, acados status). Keeps the last good command on failure."""
        eff, umax, yaw_w = self.mode_params(mode, motor, rho)
        pref = np.concatenate([goal, [1, 0, 0, 0], [0, 0, 0], [0, 0, 0], [self.u_hover] * 4]).astype(float)
        p = np.concatenate([pref, eff, umax, [yaw_w]])
        x0 = np.asarray(state, dtype=float)
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        for j in range(self.N + 1):
            self.solver.set(j, "p", p)
        status = self.solver.solve()
        if status == 0:
            self.last_u = np.array(self.solver.get(0, "u"), dtype=float)
        else:
            if mode == ADAPTIVE and motor is not None and rho is not None:
                fallback = np.full(4, self.u_hover)
                fallback[int(motor) - 1] = float(np.clip(self.u_hover / max(0.1, rho), 0.0, 1.0))
                self.last_u = fallback
        return self.last_u.copy(), status
