#!/usr/bin/env python3
"""
gazebo_ftc_demo.py - Real-time Detector and Adaptive Recovery (Hardcoded rho = 0.4)
for PX4 SITL / Gazebo.

Workflow:
1. Connects to PX4 SITL via MAVLink (udp:127.0.0.1:14550 or 14540).
2. Live Detector: Monitors IMU angular rates / acceleration.
3. Isolation: Identifies which motor failed (1, 2, 3, or 4).
4. Adaptive Recovery: Applies health-scaled pseudoinverse allocation with rho = 0.4
   to restore balance and maintain level hover (0 rad/s spin).
"""

import time
import math
import numpy as np
from pymavlink import mavutil

# ----------------- Quadrotor Physical Constants (x500 Gazebo Model) -----------------
MASS = 1.70            # kg (x500 standard)
G = 9.81               # m/s^2
ARM = 0.174            # m (radial arm length)
KM = 0.016             # Rotor torque-to-thrust ratio
MAX_THRUST = 15.0      # N max thrust per motor

# PX4 Motor directions (FRD frame):
# Motor 1: Front-Right (CW)   -> [+x, +y]
# Motor 2: Rear-Left   (CW)   -> [-x, -y]
# Motor 3: Front-Left  (CCW)  -> [+x, -y]
# Motor 4: Rear-Right  (CCW)  -> [-x, +y]
D = ARM / math.sqrt(2.0)
B = np.array([
    [1.0,  1.0,  1.0,  1.0],      # Collective thrust
    [-D,    D,    D,   -D],       # Roll torque
    [ D,   -D,    D,   -D],       # Pitch torque
    [-KM, -KM,   KM,   KM]        # Yaw torque
])

# Directional vectors for 4-quadrant isolation
# In roll-pitch residual space:
# M1: (+roll, -pitch), M2: (-roll, +pitch), M3: (-roll, -pitch), M4: (+roll, +pitch)
DIRS = np.array([
    [ 1.0, -1.0],   # M1
    [-1.0,  1.0],   # M2
    [-1.0, -1.0],   # M3
    [ 1.0,  1.0],   # M4
]) / math.sqrt(2.0)

# Preknown hardcoded health for the failed motor
PREKNOWN_RHO = 0.4

class GazeboFTC:
    def __init__(self, connection_str="udp:127.0.0.1:14550", detection_threshold=1.5):
        print(f"[FTC] Connecting to PX4 SITL on {connection_str}...")
        self.mav = mavutil.mavlink_connection(connection_str)
        self.mav.wait_heartbeat()
        print(f"[FTC] Connected! Target System={self.mav.target_system}, Component={self.mav.target_component}")

        self.threshold = detection_threshold
        self.w_prev = None
        self.t_prev = None
        self.alpha_filt = np.zeros(3)
        self.lp_alpha = 0.3   # low-pass filter factor

        self.detected = False
        self.failed_motor = None
        self.health = np.ones(4)

        # Baseline hover collective thrust
        self.hover_thrust = MASS * G  # ~16.68 N

    def compute_adaptive_allocation(self, failed_motor_idx, rho=PREKNOWN_RHO):
        """Build health-scaled allocation matrix with rho = 0.4."""
        self.health = np.ones(4)
        self.health[failed_motor_idx] = rho

        # Scale columns of B by motor health
        B_eff = B * self.health[None, :]

        # Wrench needed for level hover: [Total Thrust = mg, Tau_roll = 0, Tau_pitch = 0, Tau_yaw = 0]
        wrench_des = np.array([self.hover_thrust, 0.0, 0.0, 0.0])

        # Compute required thrust per motor via pseudoinverse
        f_cmd = np.linalg.pinv(B_eff) @ wrench_des

        # Clip thrusts to physically valid range [0, MAX_THRUST]
        f_cmd = np.clip(f_cmd, 0.0, MAX_THRUST)

        # Convert to normalized motor command [0.0, 1.0]
        u_cmd = f_cmd / MAX_THRUST
        return u_cmd

    def step_detector(self, gyro_rates, t_now):
        """
        Causal detection and 4-quadrant isolation.
        gyro_rates: [p, q, r] in rad/s
        """
        if self.w_prev is None:
            self.w_prev = np.array(gyro_rates)
            self.t_prev = t_now
            return False, None

        dt = t_now - self.t_prev
        if dt <= 0.001:
            return False, None

        w = np.array(gyro_rates)
        alpha_raw = (w - self.w_prev) / dt
        self.alpha_filt += self.lp_alpha * (alpha_raw - self.alpha_filt)

        self.w_prev = w
        self.t_prev = t_now

        # Residual norm on roll and pitch angular acceleration
        r_norm = math.hypot(self.alpha_filt[0], self.alpha_filt[1])

        if r_norm > self.threshold:
            # 4-Quadrant Isolation via cosine projection
            r_vec = self.alpha_filt[:2] / (r_norm + 1e-6)
            cos_sim = DIRS @ r_vec
            motor_idx = int(np.argmax(cos_sim))  # 0 to 3
            return True, motor_idx

        return False, None

    def send_actuator_commands(self, u_cmd):
        """
        Send direct normalized actuator commands to PX4 via MAVLink.
        Uses MAV_CMD_DO_SET_ACTUATOR or SET_ACTUATOR_CONTROL_TARGET.
        """
        # u_cmd is array of 4 floats [0.0 to 1.0]
        # In PX4 MAVLink, SET_ACTUATOR_CONTROL_TARGET or custom actuator command
        controls = [u_cmd[0], u_cmd[1], u_cmd[2], u_cmd[3], 0.0, 0.0, 0.0, 0.0]
        self.mav.mav.set_actuator_control_target_send(
            int(time.time() * 1e6),
            0,  # group_mlx (0 = main flight controls / motors)
            self.mav.target_system,
            self.mav.target_component,
            controls
        )

    def run(self):
        print("\n=======================================================")
        print("  GAZEBO FTC: Live Detector + Adaptive Recovery (rho=0.4)")
        print("  Listening for flight telemetry...")
        print("=======================================================\n")

        consecutive_triggers = 0
        trigger_required = 3

        while True:
            msg = self.mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue

            msg_type = msg.get_type()

            # Read high-rate angular velocity
            if msg_type == "HIGHRES_IMU":
                t_now = msg.time_usec * 1e-6
                gyro = [msg.xgyro, msg.ygyro, msg.zgyro]

                if not self.detected:
                    triggered, motor_idx = self.step_detector(gyro, t_now)
                    if triggered:
                        consecutive_triggers += 1
                        if consecutive_triggers >= trigger_required:
                            self.detected = True
                            self.failed_motor = motor_idx
                            print(f"\n[!] FAULT DETECTED at t = {t_now:.2f} s")
                            print(f"[!] Isolated Motor: Motor {motor_idx + 1}")
                            print(f"[!] Applied Policy: Adaptive Level Hover with rho = {PREKNOWN_RHO}")

                            # Compute adaptive allocation
                            u_recovery = self.compute_adaptive_allocation(motor_idx, PREKNOWN_RHO)
                            print(f"[!] Recovery Motor Commands: M1={u_recovery[0]:.2f}, M2={u_recovery[1]:.2f}, "
                                  f"M3={u_recovery[2]:.2f}, M4={u_recovery[3]:.2f}")
                    else:
                        consecutive_triggers = max(0, consecutive_triggers - 1)

            # Continuous recovery control loop once detected
            if self.detected:
                u_recovery = self.compute_adaptive_allocation(self.failed_motor, PREKNOWN_RHO)
                self.send_actuator_commands(u_recovery)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Live Detector and Adaptive Recovery for PX4 SITL")
    parser.add_argument("--conn", default="udp:127.0.0.1:14550", help="MAVLink connection URI")
    parser.add_argument("--thr", type=float, default=1.5, help="Detection threshold in rad/s^2")
    args = parser.parse_args()

    ftc = GazeboFTC(connection_str=args.conn, detection_threshold=args.thr)
    ftc.run()
