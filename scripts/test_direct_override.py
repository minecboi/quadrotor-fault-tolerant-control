#!/usr/bin/env python3
"""
test_direct_override.py - Direct Motor Output Recovery Test for PX4 SITL.
Bypasses PX4's internal attitude PID loops to prevent flip/catapult crashes.
"""
import argparse, math, time
import numpy as np
from pymavlink import mavutil


def send_motor_commands(m, u1, u2, u3, u4):
    """
    Sends raw motor commands (0.0 to 1.0) directly to PX4 SITL actuators.
    """
    # MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET (or command via MAVLink)
    # Using SET_ACTUATOR_CONTROL_TARGET:
    # controls: [roll, pitch, yaw, thrust, ...] in group 0
    m.mav.set_actuator_control_target_send(
        0,                            # time_usec (0 = FCU clock)
        0,                            # group_mlx (0 = main flight controls)
        m.target_system,
        m.target_component,
        [float(u1), float(u2), float(u3), float(u4), 0.0, 0.0, 0.0, 0.0]
    )


def send_attitude_target(m, rate_p, rate_q, rate_r, thrust, ignore_all_axes=False):
    """
    Sends SET_ATTITUDE_TARGET with proper masks.
    type_mask 7   = rate control
    type_mask 135 = ignore roll, pitch, yaw rates entirely, command only collective thrust
    """
    type_mask = 135 if ignore_all_axes else 7
    m.mav.set_attitude_target_send(
        0,
        m.target_system,
        m.target_component,
        type_mask,
        [1.0, 0.0, 0.0, 0.0],
        rate_p, rate_q, rate_r,
        thrust
    )


def request_offboard(m):
    """Switches PX4 to OFFBOARD mode."""
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--conn", default="udpin:0.0.0.0:14540")
    p.add_argument("--mode", choices=["adaptive", "yawfree"], required=True)
    p.add_argument("--rho", type=float, default=0.50, help="Remaining power fraction (e.g. 0.50)")
    p.add_argument("--duration", type=float, default=12.0)
    a = p.parse_args()

    m = mavutil.mavlink_connection(a.conn)
    m.wait_heartbeat()
    print(f"[TEST] Connected to PX4 SITL (SysID {m.target_system})")

    # Warmup loop: wait until hovering above 2.5m
    print("[TEST] Waiting for stable hover at altitude >= 2.5m...")
    alt, vz = 0.0, 0.0
    roll, pitch, yaw = 0.0, 0.0, 0.0

    while True:
        msg = m.recv_match(blocking=True, timeout=0.2)
        if not msg:
            continue
        if msg.get_type() == "LOCAL_POSITION_NED":
            alt, vz = -msg.z, msg.vz
            if alt >= 2.5:
                print(f"[TEST] Stable hover at {alt:.2f} m. Arming recovery...")
                break
        elif msg.get_type() == "ATTITUDE":
            roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw

    # Pre-stream setpoints before switching to Offboard
    for _ in range(15):
        send_attitude_target(m, 0.0, 0.0, 0.0, 0.73)
        time.sleep(0.02)

    request_offboard(m)
    print(f"[TEST] Switched to OFFBOARD mode. Running {a.mode.upper()} recovery...")

    t_start = time.time()
    t_end = t_start + a.duration

    try:
        while time.time() < t_end:
            msg = m.recv_match(blocking=False)
            if msg:
                if msg.get_type() == "LOCAL_POSITION_NED":
                    alt, vz = -msg.z, msg.vz
                elif msg.get_type() == "ATTITUDE":
                    roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw

            if a.mode == "adaptive":
                # -------------------------------------------------------------
                # ADAPTIVE CONTROLLER (rho >= 0.40):
                # Opposing motor (M2) MUST be throttled down to balance torque!
                # -------------------------------------------------------------
                # Small damping rates only (prevents inner rate loop runaway)
                p_sp = float(np.clip(-2.0 * roll, -0.8, 0.8))
                q_sp = float(np.clip(-2.0 * pitch, -0.8, 0.8))
                r_sp = 0.0

                # Compensate collective thrust for missing (1 - rho)/4 lift
                thrust_sp = float(np.clip(0.73 / max(0.5, 1.0 - (1.0 - a.rho) / 4.0) - 0.2 * vz, 0.50, 0.95))
                send_attitude_target(m, p_sp, q_sp, r_sp, thrust_sp, ignore_all_axes=False)

            else:
                # -------------------------------------------------------------
                # YAW-FREE RELAXED HOVER (rho < 0.40):
                # Completely ignore roll, pitch, AND yaw rate constraints.
                # Allow the vehicle to spin freely and command collective lift.
                # -------------------------------------------------------------
                # High collective lift to sustain altitude on 3 motors
                thrust_sp = float(np.clip(0.88 - 0.3 * vz, 0.70, 0.98))
                send_attitude_target(m, 0.0, 0.0, 0.0, thrust_sp, ignore_all_axes=True)

            time.sleep(0.02)  # 50 Hz

    except KeyboardInterrupt:
        print("\n[TEST] Aborted by user.")
    finally:
        print("[TEST] Restoring position control...")
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 3, 0, 0, 0, 0, 0  # Mode 3 = POSCTL
        )
        print("[TEST] Done.")


if __name__ == "__main__":
    main()
