#!/usr/bin/env python3
"""
demo_flight.py - Complete demonstration of:
  Phase 1: Airborne with Motor 1 degraded to 0.40 thrust (Adaptive Level Hover)
  Phase 2: Motor 1 completely OFF (0.0 thrust) with Yaw-Free Recovery (Lifting Up / Flying)
"""
import time, math, struct
from pymavlink import mavutil
import numpy as np

def main():
    conn = 'udpin:0.0.0.0:14550'
    print(f"[DEMO] Connecting to PX4 on {conn}...")
    m = mavutil.mavlink_connection(conn)
    m.wait_heartbeat()
    print(f"[DEMO] Connected to System {m.target_system}")

    def set_param(name, value, ptype):
        raw = struct.unpack('<f', struct.pack('<i', int(value)))[0] if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32 else float(value)
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), raw, ptype)
        time.sleep(0.02)

    def send_attitude_target(rate_p, rate_q, rate_r, thrust, ignore_yaw=False):
        type_mask = 132 if ignore_yaw else 7
        m.mav.set_attitude_target_send(
            0, m.target_system, m.target_component,
            type_mask,
            [1.0, 0.0, 0.0, 0.0],
            rate_p, rate_q, rate_r,
            thrust
        )

    # 1. Reset parameters to healthy nominal
    set_param('SIM_GZ_EC_MAX1', 1000, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param('CA_ROTOR0_CT', 6.50, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    print("[DEMO] System initialized in healthy state.")

    # Request high-rate telemetry
    for msg_id in [mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE]:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, int(1e6 / 50), 0, 0, 0, 0, 0)

    # 2. Takeoff to 2.5m in Offboard mode
    print("\n[DEMO] Pre-streaming attitude setpoints for takeoff...")
    for _ in range(50):
        send_attitude_target(0.0, 0.0, 0.0, 0.72)
        time.sleep(0.02)

    # Engage Offboard Mode and Arm
    print("[DEMO] Engaging OFFBOARD Mode and Arming...")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1, 0, 0, 0, 0, 0, 0)

    alt, vz = 0.0, 0.0
    roll, pitch, yaw = 0.0, 0.0, 0.0

    # Climb loop to 2.5m
    print("[DEMO] Climbing to 2.5m...")
    t_climb = time.time()
    while time.time() - t_climb < 10.0:
        msg = m.recv_match(blocking=True, timeout=0.05)
        if msg:
            if msg.get_type() == 'LOCAL_POSITION_NED':
                alt, vz = -msg.z, msg.vz
            elif msg.get_type() == 'ATTITUDE':
                roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw

        # PD altitude controller for climb
        t_sp = float(np.clip(0.68 + 0.15 * (2.5 - alt) - 0.20 * vz, 0.50, 0.85))
        p_sp = float(np.clip(-3.0 * roll, -1.0, 1.0))
        q_sp = float(np.clip(-3.0 * pitch, -1.0, 1.0))
        send_attitude_target(p_sp, q_sp, 0.0, t_sp)

        if alt >= 2.3 and abs(vz) < 0.3:
            print(f"[DEMO] Reached stable hover altitude: {alt:.2f}m")
            break

    # -----------------------------------------------------------------
    # PHASE 1: ADAPTIVE FLIGHT WITH MOTOR 1 AT 0.40 THRUST
    # -----------------------------------------------------------------
    print("\n" + "=" * 65)
    print("🚀 PHASE 1: ADAPTIVE RECOVERY - MOTOR 1 DEGRADED TO 0.40 THRUST")
    print("   Action: Physically capping Motor 1 to 40% in Gazebo")
    print("   Action: Setting CA_ROTOR0_CT = 2.60 & Compensating Lift")
    print("=" * 65)

    set_param('SIM_GZ_EC_MAX1', 632, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param('CA_ROTOR0_CT', 2.60, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    t_p1 = time.time()
    t_print = time.time()
    target_alt = 2.5

    while time.time() - t_p1 < 8.0:
        msg = m.recv_match(blocking=True, timeout=0.02)
        if msg:
            if msg.get_type() == 'LOCAL_POSITION_NED':
                alt, vz = -msg.z, msg.vz
            elif msg.get_type() == 'ATTITUDE':
                roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw

        # Adaptive level flight: counter pitch/roll tilt and compensate vertical thrust deficit
        p_sp = float(np.clip(-4.5 * roll, -1.5, 1.5))
        q_sp = float(np.clip(-4.5 * pitch, -1.5, 1.5))
        # 1 motor at 0.4 means ~15% total lift deficit -> baseline thrust boosted to 0.74
        t_sp = float(np.clip(0.74 + 0.25 * (target_alt - alt) - 0.20 * vz, 0.55, 0.90))
        send_attitude_target(p_sp, q_sp, 0.0, t_sp, ignore_yaw=False)

        if time.time() >= t_print:
            print(f"[ADAPTIVE {time.time()-t_p1:4.1f}s/8s] Alt: {alt:4.2f}m | Roll: {math.degrees(roll):+5.1f}° | Pitch: {math.degrees(pitch):+5.1f}° | Motor 1 @ 40%")
            t_print = time.time() + 0.5

    # -----------------------------------------------------------------
    # PHASE 2: YAW-FREE FLIGHT WITH MOTOR 1 COMPLETELY SWITCHED OFF (0.0)
    # -----------------------------------------------------------------
    print("\n" + "=" * 65)
    print("🚨 PHASE 2: SWITCHING OFF MOTOR 1 COMPLETELY -> YAW-FREE FLIGHT")
    print("   Action: Motor 1 thrust cut to 0.0")
    print("   Action: Releasing yaw constraints (type_mask 132), boosting 3-motor lift")
    print("=" * 65)

    set_param('SIM_GZ_EC_MAX1', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

    t_p2 = time.time()
    t_print = time.time()

    while time.time() - t_p2 < 8.0:
        msg = m.recv_match(blocking=True, timeout=0.02)
        if msg:
            if msg.get_type() == 'LOCAL_POSITION_NED':
                alt, vz = -msg.z, msg.vz
            elif msg.get_type() == 'ATTITUDE':
                roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw

        # Yaw-free flight: do NOT fight yaw rate. Control roll/pitch gently, boost collective thrust
        p_sp = float(np.clip(-2.5 * roll, -1.5, 1.5))
        q_sp = float(np.clip(-2.5 * pitch, -1.5, 1.5))
        # 3 motors need ~0.85 throttle to hold/lift
        t_sp = float(np.clip(0.85 + 0.20 * (target_alt - alt) - 0.25 * vz, 0.70, 0.98))
        send_attitude_target(p_sp, q_sp, 0.0, t_sp, ignore_yaw=True)

        if time.time() >= t_print:
            print(f"[YAW-FREE {time.time()-t_p2:4.1f}s/8s] Alt: {alt:4.2f}m | vz: {vz:+.2f}m/s | Roll: {math.degrees(roll):+5.1f}° | Motor 1 = 0.0 (OFF)")
            t_print = time.time() + 0.5

    # 4. Cleanup
    print("\n" + "=" * 65)
    print("🎯 DEMO COMPLETE! Restoring nominal parameters and landing...")
    set_param('SIM_GZ_EC_MAX1', 1000, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    set_param('CA_ROTOR0_CT', 6.50, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
                            0, 0, 0, 0, 0, 0, 0)
    print("[DEMO] Exited cleanly.")
    print("=" * 65)

if __name__ == '__main__':
    main()
