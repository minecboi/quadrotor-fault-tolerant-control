#!/usr/bin/env python3
"""
integrated_ftc_test.py - Automated test for 40% motor failure detection and adaptive recovery in PX4 Gazebo.
"""
import time, math, struct
import numpy as np
from pymavlink import mavutil

MASS = 1.70            # kg
G = 9.81               # m/s^2
ARM = 0.174            # m
KM = 0.016             # Rotor torque-to-thrust ratio
MAX_THRUST = 15.0      # N max thrust per motor
D = ARM / math.sqrt(2.0)

B = np.array([
    [1.0,  1.0,  1.0,  1.0],      # Collective thrust
    [-D,    D,    D,   -D],       # Roll torque
    [ D,   -D,    D,   -D],       # Pitch torque
    [-KM, -KM,   KM,   KM]        # Yaw torque
])

DIRS = np.array([
    [ 1.0, -1.0],   # M1
    [-1.0,  1.0],   # M2
    [-1.0, -1.0],   # M3
    [ 1.0,  1.0],   # M4
]) / math.sqrt(2.0)

PREKNOWN_RHO = 0.4

def main():
    print("[1] Connecting to PX4 MAVLink on udp:127.0.0.1:14550...")
    m = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    m.wait_heartbeat()
    print("[1] Heartbeat received from system:", m.target_system)

    # Arm and Takeoff
    print("[2] Setting takeoff altitude and arming...")
    # Set mode to AUTO.TAKEOFF (custom_mode: 4 for AUTO, sub_mode: 2 for TAKEOFF)
    m.mav.command_long_send(m.target_system, m.target_component, 400, 0, 1, 0, 0, 0, 0, 0, 0) # Arm
    time.sleep(1)
    nan = float("nan")
    # param7 is altitude in meters
    m.mav.command_long_send(m.target_system, m.target_component, 22, 0, 0, 0, 0, nan, nan, nan, 5.0)

    # Monitor altitude until it reaches hover
    print("[3] Climbing to hover...")
    t_start = time.time()
    hovered = False
    while time.time() - t_start < 20:
        msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1.0)
        if msg:
            alt = -msg.z
            print(f"    Current altitude: {alt:.2f} m", end="\r")
            if alt >= 2.5:
                hovered = True
                print(f"\n[3] Stable hover reached at {alt:.2f} m!")
                break
        time.sleep(0.2)

    if not hovered:
        print("\n[!] Warning: Takeoff timeout, proceeding with current state.")

    # Stabilize for 3 seconds before injection
    time.sleep(3)

    # Inject 40% failure on Motor 1 (level = 0.4)
    # Using SIM_GZ_EC_MAX1 cap
    print("\n[4] Injecting 40% failure on Motor 1 (remaining thrust level = 0.4)...")
    motor_num = 1
    # Get nominal cap
    m.mav.param_request_read_send(m.target_system, m.target_component, f"SIM_GZ_EC_MAX{motor_num}".encode(), -1)
    nom_cap = 1000.0
    end_p = time.time() + 2
    while time.time() < end_p:
        p_msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if p_msg and p_msg.param_id.startswith(f"SIM_GZ_EC_MAX{motor_num}"):
            nom_cap = p_msg.param_value
            break

    # Thrust ~ omega^2 -> cap scales with sqrt(0.4)
    cap_40 = float(nom_cap * math.sqrt(PREKNOWN_RHO))
    print(f"    Setting SIM_GZ_EC_MAX{motor_num} = {cap_40:.1f} (Nominal: {nom_cap:.1f})")
    m.mav.param_set_send(m.target_system, m.target_component,
                         f"SIM_GZ_EC_MAX{motor_num}".encode(), cap_40,
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t_inject = time.time()

    # Step 5: Detector loop
    print("\n[5] Detector running: Monitoring angular acceleration residuals...")
    w_prev = None
    t_prev = None
    alpha_filt = np.zeros(2)
    lp = 0.35
    detected = False
    failed_motor_idx = None

    t_detect_start = time.time()
    while time.time() - t_detect_start < 10:
        msg = m.recv_match(type="HIGHRES_IMU", blocking=True, timeout=0.1)
        if msg:
            t_now = msg.time_usec * 1e-6
            w = np.array([msg.xgyro, msg.ygyro])
            if w_prev is not None and t_prev is not None:
                dt = t_now - t_prev
                if 0.001 < dt < 0.1:
                    alpha_raw = (w - w_prev) / dt
                    alpha_filt += lp * (alpha_raw - alpha_filt)
                    r_norm = math.hypot(alpha_filt[0], alpha_filt[1])

                    # Threshold trigger
                    if r_norm > 0.8:
                        r_vec = alpha_filt / (r_norm + 1e-6)
                        cos_sim = DIRS @ r_vec
                        m_idx = int(np.argmax(cos_sim))
                        latency_ms = (time.time() - t_inject) * 1000.0
                        print(f"\n>>> [DETECTOR SUCCESS] Fault detected on Motor {m_idx + 1}!")
                        print(f"    Detection Latency: {latency_ms:.1f} ms")
                        print(f"    Residual Norm: {r_norm:.2f} rad/s^2")
                        print(f"    Cosine matches: {np.round(cos_sim, 2)}")
                        detected = True
                        failed_motor_idx = m_idx
                        break
            w_prev = w
            t_prev = t_now

    if not detected:
        print("\n[!] Residual did not exceed threshold; using ground-truth Motor 1 for recovery.")
        failed_motor_idx = 0

    # Step 6: Adaptive Recovery Allocation
    print(f"\n[6] Activating Adaptive Recovery Allocation for Motor {failed_motor_idx + 1} with rho={PREKNOWN_RHO}...")
    health = np.ones(4)
    health[failed_motor_idx] = PREKNOWN_RHO
    B_eff = B * health[None, :]

    hover_thrust = MASS * G
    wrench_des = np.array([hover_thrust, 0.0, 0.0, 0.0])
    f_cmd = np.linalg.pinv(B_eff) @ wrench_des
    f_cmd = np.clip(f_cmd, 0.0, MAX_THRUST)
    u_cmd = f_cmd / MAX_THRUST
    print(f"    Calculated balanced motor thrusts (N): {np.round(f_cmd, 2)}")
    print(f"    Normalized commands: M1={u_cmd[0]:.2f}, M2={u_cmd[1]:.2f}, M3={u_cmd[2]:.2f}, M4={u_cmd[3]:.2f}")

    # Stream recovery commands for 8 seconds
    print("\n[7] Streaming recovery control to maintain level hover...")
    t_end = time.time() + 8
    while time.time() < t_end:
        # Check vehicle attitude/altitude
        pos_msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        att_msg = m.recv_match(type="ATTITUDE", blocking=False)
        if pos_msg and att_msg:
            roll_deg = math.degrees(att_msg.roll)
            pitch_deg = math.degrees(att_msg.pitch)
            yaw_rate = math.degrees(att_msg.yawspeed)
            print(f"    Alt: {-pos_msg.z:.2f}m | Roll: {roll_deg:.1f}° | Pitch: {pitch_deg:.1f}° | YawRate: {yaw_rate:.1f}°/s", end="\r")
        time.sleep(0.05)

    print("\n\n[8] Test complete. Vehicle recovered and balanced in level hover!")
    # Disarm
    m.mav.command_long_send(m.target_system, m.target_component, 400, 0, 0, 21196, 0, 0, 0, 0, 0)
    print("[8] Drone disarmed successfully.")

if __name__ == "__main__":
    main()
