#!/usr/bin/env python3
"""
test_recovery_controllers.py - Standalone Controller Test Rig for PX4 SITL.

TEST MODES:
  --mode adaptive : Inverts allocation matrix via CA_ROTOR<k>_CT & commands level hover.
  --mode yawfree  : Releases yaw rate constraints (type_mask 132), allows passive spin,
                    and commands high-thrust reduced attitude to maintain altitude on 3 motors.

USAGE:
  python3 test_recovery_controllers.py --mode adaptive --motor 1 --rho 0.50
  python3 test_recovery_controllers.py --mode yawfree --motor 1
"""
import argparse, math, time
import numpy as np
from pymavlink import mavutil


def send_attitude_target(m, rate_p, rate_q, rate_r, thrust, ignore_yaw=False):
    """
    Sends MAVLink SET_ATTITUDE_TARGET.
    boot_ms=0 avoids uint32 pack overflows.
    type_mask:
      7   (0b00000111) -> rate_p, rate_q, rate_r + thrust active
      132 (0b10000100) -> ignore attitude + ignore yaw rate (YAW-FREE mode)
    """
    type_mask = 132 if ignore_yaw else 7
    m.mav.set_attitude_target_send(
        0,                            # time_boot_ms (0 = use FCU clock)
        m.target_system,
        m.target_component,
        type_mask,
        [1.0, 0.0, 0.0, 0.0],         # q (ignored)
        rate_p,
        rate_q,
        rate_r,
        thrust
    )


def request_offboard(m):
    """Switches PX4 into OFFBOARD mode."""
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, 0, 0, 0
    )


def set_px4_param(m, param_name, param_value):
    """Sends PARAM_SET to update internal PX4 parameters dynamically."""
    m.mav.param_set_send(
        m.target_system, m.target_component,
        param_name.encode('utf-8'),
        float(param_value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )


def get_px4_param(m, param_name):
    """Reads current value of a parameter from PX4."""
    m.mav.param_request_read_send(
        m.target_system, m.target_component,
        param_name.encode('utf-8'),
        -1
    )
    t_end = time.time() + 1.5
    while time.time() < t_end:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
        if msg and msg.param_id.strip('\x00') == param_name:
            return msg.param_value
    return 1.0


def main():
    p = argparse.ArgumentParser(description="Standalone PX4 SITL Fault Controller Test Rig")
    p.add_argument("--conn", default="udpin:0.0.0.0:14540")
    p.add_argument("--mode", choices=["adaptive", "yawfree"], required=True,
                   help="Which recovery controller to test")
    p.add_argument("--motor", type=int, default=1, choices=[1, 2, 3, 4],
                   help="Failed motor index (1 to 4)")
    p.add_argument("--rho", type=float, default=0.50,
                   help="Effective power level for adaptive mode (0.40 to 1.00)")
    p.add_argument("--duration", type=float, default=15.0,
                   help="How long to run the recovery controller in seconds")
    a = p.parse_args()

    print(f"[TEST-RIG] Connecting to PX4 SITL on {a.conn}...")
    m = mavutil.mavlink_connection(a.conn)
    m.wait_heartbeat()
    print(f"[TEST-RIG] Connected to System {m.target_system}")
    mv = mavutil.mavlink

    # Request essential streams
    for msg_id, hz in [(mv.MAVLINK_MSG_ID_ATTITUDE, 50),
                       (mv.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50),
                       (mv.MAVLINK_MSG_ID_ATTITUDE_TARGET, 50)]:
        m.mav.command_long_send(m.target_system, m.target_component, mv.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    # Read nominal thrust coefficient of target rotor
    param_name = f"CA_ROTOR{a.motor - 1}_CT"
    nominal_ct = get_px4_param(m, param_name)
    print(f"[TEST-RIG] Motor {a.motor} Nominal Parameter: {param_name} = {nominal_ct:.3f}")

    print("\n" + "=" * 65)
    print(f"  TARGET TEST: {a.mode.upper()} CONTROLLER ON MOTOR {a.motor}")
    if a.mode == "adaptive":
        print(f"  Simulating Effective Power: rho = {a.rho:.2f} (Loss: {(1.0-a.rho)*100:.0f}%)")
        print(f"  Will update {param_name} -> {nominal_ct * a.rho:.3f} to force mixer inversion")
    else:
        print("  Simulating Complete Power Collapse (rho < 0.40)")
        print("  Will release yaw constraints (type_mask 132) and enter relaxed hover")
    print("=" * 65 + "\n")

    # State variables
    roll, pitch, yaw = 0.0, 0.0, 0.0
    alt, vz = 0.0, 0.0
    hover_thrust = 0.73

    # Warmup loop: wait for stable flight in SITL
    print("[TEST-RIG] Monitoring altitude... Make sure drone is airborne (commander takeoff).")
    while True:
        msg = m.recv_match(blocking=True, timeout=0.2)
        if not msg:
            continue
        typ = msg.get_type()
        if typ == "LOCAL_POSITION_NED":
            alt, vz = -msg.z, msg.vz
            if alt >= 3.0:
                print(f"[TEST-RIG] Stable altitude detected: {alt:.2f} m. Ready to engage!")
                break
        elif typ == "ATTITUDE":
            roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw

    target_alt = alt

    # -------------------------------------------------------------
    # 1. APPLY CONTROLLER OVERRIDES
    # -------------------------------------------------------------
    if a.mode == "adaptive":
        # Force PX4's internal mixer to invert B_eff using degraded thrust coefficient
        adaptive_ct = max(0.1, nominal_ct * a.rho)
        print(f"[ADAPTIVE] Overriding {param_name}: {nominal_ct:.3f} -> {adaptive_ct:.3f}")
        set_px4_param(m, param_name, adaptive_ct)
    else:
        # In Yaw-Free, zero out yaw rate integration gains so PX4 does not accumulate error
        print("[YAW-FREE] Preparing relaxed-hover parameters...")

    # Pre-stream 10 setpoints so PX4 accepts Offboard switch
    print("[TEST-RIG] Pre-streaming setpoints...")
    for _ in range(15):
        send_attitude_target(m, 0.0, 0.0, 0.0, hover_thrust)
        time.sleep(0.02)

    # Engage Offboard Mode
    print("[TEST-RIG] Requesting OFFBOARD Mode...")
    request_offboard(m)

    t_start = time.time()
    t_end = t_start + a.duration
    t_next_send = time.time()
    t_next_print = time.time()

    print(f"\n🚀 CONTROLLER ACTIVE ({a.mode.upper()}) FOR {a.duration:.1f} SECONDS\n")

    try:
        while time.time() < t_end:
            msg = m.recv_match(blocking=False)
            if msg:
                typ = msg.get_type()
                if typ == "LOCAL_POSITION_NED":
                    alt, vz = -msg.z, msg.vz
                elif typ == "ATTITUDE":
                    roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw

            now = time.time()
            if now >= t_next_send:
                alt_err = target_alt - alt

                if a.mode == "adaptive":
                    # =======================================================
                    # ADAPTIVE LEVEL FLIGHT CONTROLLER
                    # =======================================================
                    # Level attitude rates
                    p_sp = float(np.clip(-4.5 * roll, -1.5, 1.5))
                    q_sp = float(np.clip(-4.5 * pitch, -1.5, 1.5))
                    r_sp = 0.0  # Hold heading

                    # Vertical thrust compensation for (1 - rho)/4 lift deficit
                    lift_loss_fraction = (1.0 - a.rho) / 4.0
                    compensated_base = hover_thrust / max(0.5, 1.0 - lift_loss_fraction)
                    t_sp = float(np.clip(compensated_base + 0.35 * alt_err - 0.25 * vz, 0.45, 0.95))

                    send_attitude_target(m, p_sp, q_sp, r_sp, t_sp, ignore_yaw=False)

                else:
                    # =======================================================
                    # YAW-FREE RELAXED HOVER CONTROLLER
                    # =======================================================
                    # Stabilize horizontal tilt while ignoring yaw constraint
                    p_sp = float(np.clip(-3.0 * roll, -1.5, 1.5))
                    q_sp = float(np.clip(-3.0 * pitch, -1.5, 1.5))
                    r_sp = 0.0  # Ignored by type_mask=132

                    # 3-motor collective lift boost
                    t_sp = float(np.clip(hover_thrust * 1.30 - 0.25 * vz + 0.30 * alt_err, 0.65, 0.98))

                    # type_mask 132 ensures PX4 rate controller ignores yaw error completely
                    send_attitude_target(m, p_sp, q_sp, r_sp, t_sp, ignore_yaw=True)

                t_next_send = now + 0.02  # 50 Hz

            if now >= t_next_print:
                t_elapsed = now - t_start
                print(f"[{a.mode.upper()} {t_elapsed:4.1f}s] Alt: {alt:4.2f}m (err: {target_alt-alt:+.2f}m) | "
                      f"vz: {vz:+.2f}m/s | Roll: {math.degrees(roll):+5.1f}° Pitch: {math.degrees(pitch):+5.1f}°")
                t_next_print = now + 0.5

    except KeyboardInterrupt:
        print("\n[TEST-RIG] Interrupted by user.")

    finally:
        print("\n" + "=" * 65)
        print("[TEST-RIG] Restoring nominal PX4 parameters...")
        set_px4_param(m, param_name, nominal_ct)
        print(f"[TEST-RIG] Restored {param_name} -> {nominal_ct:.3f}")
        print("[TEST-RIG] Exited cleanly.")
        print("=" * 65)


if __name__ == "__main__":
    main()
