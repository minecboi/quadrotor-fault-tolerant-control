#!/usr/bin/env python3
"""
scripts/reset_motors.py - Reset all Gazebo motor speed caps to nominal (1000 rad/s)
and clear any leftover fault injection states.
"""
import struct, time
from pymavlink import mavutil

INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32

def main():
    print("Connecting to PX4 over MAVLink (udp:127.0.0.1:14550)...")
    m = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    m.wait_heartbeat()
    print("Connected to PX4.")

    # Reset all SIM_GZ_EC_MAX to 1000
    for i in range(1, 5):
        name = f"SIM_GZ_EC_MAX{i}"
        raw = struct.unpack("<f", struct.pack("<i", 1000))[0]
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), raw, INT32)
        time.sleep(0.05)

    # Clear motor failures
    for i in range(1, 5):
        m.mav.command_long_send(m.target_system, m.target_component,
                                420, 0, 101, 0, i, 0, 0, 0, 0)
        time.sleep(0.05)

    print("All 4 motors reset to 100% nominal authority (1000 rad/s). Drone ready for takeoff!")

if __name__ == "__main__":
    main()
