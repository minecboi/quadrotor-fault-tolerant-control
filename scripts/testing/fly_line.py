#!/usr/bin/env python3
"""fly_line.py - tell a hovering PX4 drone to fly a straight line at a set speed, then hold.   UNTESTED.

Sends one MAV_CMD_DO_REPOSITION (192) to a point --dist metres away along --bearing
(0 = north, 90 = east) at the current altitude, with ground speed --speed m/s. PX4 switches
to Hold/reposition mode by itself and holds at the target.

Run it, wait for the ACK, and let it EXIT before starting live_detect.py: both would listen on the
same UDP port, and two listeners on one port starve each other.

  python3 fly_line.py --conn udpin:0.0.0.0:14540 --dist 100 --speed 3 --bearing 0
"""
import argparse, math, time
from pymavlink import mavutil


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--conn", default="udpin:0.0.0.0:14540")
    p.add_argument("--dist", type=float, default=100.0, help="metres to fly")
    p.add_argument("--speed", type=float, default=3.0, help="ground speed m/s")
    p.add_argument("--bearing", type=float, default=0.0, help="deg, 0 = north, 90 = east")
    a = p.parse_args()

    m = mavutil.mavlink_connection(a.conn)
    m.wait_heartbeat()
    print(f"connected (SysID {m.target_system})")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, int(1e6 / 5), 0, 0, 0, 0, 0)
    g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=10)
    if g is None:
        raise SystemExit("no GLOBAL_POSITION_INT received")
    lat, lon, alt = g.lat / 1e7, g.lon / 1e7, g.alt / 1000.0        # alt is AMSL
    b = math.radians(a.bearing)
    lat2 = lat + a.dist * math.cos(b) / 111320.0
    lon2 = lon + a.dist * math.sin(b) / (111320.0 * math.cos(math.radians(lat)))
    print(f"now {lat:.6f},{lon:.6f} alt {alt:.1f} -> target {lat2:.6f},{lon2:.6f}  ({a.dist:.0f} m @ {a.speed} m/s)")

    m.mav.command_int_send(m.target_system, m.target_component,
                           mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                           0, 0, a.speed, 0, 0, float("nan"),
                           int(lat2 * 1e7), int(lon2 * 1e7), alt)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    if ack is None:
        print("no ACK (command may still have been accepted)")
    else:
        print("ACK result:", ack.result, "(0 = accepted)")
    print(f"flight time ~{a.dist / a.speed:.0f} s. Exit done; now start live_detect.py.")


if __name__ == "__main__":
    main()
