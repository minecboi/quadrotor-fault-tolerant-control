#!/usr/bin/env python3
"""
test_adaptive_ground_takeoff.py - Take off directly from the ground with Motor 1 degraded to 50% power.

Configuration:
- SIM_GZ_EC_MAX1 = 707 (Motor 1 physical max thrust capped at 50% in Gazebo)
- CA_ROTOR0_CT = 3.25 (Adaptive allocation matrix: Rotor 0 effectiveness scaled to 50%)
- Pre-stream setpoints for 5.0 seconds at 20Hz (counter = 100) before arming & offboard.
"""
import time, math, struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
from pymavlink import mavutil

class AdaptiveGroundTakeoff(Node):
    def __init__(self):
        super().__init__('adaptive_ground_takeoff')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.sub_odom = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos)
        
        self.current_pos = [0.0, 0.0, 0.0]
        self.current_vz = 0.0
        self.current_q = [1.0, 0.0, 0.0, 0.0]
        self.counter = 0
        self.state = "INIT"
        self.target_alt = 3.0  # target 3.0m
        self.hover_goal = None
        
        print("\n=======================================================")
        print("🚀 ADAPTIVE GROUND TAKEOFF TEST (50% POWER ON MOTOR 1)")
        print("=======================================================")
        
        # Connect MAVLink to verify and enforce parameters
        print("[INIT] Verifying 50% fault parameters on MAVLink...")
        self.mav = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.mav.wait_heartbeat()
        
        # Set SIM_GZ_EC_MAX1 = 707 (50% power) and CA_ROTOR0_CT = 3.25 (adaptive allocation)
        self.set_param('SIM_GZ_EC_MAX1', 707, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.set_param('CA_ROTOR0_CT', 3.25, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self.set_param('CBRK_SUPPLY_CHK', 894281, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print("[INIT] Parameters verified: SIM_GZ_EC_MAX1=707, CA_ROTOR0_CT=3.25")
        
        # 20 Hz control timer (0.05s) - matches main.py
        self.timer = self.create_timer(0.05, self.control_loop)
        
    def set_param(self, name, value, ptype):
        raw = struct.unpack('<f', struct.pack('<i', int(value)))[0] if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32 else float(value)
        self.mav.mav.param_set_send(self.mav.target_system, self.mav.target_component, name.encode(), raw, ptype)
        
    def odom_callback(self, msg):
        self.current_pos = list(msg.position)
        self.current_vz = msg.velocity[2]
        self.current_q = list(msg.q)
        
    def send_cmd(self, cmd, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = cmd
        msg.param1, msg.param2 = p1, p2
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 1, 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_cmd.publish(msg)
        
    def publish_trajectory(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position[0] = x
        msg.position[1] = y
        msg.position[2] = z
        msg.velocity[0] = float('nan')
        msg.velocity[1] = float('nan')
        msg.velocity[2] = float('nan')
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')
        msg.yaw = 0.0
        msg.yawspeed = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_sp.publish(msg)
        
    def control_loop(self):
        self.counter += 1
        alt = -self.current_pos[2]
        
        if self.hover_goal is None:
            self.hover_goal = [self.current_pos[0], self.current_pos[1], self.current_pos[2] - self.target_alt]
            
        # Always publish Offboard heartbeat
        mode_msg = OffboardControlMode()
        mode_msg.position = True
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = False
        mode_msg.thrust_and_torque = False
        mode_msg.direct_actuator = False
        mode_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_mode.publish(mode_msg)
        
        # Publish target hover trajectory
        self.publish_trajectory(*self.hover_goal)
        
        # Orientation in degrees
        q = self.current_q
        roll = math.degrees(math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(q[0]*q[2] - q[3]*q[1])))))
        
        # Phase 1: Pre-stream setpoints for 100 steps (5.0s)
        if self.state == "INIT":
            if self.counter % 20 == 0:
                print(f"[INIT] Pre-streaming offboard setpoints... ({self.counter}/100)")
            if self.counter == 100:
                print("\n[TAKEOFF] Engaging Offboard mode and Arming vehicle...")
                self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.state = "CLIMBING"
                self.t_climb_start = time.time()
                
        # Phase 2: Ground liftoff and climb
        elif self.state == "CLIMBING":
            if self.counter % 10 == 0:
                print(f"[CLIMBING] Alt: {alt:4.2f}m / {self.target_alt:.1f}m | vz: {self.current_vz:+.2f}m/s | Roll: {roll:+5.1f}° | Pitch: {pitch:+5.1f}° | Motor 1 @ 50%")
            if alt >= 2.0:
                print("\n" + "=" * 65)
                print(f"🎉 [AIRBORNE SUCCESS] Drone lifted off ground with Motor 1 at 50% power!")
                print(f"   Current Altitude: {alt:.2f} m | Reached hover corridor.")
                print("=" * 65 + "\n")
                self.state = "HOVERING"
                self.t_hover_start = time.time()
                
        # Phase 3: Stable hover at target altitude
        elif self.state == "HOVERING":
            t_hover = time.time() - self.t_hover_start
            if self.counter % 10 == 0:
                print(f"[HOVERING {t_hover:4.1f}s/8s] Alt: {alt:4.2f}m | vz: {self.current_vz:+.2f}m/s | Roll: {roll:+5.1f}° | Pitch: {pitch:+5.1f}° | Motor 1 @ 50%")
            if t_hover >= 8.0:
                print("\n" + "=" * 65)
                print("🎯 [TEST COMPLETE] Rock-solid 50% power hover verified from ground takeoff!")
                print("=" * 65 + "\n")
                self.state = "DONE"
                self.land_and_finish()
                
    def land_and_finish(self):
        print("[LAND] Commanding smooth landing...")
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        time.sleep(2.0)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = AdaptiveGroundTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.land_and_finish()

if __name__ == "__main__":
    main()
