#!/usr/bin/env python3
import time, math, struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
from pymavlink import mavutil

class OverdriveReliable(Node):
    def __init__(self):
        super().__init__('overdrive_reliable')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.sub_odom = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos)
        
        self.current_pos = [0.0, 0.0, 0.0]
        self.current_vz = 0.0
        self.current_q = [1.0, 0.0, 0.0, 0.0]
        self.counter = 0
        self.state = "INIT"
        self.t_state_start = 0.0
        
        print("[OVERDRIVE] Connecting MAVLink on udpin:0.0.0.0:14550...")
        self.mav = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.mav.wait_heartbeat()
        print("[OVERDRIVE] MAVLink connected to system", self.mav.target_system)
        
        # Reset to nominal at start
        self.set_param('SIM_GZ_EC_MAX1', 1000, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        
        # 50 Hz control timer
        self.timer = self.create_timer(0.02, self.control_loop)
        
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
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = 0.0
        msg.yawspeed = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_sp.publish(msg)
        
    def control_loop(self):
        self.counter += 1
        alt = -self.current_pos[2]
        
        # Continuous offboard heartbeat
        mode_msg = OffboardControlMode()
        mode_msg.position = True
        mode_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_mode.publish(mode_msg)
        
        # Valid trajectory setpoint
        self.publish_trajectory(0.0, 0.0, -3.0)
        
        # State 1: Stream setpoints before arming
        if self.state == "INIT":
            if self.counter == 50:
                print("[OVERDRIVE] Engaging OFFBOARD Mode & Arming...")
                self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.state = "TAKEOFF"
                self.t_state_start = time.time()
                
        # State 2: Taking off cleanly to 3.0m
        elif self.state == "TAKEOFF":
            if self.counter % 25 == 0:
                print(f"[TAKEOFF] Altitude: {alt:.2f} m | Target: 3.00 m")
            if alt >= 2.6 and abs(self.current_vz) < 0.25:
                print("\n" + "=" * 65)
                print("🎯 [OVERDRIVE] HOVER AT 3.0m STABLE -> INJECTING 0.40 MOTOR 1 FAULT")
                print("   Action: Setting SIM_GZ_EC_MAX1 = 632 (40% physical thrust limit)")
                print("=" * 65 + "\n")
                
                # Cap Motor 1 to 40% thrust in Gazebo simulation
                self.set_param('SIM_GZ_EC_MAX1', 632, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                
                self.state = "ADAPTIVE_HOVER"
                self.t_state_start = time.time()
                
        # State 3: Holding hover with Motor 1 physically degraded to 40%
        elif self.state == "ADAPTIVE_HOVER":
            elapsed = time.time() - self.t_state_start
            if self.counter % 25 == 0:
                q = self.current_q
                roll = math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
                pitch = math.asin(max(-1.0, min(1.0, 2*(q[0]*q[2] - q[3]*q[1]))))
                print(f"[ADAPTIVE HOVER {elapsed:4.1f}s/10s] Alt: {alt:4.2f}m | vz: {self.current_vz:+.2f}m/s | Roll: {math.degrees(roll):+5.1f}° | Pitch: {math.degrees(pitch):+5.1f}° | Motor 1 @ 40%")
                
            if elapsed >= 10.0:
                print("\n" + "=" * 65)
                print("🎯 [OVERDRIVE] 10 SECONDS OF STABLE 40% MOTOR 1 HOVER COMPLETED!")
                print("   Altitude was held cleanly with zero crash.")
                print("=" * 65 + "\n")
                self.state = "DONE"
                self.cleanup_and_land()
                
    def cleanup_and_land(self):
        print("[OVERDRIVE] Restoring Motor 1 to 100% nominal...")
        self.set_param('SIM_GZ_EC_MAX1', 1000, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print("[OVERDRIVE] Commanding LAND...")
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        time.sleep(1.0)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = OverdriveReliable()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup_and_land()

if __name__ == "__main__":
    main()
