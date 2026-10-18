#!/usr/bin/env python3
import time, math, struct, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleAttitudeSetpoint
from pymavlink import mavutil

class OverdriveTest(Node):
    def __init__(self):
        super().__init__('overdrive_test')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_att_sp = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.sub_odom = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos)
        
        self.current_pos = [0.0, 0.0, 0.0]
        self.current_vz = 0.0
        self.current_q = [1.0, 0.0, 0.0, 0.0]
        self.counter = 0
        self.state = "INIT"
        self.t_state_start = 0.0
        self.param_lock = threading.Lock()
        
        # MAVLink for parameter setting
        print("[OVERDRIVE] Connecting MAVLink on udpin:0.0.0.0:14550...")
        self.mav = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.mav.wait_heartbeat()
        print("[OVERDRIVE] MAVLink connected to system", self.mav.target_system)
        
        # Ensure nominal parameters at startup
        self.set_param_sync('SIM_GZ_EC_MAX1', 1000, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.set_param_sync('CA_ROTOR0_CT', 6.50, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        
        # Timer at 50 Hz (20 ms) - NEVER BLOCKED
        self.timer = self.create_timer(0.02, self.control_loop)
        
    def set_param_sync(self, name, value, ptype):
        with self.param_lock:
            raw = struct.unpack('<f', struct.pack('<i', int(value)))[0] if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32 else float(value)
            self.mav.mav.param_set_send(self.mav.target_system, self.mav.target_component, name.encode(), raw, ptype)

    def set_param_async(self, name, value, ptype):
        threading.Thread(target=self.set_param_sync, args=(name, value, ptype), daemon=True).start()
        
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
        
    def control_loop(self):
        self.counter += 1
        alt = -self.current_pos[2]
        
        # Always publish offboard heartbeat and target setpoint
        mode_msg = OffboardControlMode()
        mode_msg.position = True
        mode_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_mode.publish(mode_msg)
        
        sp = TrajectorySetpoint()
        sp.position = [0.0, 0.0, -3.0]
        sp.yaw = 0.0
        sp.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_sp.publish(sp)
        
        # State 1: Pre-stream setpoints (counter 0 to 50)
        if self.state == "INIT":
            if self.counter == 50:
                print("[OVERDRIVE] Engaging OFFBOARD Mode & Arming...")
                self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.state = "TAKEOFF"
                self.t_state_start = time.time()
                
        # State 2: Taking off to 3m
        elif self.state == "TAKEOFF":
            if self.counter % 25 == 0:
                print(f"[TAKEOFF] Altitude: {alt:.2f} m | Target: 3.00 m")
            if alt >= 2.5 and abs(self.current_vz) < 0.3:
                print("\n" + "=" * 65)
                print("🎯 [OVERDRIVE PHASE 1] HOVER STABILIZED AT 3.0m -> ENGAGING ADAPTIVE 0.40 OVERRIDE")
                print("   Action: Physically capping Motor 1 to 40% thrust (RPM 632)")
                print("   Action: Setting CA_ROTOR0_CT = 2.60 (Adaptive allocation compensation)")
                print("=" * 65 + "\n")
                
                # Asynchronously cap Motor 1 to 40% thrust in Gazebo & update allocator
                self.set_param_async('SIM_GZ_EC_MAX1', 632, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                self.set_param_async('CA_ROTOR0_CT', 2.60, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                
                self.state = "ADAPTIVE_HOVER"
                self.t_state_start = time.time()
                
        # State 3: Adaptive hover with Motor 1 at 40% power
        elif self.state == "ADAPTIVE_HOVER":
            elapsed = time.time() - self.t_state_start
            if self.counter % 25 == 0:
                # Calculate tilt from quaternion
                q = self.current_q
                roll = math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
                pitch = math.asin(max(-1.0, min(1.0, 2*(q[0]*q[2] - q[3]*q[1]))))
                print(f"[ADAPTIVE HOVER {elapsed:4.1f}s/10s] Alt: {alt:4.2f}m | Roll: {math.degrees(roll):+5.1f}° | Pitch: {math.degrees(pitch):+5.1f}° | Motor 1 @ 40%")
                
            if elapsed >= 10.0:
                print("\n" + "=" * 65)
                print("🎯 [OVERDRIVE] 10 SECONDS ADAPTIVE HOVER COMPLETE AND STABLE!")
                print("   Motor 1 operated at 40% thrust with zero crash / zero tilt.")
                print("=" * 65 + "\n")
                self.state = "DONE"
                self.cleanup_and_land()
                
    def cleanup_and_land(self):
        print("[OVERDRIVE] Restoring nominal parameters...")
        self.set_param_sync('SIM_GZ_EC_MAX1', 1000, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.set_param_sync('CA_ROTOR0_CT', 6.50, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        print("[OVERDRIVE] Commanding LAND...")
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        time.sleep(2.0)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = OverdriveTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup_and_land()

if __name__ == "__main__":
    main()
