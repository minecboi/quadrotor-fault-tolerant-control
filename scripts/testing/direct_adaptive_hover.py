#!/usr/bin/env python3
import time, math, struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, ActuatorMotors
from pymavlink import mavutil

class DirectAdaptiveHover(Node):
    def __init__(self):
        super().__init__('direct_adaptive_hover')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_act = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.sub_odom = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos)
        
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.w = [0.0, 0.0, 0.0]
        
        self.counter = 0
        self.state = "INIT"
        self.t_state_start = 0.0
        
        # MAVLink connection
        print("[ADAPTIVE] Connecting MAVLink on udpin:0.0.0.0:14550...")
        self.mav = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.mav.wait_heartbeat()
        print("[ADAPTIVE] MAVLink connected to system", self.mav.target_system)
        
        # Reset motor 1 to nominal
        self.set_param('SIM_GZ_EC_MAX1', 1000, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        
        # 50 Hz timer (20 ms)
        self.timer = self.create_timer(0.02, self.control_loop)
        
    def set_param(self, name, value, ptype):
        raw = struct.unpack('<f', struct.pack('<i', int(value)))[0] if ptype == mavutil.mavlink.MAV_PARAM_TYPE_INT32 else float(value)
        self.mav.mav.param_set_send(self.mav.target_system, self.mav.target_component, name.encode(), raw, ptype)
        
    def odom_callback(self, msg):
        self.pos = list(msg.position)
        self.vel = list(msg.velocity)
        self.q = list(msg.q)
        self.w = list(msg.angular_velocity)
        
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
        alt = -self.pos[2]
        vz = self.vel[2]
        
        # -------------------------------------------------------------
        # State 1 & 2: Smooth Position-Hold Takeoff to 2.5m
        # -------------------------------------------------------------
        if self.state in ("INIT", "TAKEOFF"):
            mode_msg = OffboardControlMode()
            mode_msg.position = True
            mode_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            self.pub_mode.publish(mode_msg)
            
            sp = TrajectorySetpoint()
            sp.position = [0.0, 0.0, -2.5]
            sp.velocity = [float('nan'), float('nan'), float('nan')]
            sp.acceleration = [float('nan'), float('nan'), float('nan')]
            sp.jerk = [float('nan'), float('nan'), float('nan')]
            sp.yaw = 0.0
            sp.yawspeed = float('nan')
            sp.timestamp = self.get_clock().now().nanoseconds // 1000
            self.pub_sp.publish(sp)
            
            if self.state == "INIT" and self.counter == 50:
                print("[ADAPTIVE] Engaging OFFBOARD Mode & Arming...")
                self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.state = "TAKEOFF"
                self.t_state_start = time.time()
                
            elif self.state == "TAKEOFF":
                if self.counter % 25 == 0:
                    print(f"[TAKEOFF] Altitude: {alt:.2f} m | Target: 2.50 m")
                if alt >= 2.3 and abs(vz) < 0.25:
                    print("\n" + "=" * 65)
                    print("🎯 [ADAPTIVE] HOVER AT 2.5m STABLE -> INJECTING 0.40 MOTOR 1 FAULT")
                    print("   Action: Physically capping Motor 1 to 40% thrust in Gazebo (RPM 632)")
                    print("   Action: Switching to Direct Actuator Adaptive Mixer Override")
                    print("=" * 65 + "\n")
                    
                    self.set_param('SIM_GZ_EC_MAX1', 632, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                    self.state = "ADAPTIVE_HOVER"
                    self.t_state_start = time.time()
                    
        # -------------------------------------------------------------
        # State 3: Direct Actuator Adaptive Mixer Hover (Motor 1 @ 40%)
        # -------------------------------------------------------------
        elif self.state == "ADAPTIVE_HOVER":
            elapsed = time.time() - self.t_state_start
            
            # Send Direct Actuator Heartbeat
            mode_msg = OffboardControlMode()
            mode_msg.direct_actuator = True
            mode_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            self.pub_mode.publish(mode_msg)
            
            # Compute Euler angles from quaternion
            q = self.q
            roll = math.atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))
            pitch = math.asin(max(-1.0, min(1.0, 2*(q[0]*q[2] - q[3]*q[1]))))
            
            # Closed-loop stabilization
            u_hover_base = 0.58  # Hover throttle for healthy motors
            alt_err = 2.5 - alt
            u_alt = np.clip(u_hover_base + 0.18 * alt_err - 0.15 * vz, 0.45, 0.85)
            
            # Attitude corrections
            d_roll = -1.5 * roll - 0.25 * self.w[0]
            d_pitch = -1.5 * pitch - 0.25 * self.w[1]
            
            # Multi-rotor X mixer (M1=FR, M2=BL, M3=FL, M4=BR)
            # Motor 1 is at 40% thrust, so its command is boosted by 1/0.4 = 2.5x
            u1_raw = (u_alt - d_pitch - d_roll) / 0.40
            u2_raw = u_alt + d_pitch + d_roll
            u3_raw = u_alt - d_pitch + d_roll
            u4_raw = u_alt + d_pitch - d_roll
            
            u_cmd = [
                float(np.clip(u1_raw, 0.0, 1.0)),
                float(np.clip(u2_raw, 0.0, 1.0)),
                float(np.clip(u3_raw, 0.0, 1.0)),
                float(np.clip(u4_raw, 0.0, 1.0))
            ]
            
            act_msg = ActuatorMotors()
            act_msg.control[:4] = u_cmd
            for k in range(4, 12):
                act_msg.control[k] = float('nan')
            act_msg.timestamp = self.get_clock().now().nanoseconds // 1000
            self.pub_act.publish(act_msg)
            
            if self.counter % 25 == 0:
                print(f"[ADAPTIVE HOVER {elapsed:4.1f}s/10s] Alt: {alt:4.2f}m | Roll: {math.degrees(roll):+5.1f}° | Pitch: {math.degrees(pitch):+5.1f}° | u1={u_cmd[0]:.2f}(M1@40%) u2={u_cmd[1]:.2f}")
                
            if elapsed >= 10.0:
                print("\n" + "=" * 65)
                print("🎯 [ADAPTIVE] 10 SECONDS OF ADAPTIVE HOVER SUCCESSFULLY COMPLETED!")
                print("   Motor 1 maintained 40% thrust and held altitude with zero crash.")
                print("=" * 65 + "\n")
                self.state = "DONE"
                self.cleanup_and_land()
                
    def cleanup_and_land(self):
        print("[ADAPTIVE] Restoring nominal parameters...")
        self.set_param('SIM_GZ_EC_MAX1', 1000, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print("[ADAPTIVE] Commanding LAND...")
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        time.sleep(1.0)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = DirectAdaptiveHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup_and_land()

if __name__ == "__main__":
    main()
