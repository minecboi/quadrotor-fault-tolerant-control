#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, OffboardControlMode, VehicleControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint, VehicleGlobalPosition, ActuatorMotors, VehicleThrustSetpoint, VehicleOdometry

import numpy as np
import matplotlib

from quadrotor import Quadrotor3D
from fault_bus import FaultReceiver, NOMINAL, ADAPTIVE, YAW_FREE
from mode_controller import ModeController

iter = 0
matplotlib.use('TkAgg')

class Offboard(Node):
	def __init__(self):
		super().__init__("controller")

		# QOS Profiles
		qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
		qos_profile_mavros = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

		# Publishers
		self.offboard_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
		self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
		self.actuator_motors_publisher = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", qos_profile)
		self.vehicle_thrust_publisher = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", qos_profile)
		self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)

		# Subscribers
		## PX4
		self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)
		self.status_subscriber = self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode", self.state_callback, qos_profile)
		self.attitude_subscirber = self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.attitude_callback, qos_profile)
		self.local_position_subscriber = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_callback, qos_profile)
		self.global_position_subscriber = self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.global_position_callback, qos_profile)
		self.vehicle_odometry_subscriber = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odometry_callback, qos_profile)
		self.vehicle_actuator_sub = self.create_subscription(ActuatorMotors, "/fmu/out/actuator_motors", self.actuator_callback, qos_profile)
		## MAVROS
		self.imu_subscriber = self.create_subscription(Imu, "/mavros/imu/data", self.imu_callback, qos_profile_mavros)
		self.local_position_mavros_subscriber = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.local_position_mavros_callback, qos_profile_mavros)
		self.velocity_subscriber = self.create_subscription(TwistStamped, "/mavros/local_position/velocity_body", self.velocity_callback, qos_profile_mavros)

		# Subscriber Messages Received
		## PX4
		self.vehicle_status = None
		self.state = None
		self.attitude = None
		self.local_position = None
		self.global_position = None
		self.vehicle_odometry = None
		self.actuator_thrusts = None
		## MAVROS
		self.imu_data = None
		self.local_position_mavros = None
		self.velocity = None

		# Controller
		self.dt = 0.05
		self.N = 20
		self.omega_history = []
		self.quad = Quadrotor3D()
		self.path, self.thrust_history = [], []
		self.hover = False

		# Command Loop
		self.time_period_drone = 0.05
		self.timer = self.create_timer(self.time_period_drone, self.command_loop)
		self.counter = 0
		self.ctrl = None
		self.mode, self.motor, self.rho = NOMINAL, None, None
		self.fault = FaultReceiver()

	# Subscriber Callback Functions
	## PX4
	def vehicle_status_callback(self, vehicle_status_msg):
		self.vehicle_status = vehicle_status_msg
	def state_callback(self, status_msg):
		self.status = status_msg
	def attitude_callback(self, attitude_msg):
		self.attitude = attitude_msg
	def local_position_callback(self, local_position_msg):
		self.local_position = local_position_msg
	def global_position_callback(self, global_position_msg):
		self.global_position = global_position_msg
	def vehicle_odometry_callback(self, vehicle_odometry_msg):
		self.vehicle_odometry = vehicle_odometry_msg
	## MAVROS
	def imu_callback(self, imu_msg):
		self.imu_data = imu_msg
	def local_position_mavros_callback(self, local_position_mavros_msg):
		self.local_position_mavros = local_position_mavros_msg
	def velocity_callback(self, velocity_msg):
		self.velocity = velocity_msg
	def actuator_callback(self, msg):
		self.actuator_thrusts = msg.control[:4]

	def offboard_control_heartbeat_signal_publisher(self, what_control):
		"""PX4 Offboard Mode Heartbeat Signal Publisher"""
		msg = OffboardControlMode()
		msg.position = False
		msg.velocity = False
		msg.acceleration = False
		msg.attitude = False
		msg.body_rate = False
		msg.thrust_and_torque = False
		msg.direct_actuator = False
		match what_control:
			case 'position':
				msg.position = True
			case 'velocity':
				msg.velocity = True
			case 'acceleration':
				msg.acceleration = True
			case 'attitude':
				msg.attitude = True
			case 'body_rate':
				msg.body_rate = True
			case 'thrust_and_torque':
				msg.thrust_and_torque = True
			case 'direct_actuator':
				msg.direct_actuator = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.offboard_mode_publisher.publish(msg)
	def engage_offboard_mode(self):
		"""Send Command to Engage Offboard Mode on PX4"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
		msg.param1 = 1.0
		msg.param2 = 6.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def disengage_offboard_mode(self):
		"""Send Command to Disengage Offboard Mode on PX4"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
		msg.param1 = 1.0
		msg.param2 = 3.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def arm(self):
		"""Send Command to PX4 for ARMING"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
		msg.param1 = 1.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def disarm(self):
		"""Send Command to PX4 for DISARMING"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
		msg.param1 = 0.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def kill(self):
		"""Send Command to PX4 for Flight Termination"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION
		msg.param1 = 1.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def takeoff(self):
		"""Send Command to PX4 for TAKEOFF"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
		msg.param1 = 0.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def land(self):
		"""Send Command to PX4 for LANDING"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
		msg.param1 = 0.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def set_origin(self, latitude, longitude, altitude):
		"""Send Command to PX4 for setting the ORIGIN"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN
		msg.param1 = 0.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = latitude
		msg.param6 = longitude
		msg.param7 = altitude
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def individual_motor(self, motor_1, motor_2, motor_3, motor_4):
		"""Send Command to PX4 for Setting the Individual Thrusts to 4 Rotors"""
		msg = ActuatorMotors()
		msg.control[0] = motor_1
		msg.control[1] = motor_2
		msg.control[2] = motor_3
		msg.control[3] = motor_4
		msg.control[4] = float('nan')
		msg.control[5] = float('nan')
		msg.control[6] = float('nan')
		msg.control[7] = float('nan')
		msg.control[8] = float('nan')
		msg.control[9] = float('nan')
		msg.control[10] = float('nan')
		msg.control[11] = float('nan')
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.actuator_motors_publisher.publish(msg)
	def set_thrust(self, x, y, z):
		"""Send Command to PX4 for Setting Overall Thrust in X,Y,Z Coordinates"""
		msg = VehicleThrustSetpoint()
		msg.xyz[0] = x
		msg.xyz[1] = y
		msg.xyz[2] = z
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_thrust_publisher.publish(msg)
	def publish_trajectory(self, x, y, z):
		"""Send Command to PX4 for Publishing Trajectory (X,Y,Z)"""
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
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.trajectory_publisher.publish(msg)

	def command_loop(self):
		if self.vehicle_odometry is None:
			print("No odom!!")
			return

		# 1. build the single compiled MPC once, from the first valid odometry
		if self.ctrl is None:
			if False:   # MAVROS not used
				return
			o = self.vehicle_odometry
			init = [o.position[0], o.position[1], o.position[2], o.q[0], o.q[1], o.q[2], o.q[3],
			        o.velocity[0], o.velocity[1], o.velocity[2],
			        o.angular_velocity[0], o.angular_velocity[1], o.angular_velocity[2]]
			self.hover_goal = [o.position[0], o.position[1], o.position[2] - 5.0]   # 5 m up (NED)
			self.ctrl = ModeController(self.quad, initial_state=init, n_nodes=self.N)
			print("controller built: one compiled MPC, fault mode set by parameters")
			return

		o = self.vehicle_odometry
		state = np.array(list(o.position) + list(o.q) + list(o.velocity) + list(o.angular_velocity), dtype=float)
		self.counter += 1

		# 2. detector link. The first fault latches. YAW-FREE is latched; ADAPTIVE after YAW-FREE is ignored.
		st = self.fault.poll()
		want, motor, rho = st.get("mode", NOMINAL), st.get("motor"), st.get("rho")
		if want in (ADAPTIVE, YAW_FREE) and motor is not None:
			if not (self.mode == YAW_FREE and want == ADAPTIVE):
				if self.mode == NOMINAL:
					print(f"[takeover @ {self.counter}] detector: {want}, motor {motor}, rho {rho} -> overriding PX4")
				self.mode, self.motor, self.rho = want, motor, rho

		# 3. NOMINAL: PX4 flies (position hold). Our MPC runs silently so its warm start is ready at takeover.
		if self.mode == NOMINAL:
			self.offboard_control_heartbeat_signal_publisher("position")
			self.publish_trajectory(*self.hover_goal)
			self.ctrl.solve(state, self.hover_goal, NOMINAL)
			if self.counter == 100:
				self.engage_offboard_mode()
				self.arm()
			return

		# 4. OVERRIDE: our controller commands the four motors directly
		self.offboard_control_heartbeat_signal_publisher("direct_actuator")
		u, status = self.ctrl.solve(state, self.hover_goal, self.mode, self.motor, self.rho)
		thrust = u.copy()
		if self.mode == YAW_FREE:
			thrust[self.motor - 1] = 0.0          # off = 0, not the 0.15 idle used for healthy motors
		if status != 0:
			print(f"[mpc] solver status {status}: holding last command")
		self.individual_motor(*thrust)

def main(args=None):
	rclpy.init(args=args)                            # Initial ROS2 Client
	offboard = Offboard()                            # Creating Offboard Class Object for Controller
	rclpy.spin(offboard)                             # Spin the ROS Node
	offboard.destroy_node()                          # Destroy the ROS Node
	rclpy.shutdown()                                 # Shutdown ROS2 Client

if __name__ == "__main__":
    main()                                  # Calling main function