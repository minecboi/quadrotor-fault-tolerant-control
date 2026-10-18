#!/usr/bin/env python3
"""
px4_sensor_logger.py - ROS 2 (Humble) node that listens to PX4 sensor/state
topics (published by the uXRCE-DDS agent) and stores every message in CSV
files for later analysis.

One CSV file is written per topic, e.g.
    ~/px4_logs/20261015_143000/fmu_out_sensor_combined.csv

Each row has:  t_wall_s (unix time), t_rel_s (seconds since the logger started),
followed by every field of the message (nested fields and arrays are flattened,
e.g. gyro_rad[0], q[3], ...). PX4's own `timestamp` field (microseconds) is kept.

Requirements
  - ROS 2 Humble sourced
  - px4_msgs built in a workspace and sourced (version must match your PX4)
  - MicroXRCEAgent running and PX4 simulation running

Run
  source /opt/ros/humble/setup.bash
  source ~/your_ws/install/setup.bash
  python3 px4_sensor_logger.py
  # custom output folder:
  python3 px4_sensor_logger.py --ros-args -p output_dir:=/home/you/my_run

Stop with Ctrl+C (files are flushed every second and closed on exit).

Analyse
  import pandas as pd
  df = pd.read_csv("fmu_out_sensor_combined.csv")
"""

import csv
import importlib
import os
import time
from datetime import datetime
from functools import partial

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosidl_runtime_py.convert import message_to_ordereddict

# (topic, [candidate px4_msgs class names - first one that exists is used])
# Edit this list to add/remove topics. Check what your PX4 really publishes with:
#   ros2 topic list | grep fmu/out
# Newer PX4 versions may add a version suffix to topic names (e.g. *_v1);
# if so, change the names here to match `ros2 topic list`.
TOPICS = [
    ("/fmu/out/sensor_combined", ["SensorCombined"]),            # gyro + accel
    ("/fmu/out/vehicle_angular_velocity", ["VehicleAngularVelocity"]),
    ("/fmu/out/vehicle_attitude", ["VehicleAttitude"]),
    ("/fmu/out/vehicle_local_position", ["VehicleLocalPosition"]),
    ("/fmu/out/vehicle_global_position", ["VehicleGlobalPosition"]),
    ("/fmu/out/vehicle_gps_position", ["SensorGps", "VehicleGpsPosition"]),
    ("/fmu/out/vehicle_air_data", ["VehicleAirData"]),           # barometer
    ("/fmu/out/vehicle_magnetometer", ["VehicleMagnetometer"]),
    ("/fmu/out/vehicle_odometry", ["VehicleOdometry"]),
    ("/fmu/out/battery_status", ["BatteryStatus"]),
    ("/fmu/out/vehicle_status", ["VehicleStatus"]),
    ("/fmu/out/actuator_motors", ["ActuatorMotors"]),            # only if published
]


def flatten(prefix, value, out):
    """Flatten nested dicts / arrays into {'a.b[0]': value} form."""
    if isinstance(value, dict):
        for key, val in value.items():
            flatten(f"{prefix}.{key}" if prefix else key, val, out)
    elif isinstance(value, (list, tuple, np.ndarray)):
        for i, val in enumerate(value):
            flatten(f"{prefix}[{i}]", val, out)
    else:
        out[prefix] = value


class TopicWriter:
    """Writes one topic's messages to its own CSV file."""

    def __init__(self, path):
        self.path = path
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.columns = None
        self.count = 0

    def write(self, t_wall, t_rel, flat):
        if self.columns is None:  # header comes from the first message
            self.columns = list(flat.keys())
            self.writer.writerow(["t_wall_s", "t_rel_s"] + self.columns)
        row = [f"{t_wall:.6f}", f"{t_rel:.6f}"]
        row += [flat.get(col, "") for col in self.columns]
        self.writer.writerow(row)
        self.count += 1

    def flush(self):
        self.file.flush()

    def close(self):
        self.file.close()


class SensorLogger(Node):
    def __init__(self):
        super().__init__("px4_sensor_logger")

        self.declare_parameter("output_dir", "")
        out_dir = self.get_parameter("output_dir").value
        if not out_dir:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            out_dir = os.path.expanduser(f"~/px4_logs/{stamp}")
        os.makedirs(out_dir, exist_ok=True)
        self.out_dir = out_dir

        self.t0 = time.time()
        self.writers = {}
        self.subs = []
        self.error_reported = set()

        # PX4's uXRCE-DDS topics are best-effort / volatile.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        try:
            px4_msgs = importlib.import_module("px4_msgs.msg")
        except ImportError as exc:
            raise SystemExit(
                "Could not import px4_msgs. Build it in a workspace and "
                f"source that workspace first. ({exc})"
            )

        for topic, class_names in TOPICS:
            msg_cls = None
            for name in class_names:
                msg_cls = getattr(px4_msgs, name, None)
                if msg_cls is not None:
                    break
            if msg_cls is None:
                self.get_logger().warn(
                    f"px4_msgs has none of {class_names}; skipping {topic}"
                )
                continue

            fname = topic.strip("/").replace("/", "_") + ".csv"
            self.writers[topic] = TopicWriter(os.path.join(out_dir, fname))
            self.subs.append(
                self.create_subscription(
                    msg_cls, topic, partial(self.on_msg, topic), qos
                )
            )
            self.get_logger().info(f"Logging {topic} ({msg_cls.__name__})")

        self.get_logger().info(f"Writing CSV files to {out_dir}")
        self.flush_timer = self.create_timer(1.0, self.flush_all)
        self.check_timer = self.create_timer(5.0, self.check_silent_topics)

    def on_msg(self, topic, msg):
        now = time.time()
        try:
            flat = {}
            flatten("", message_to_ordereddict(msg), flat)
            self.writers[topic].write(now, now - self.t0, flat)
        except Exception as exc:  # keep logging the other topics
            if topic not in self.error_reported:
                self.error_reported.add(topic)
                self.get_logger().error(f"Failed to log {topic}: {exc}")

    def flush_all(self):
        for writer in self.writers.values():
            writer.flush()

    def check_silent_topics(self):
        """One-time warning for topics that never delivered a message."""
        self.check_timer.cancel()
        for topic, writer in self.writers.items():
            if writer.count == 0:
                self.get_logger().warn(
                    f"No messages yet on {topic}. Is the agent/simulator "
                    "running, and is the topic name right? "
                    "(check `ros2 topic list`)"
                )

    def close(self):
        for topic, writer in self.writers.items():
            writer.close()
            self.get_logger().info(f"{topic}: {writer.count} messages")
        self.get_logger().info(f"Saved in {self.out_dir}")


def main():
    rclpy.init()
    node = SensorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
