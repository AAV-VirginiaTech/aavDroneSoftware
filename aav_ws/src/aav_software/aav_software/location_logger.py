#!/usr/bin/env python3
import csv
import os
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node

from aav_msgs.msg import TargetPosition


class LocationLogger(Node):
    def __init__(self):
        super().__init__("location_logger")

        self.declare_parameter("topic_name", "AAV/estimated_target_position")
        self.declare_parameter("log_file", "location_log.csv")

        topic_name = self.get_parameter("topic_name").value
        log_file = self.get_parameter("log_file").value

        self.log_file_path = os.path.abspath(log_file)
        self._ensure_log_file()

        self.subscription = self.create_subscription(
            TargetPosition,
            topic_name,
            self.location_callback,
            10,
        )

        self.get_logger().info(
            f"Location Logger started. Subscribing to '{topic_name}', logging to '{self.log_file_path}'"
        )

    def _ensure_log_file(self):
        if not os.path.exists(self.log_file_path):
            try:
                with open(self.log_file_path, mode="w", newline="", encoding="utf-8") as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(["timestamp", "name", "latitude", "longitude"])
            except OSError as exc:
                self.get_logger().error(f"Failed to create log file {self.log_file_path}: {exc}")

    def location_callback(self, msg: TargetPosition):
        now = self.get_clock().now()
        seconds, nanoseconds = now.seconds_nanoseconds()
        timestamp = datetime.fromtimestamp(seconds + nanoseconds * 1e-9, tz=timezone.utc).isoformat()
        row = [timestamp, msg.object_label, msg.latitude, msg.longitude]

        try:
            with open(self.log_file_path, mode="a", newline="", encoding="utf-8") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(row)
        except OSError as exc:
            self.get_logger().error(f"Could not write location record: {exc}")
            return

        self.get_logger().info(
            f"Logged location: timestamp={timestamp}, object_label={msg.object_label}, latitude={msg.latitude}, longitude={msg.longitude}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LocationLogger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
