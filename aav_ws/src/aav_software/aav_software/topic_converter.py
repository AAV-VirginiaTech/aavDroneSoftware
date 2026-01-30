#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ardupilot_msgs.msg import Status
from ardupilot_msgs.msg import GlobalPosition 


class TopicConverter(Node):
    def __init__(self):
        super().__init__("topic_converter")
        self.get_logger().info("Topic Converter has been launched")
        # Subscribers
        self.status_subscriber = self.create_subscription(Status, 'status', self.status_callback, 10)
        self.global_position_subscriber = self.create_subscription(GlobalPosition, 'global_position', self.global_position_callback, 10)
        # Publishers
        self.mode_publisher = self.create_publisher(int, 'current_mode', 10)
        self.gps_publisher = self.create_publisher(float, 'current_gps_position', 10)

    def status_callback(self, msg: Status):
        current_mode = msg.mode
        self.mode_publisher.publish(current_mode)
        self.get_logger().info(f"Published current mode: {current_mode}")

    def global_position_callback(self, msg: GlobalPosition):
        current_gps = msg.latitude
        self.gps_publisher.publish(current_gps)
        self.get_logger().info(f"Published current GPS position: {current_gps}")



def main(args=None):
    rclpy.init(args=args)
    node = TopicConverter()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
