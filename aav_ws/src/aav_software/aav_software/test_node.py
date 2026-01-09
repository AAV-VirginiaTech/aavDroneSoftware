#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ardupilot_msgs.msg import GlobalPosition




class TestNode(Node):


    def __init__(self):
        super().__init__("test_node")
        self.counter = 0
        self.publisher = self.create_publisher(GlobalPosition, '/ap/cmd_gps_pose', 10)
        self.get_logger().info("Test Node has been launched")
        self.create_timer(1.0, self.timer_callback)
        
        


    def timer_callback(self):
        self.get_logger().info("Test Node Count " + str(self.counter))
        self.counter += 1
        
        
        msg = GlobalPosition()

        # Match the CLI command exactly
        msg.header.frame_id = 'map'
        msg.coordinate_frame = GlobalPosition.FRAME_GLOBAL_INT  # = 5
        msg.latitude = -35.365822
        msg.longitude = 149.163124

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
