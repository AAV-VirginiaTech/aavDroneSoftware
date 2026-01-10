#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import message interfaces
from ardupilot_msgs.msg import GlobalPosition
from aav_msgs.msg import Mode



# This node just serves as an example to show how to write ros2 python code.
# This is not actually used in the actual production code.


class TestNode(Node):


    def __init__(self):
        super().__init__("test_node")
        self.ardupilotPublisher = self.create_publisher(GlobalPosition, '/ap/cmd_gps_pose', 10)
        self.aavPublisher = self.create_publisher(Mode, '/aav/current_mode', 10)
        self.get_logger().info("Test Node has been launched")
        self.create_timer(1.0, self.timer_callback)
        
        


    def timer_callback(self):
        
        # Use ardupilot interface
        msg1 = GlobalPosition()
        msg1.header.frame_id = 'map'
        msg1.coordinate_frame = GlobalPosition.FRAME_GLOBAL_INT  # = 5
        msg1.latitude = -35.365822
        msg1.longitude = 149.163124
        self.ardupilotPublisher.publish(msg1)
        
        # Use AAV interface
        msg2 = Mode()
        msg2.mode = 8
        self.aavPublisher.publish(msg2)
        
        
        


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
