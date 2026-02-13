#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aav_msgs.msg import Mode
from aav_msgs.msg import TargetPosition
from aav_msgs.msg import LatLong
from topic_converter import GuidedMode


class ObjectAlignmentController(Node):
    def __init__(self):
        super().__init__("object_alignment_controller")

        self.modeSub = self.create_subscription(Mode, "/AAV/current_mode", self.callback_mode, 10)
        self.posSub = self.create_subscription(TargetPosition, "/AAV/estimated_target_position", self.callback_pos, 10)
        self.posPub = self.create_publisher(LatLong, "/AAV/send_new_position", 10)

        self.currentMode = GuidedMode.AUTO
        self.lastObjectLabel = "none"

        self.get_logger().info("Object Alignment Controller has been launched")

    def callback_mode(self, current_mode: Mode):
        self.get_logger().info(f"Recieved new mode: {current_mode}")
        self.currentMode = GuidedMode(current_mode.mode)

    def callback_pos(self, target_position: TargetPosition):
        self.get_logger().info(f"Recieved new target position: {target_position}")
        if (self.currentMode != GuidedMode.GUIDED):
            self.get_logger().info("Not guided. Doing nothing.")
            return

        self.get_logger().info(f"Sending new positition")
        self.lastObjectLabel = target_position.object_label

        new_position = LatLong()
        new_position.latitude = target_position.latitude
        new_position.longitude = target_position.longitude
        self.posPub.publish(new_position)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAlignmentController()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
