#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ardupilot_msgs.msg import Status
from aav_msgs.msg import Mode
from ardupilot_msgs.msg import GlobalPosition 
from aav_msgs.msg import DronePosition
from aav_msgs.msg import LatLong
from enum import Enum

#FOR NEXT MEETING!!!!
# TODO: Can "Hardcode" the alititude to send to ardupilot
# TODO: Create enum within this file. Look at Charlie's object_alignment_controller code. Or just ask him about it.
# TODO: Finish rest of the functionality in this file. Look in test_node on how to publish new gps location to arudpilot. Need altitude in order for drone to not crash :)
# todo: remove (or update) this class once topic_converter assigns mode values

#TODO: 

class ModeEnum(Enum):
    """Dummy enum that represents Ardupilot mode. Should be replaced by similar enum in topic_converter.py"""
    NOT_GUIDED = 0
    GUIDED = 1

    def from_int(num: int):
        match num:
            case 0:
                return NOT_GUIDED
            case 1:
                return GUIDED
            case _:
                raise ValueError("ModeEnum requires a value between 0 and 1")


class TopicConverter(Node):
    def __init__(self):
        super().__init__("topic_converter")
        self.get_logger().info("Topic Converter has been launched")

        self.current_altitude = None

        # Subscriber(Mode): ArduPilot -> TC
        self.status_subscriber = self.create_subscription(Status, '/ap/status', self.status_callback, 10)
        # Publisher(Mode): TC -> AAV Software
        self.mode_publisher = self.create_publisher(Mode, '/AAV/current_mode', 10)

        # Subscriber(GlobalPosition): ArduPilot -> TC
        self.global_position_subscriber = self.create_subscription(GlobalPosition, '/ap/global_position', self.global_position_callback, 10)
        # Publisher(GlobalPosition): TC -> AAV Software
        self.gps_publisher = self.create_publisher(DronePosition, '/AAV/current_gps_position', 10)

        #Subscriber(NewPosition): AAV Software -> TC
        self.new_position_subscriber = self.create_subscription(LatLong, '/AAV/send_new_position', self.new_position_callback, 10)
        #Publisher(NewPosition): TC -> ArduPilot
        self.new_gps_publisher = self.create_publisher(GlobalPosition, '/ap/set_gps_position', 10)

    
    def status_callback(self, msg: Status):
        self.current_mode = msg.mode
        mode_msg = Mode()
        mode_msg.mode = msg.mode
        
        self.mode_publisher.publish(mode_msg)
        self.get_logger().info(f"Published current mode: {mode_msg.mode}")

    def global_position_callback(self, msg: GlobalPosition):
        self.current_altitude = msg.altitude
        gps_msg = DronePosition()
        gps_msg.latitude = msg.latitude
        gps_msg.longitude = msg.longitude
        gps_msg.altitude = msg.altitude
        gps_msg.yaw = msg.yaw   

        self.gps_publisher.publish(gps_msg)
        self.get_logger().info(f"Published GPS lat={gps_msg.latitude}, lon={gps_msg.longitude}, alt={gps_msg.altitude}, yaw={gps_msg.yaw}")

    def new_position_callback(self, msg: LatLong):
        if self.current_altitude is None:
            self.get_logger().warning("Current altitude is unknown. Cannot publish new GPS position to ArduPilot.")
            return
        
        new_gps_msg = GlobalPosition()
        new_gps_msg.latitude = msg.latitude
        new_gps_msg.longitude = msg.longitude
        new_gps_msg.altitude = self.current_altitude  # Use current altitude to avoid crashing
        new_gps_msg.yaw = 0.0  # Default yaw, can be modified as needed

        self.new_gps_publisher.publish(new_gps_msg)
        self.get_logger().info(f"Published new GPS position to ArduPilot: lat={new_gps_msg.latitude}, lon={new_gps_msg.longitude}, alt={new_gps_msg.altitude}, yaw={new_gps_msg.yaw}")




def main(args=None):
    rclpy.init(args=args)
    node = TopicConverter()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
