#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ardupilot_msgs.msg import Status
from aav_msgs.msg import Mode
from ardupilot_msgs.msg import GlobalPosition 
from aav_msgs.msg import DronePosition
from aav_msgs.msg import LatLong
from enum import Enum, IntEnum

#FOR NEXT MEETING!!!!
# TODO: Can "Hardcode" the alititude to send to ardupilot. Need to do this based on min altitude
# TODO: Finish rest of the functionality in this file. Look in test_node on how to publish new gps location to arudpilot. Need altitude in order for drone to not crash :)
# TODO: remove (or update) this class once topic_converter assigns mode values

class ArduPilotMode(IntEnum):
    STABILIZE =     0  # manual airframe angle with manual throttle
    ACRO =          1  # manual body-frame angular rate with manual throttle
    ALT_HOLD =      2  # manual airframe angle with automatic throttle
    AUTO =          3  # fully automatic waypoint control using mission commands
    GUIDED =        4  # fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5  # automatic horizontal acceleration with automatic throttle
    RTL =           6  # automatic return to launching point
    CIRCLE =        7  # automatic circular flight with automatic throttle
    LAND =          9   # automatic landing with horizontal position control
    DRIFT =        11   # semi-autonomous position, yaw and throttle control
    SPORT =        13  # manual earth-frame angular rate control with manual throttle
    FLIP =         14  # automatically flip the vehicle on the roll axis
    AUTOTUNE =     15  # automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16  # automatic position hold with manual override, with automatic throttle
    BRAKE =        17  # full-brake using inertial/GPS system, no pilot input
    THROW =        18  # throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB =   19  # automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20  # guided mode but only accepts attitude and altitude
    SMART_RTL =    21  # SMART_RTL returns to home by retracing its steps
    FLOWHOLD  =    22  # FLOWHOLD holds position with optical flow without rangefinder
    FOLLOW    =    23  # follow attempts to follow another vehicle or ground station
    ZIGZAG    =    24  # ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    SYSTEMID  =    25  # System ID mode produces automated system identification signals in the controllers
    AUTOROTATE =   26  # Autonomous autorotation
    AUTO_RTL =     27  # Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
    TURTLE =       28  # Flip over after crash


class TopicConverter(Node):
    def __init__(self):
        super().__init__("topic_converter")
        self.get_logger().info("Topic Converter has been launched")

        self.current_altitude = None
        self.minimun_altitude = None #update everytime 

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
        ap_mode = ArduPilotMode(msg.mode)

        self.current_mode = ap_mode 
        self.get_logger().info(f"Received ArduPilot mode: {self.current_mode.name} ({self.current_mode.value})")
        mode_msg = Mode()
        mode_msg.mode = self.current_mode.value
        
        self.mode_publisher.publish(mode_msg)

    def global_position_callback(self, msg: GlobalPosition):
        self.current_altitude = msg.altitude
        gps_msg = DronePosition()
        gps_msg.latitude = msg.latitude
        gps_msg.longitude = msg.longitude
        gps_msg.altitude = 10.0
        gps_msg.yaw = msg.yaw   

        self.gps_publisher.publish(gps_msg)
        self.get_logger().info(f"Published GPS lat={gps_msg.latitude}, lon={gps_msg.longitude}, alt={gps_msg.altitude}, yaw={gps_msg.yaw}")

    def new_position_callback(self, msg: LatLong):
        if self.current_mode != ArduPilotMode.GUIDED:
            self.get_logger().warning("Current mode is not GUIDED. Cannot publish new GPS position to ArduPilot.")
            return
        
        if self.current_altitude < self.minimun_altitude:
            self.minimun_altitude = self.current_altitude
  
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
