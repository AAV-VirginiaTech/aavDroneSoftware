#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ardupilot_msgs.msg import Status
from aav_msgs.msg import Mode
from ardupilot_msgs.msg import GlobalPosition 
from aav_msgs.msg import DronePosition
from aav_msgs.msg import LatLong
from enum import Enum, IntEnum
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import Takeoff

# TODO: Can "Hardcode" the alititude to send to ardupilot. Need to do add desired altitude to min altitude. Hardcode to 8 meters above the ground.
# TODO: Finish rest of the functionality in this file. Look in test_node on how to publish new gps location to arudpilot.

# TODO: The altitude you publish to /AAV/current_gps_position topic should be the altitude relative to the ground. Will need to do math for this.

# TODO: Implement the new takeoff mode.
# 1. Switch mode to guided
# 2. Arm motors
# 3. Publish new gps position with desired takeoff altitude (e.g., 30 meters)

# NOTE: Don't worry about the lidar stuff for now. Just do the above todos and we can figure out lidar later


#Testing commands:
"""
# Send new position
ros2 topic pub --once /AAV/send_new_position aav_msgs/msg/LatLong "{
  latitude: 37.2295,
  longitude: -80.4138
}"



# View Topic
ros2 topic echo /AAV/current_gps_position

ros2 topic echo /AAV/current_mode




# Send new mode

ros2 topic pub --once /AAV/current_mode aav_msgs/msg/Mode "{mode: 4}"


### Aditional testing ###

# Send new gps cordinate
ros2 topic pub --once /ap/cmd_gps_pose ardupilot_msgs/msg/GlobalPosition "{
  header: {frame_id: 'map'},
  coordinate_frame: 5,
  latitude: -35.365822,
  longitude: 149.163124,
  altitude: 600.0
}"

# Test landing drone
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 9}"


# Test taking off drone

Switch to Guided Mode (needed for takeoff):
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}" 

Arm Motors (need to call execute takeoff command right after this, otherwise drone will disarm for safety):
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"

Execute Takeoff (e.g., 30 meters):
ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 30.0}"

"""

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
    TAKEOFF =      29  # Custom AAV mode. Will report as GUIDED to ArduPilot, but will have custom takeoff behavior in the topic converter.


class TopicConverter(Node):
    def __init__(self):
        super().__init__("topic_converter")
        self.get_logger().info("Topic Converter has been launched")

        self.current_altitude = None
        self.minimum_altitude = None #update everytime 

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

        self.hardcoded_altitude = 8.0

    
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
        gps_msg.altitude = msg.altitude # needs to change we are now using lidar
        gps_msg.yaw = msg.yaw   

        self.gps_publisher.publish(gps_msg)
        self.get_logger().info(f"Published GPS lat={gps_msg.latitude}, lon={gps_msg.longitude}, alt={gps_msg.altitude}, yaw={gps_msg.yaw}")

    def new_position_callback(self, msg: LatLong):
        if self.current_mode != ArduPilotMode.GUIDED:
            self.get_logger().warning("Current mode is not GUIDED. Cannot publish new GPS position to ArduPilot.")
            return
        
        if self.current_altitude < self.minimum_altitude:
            self.minimum_altitude = self.current_altitude
  
        new_gps_msg = GlobalPosition()
        new_gps_msg.latitude = msg.latitude
        new_gps_msg.longitude = msg.longitude
        new_gps_msg.altitude = self.hardcoded_altitude  # Hardcoded altitude

        new_gps_msg.yaw = 0.0  # Default yaw, can be modified as needed

        self.new_gps_publisher.publish(new_gps_msg)
        self.get_logger().info(f"Published new GPS position to ArduPilot: lat={new_gps_msg.latitude}, lon={new_gps_msg.longitude}, alt={new_gps_msg.altitude}, yaw={new_gps_msg.yaw}")



    def takeoff(self, takeoff_altitude: float = 30.0) -> bool:
        """
        Perform a takeoff sequence:
        1) switch ArduPilot to GUIDED
        2) arm motors
        3) publish a GlobalPosition with the desired takeoff altitude


        Returns True on success, False on failure.
        """
        # 1) Switch to GUIDED via service
        try:
            mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
            if not mode_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Mode switch service not available")
                return False
            mode_req = ModeSwitch.Request()
            mode_req.mode = 4 # GUIDED mode value
            mode_fut = mode_client.call_async(mode_req)
            rclpy.spin_until_future_complete(self, mode_fut, timeout_sec=5.0)
            if mode_fut.result() is None:
                self.get_logger().error(f"Mode switch service call failed")
                return False
            self.get_logger().info(f"Switched to GUIDED mode")


            # 2) Arm motors via service
            arm_client = self.create_client(ArmMotors, '/ap/arm_motors')
            if not arm_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Arm service not available")
                return False
            arm_req = ArmMotors.Request()
            arm_req.arm = True
            arm_fut = arm_client.call_async(arm_req)
            rclpy.spin_until_future_complete(self, arm_fut, timeout_sec=5.0)
            if arm_fut.result() is None:
                self.get_logger().error(f"Arm call failed")
                return False
            self.get_logger().info("Motors armed")
       
            # 3) Publish a new GlobalPosition with desired takeoff altitude
            takeoff_client = self.create_client(Takeoff, '/ap/esperimental/takeoff')
            if not takeoff_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('Takeoff service not available')
                return False
            tk_req = Takeoff.Request()
            tk_req.alt = float(takeoff_altitude)
            tk_fut = takeoff_client.call_async(tk_req)
            rclpy.spin_until_future_complete(self, tk_fut, timeout_sec=5.0)
            if tk_fut.result() is None:
                self.get_logger().error("Takeoff call failed")
                return False
            self.get_logger().info(f'Takeoff initiated to {takeoff_altitude} meters')
            return True
        finally:
            self.destroy_node()
            rclpy.shutdown()
   


def main(args=None):
    rclpy.init(args=args)
    node = TopicConverter()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()