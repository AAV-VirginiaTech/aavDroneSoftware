#!/usr/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ardupilot_msgs.msg import Status
from aav_msgs.msg import Mode
from geographic_msgs.msg import GeoPoseStamped
from ardupilot_msgs.msg import GlobalPosition 
from aav_msgs.msg import DronePosition
from aav_msgs.msg import NewDronePosition
from enum import IntEnum
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import Takeoff



#Testing commands:
"""
# Send new position
ros2 topic pub --once /AAV/send_new_position aav_msgs/msg/NewDronePosition "{
  latitude: 37.2295,
  longitude: -80.4138,
  altitude: 20.0
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
  latitude: -35.363123,
  longitude: 149.16586614,
  altitude: 592.0
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
        super().__init__("topic_converter_for_simulation")
        self.get_logger().info("Topic Converter has been launched")

        self.minimum_altitude = None

        self.current_mode = None
        
        # Rate limiting for new position callback (max 1 per 5 seconds)
        self.last_new_gps_publish_time = 0.0
        self.rate_limit_interval = 5.0

        # Subscriber(Mode): ArduPilot -> TC
        self.status_subscriber = self.create_subscription(Status, '/ap/status', self.status_callback, 10)
        # Publisher(Mode): TC -> AAV Software
        self.mode_publisher = self.create_publisher(Mode, '/AAV/current_mode', 10)

        # QoS profile for sensor data (BEST_EFFORT reliability)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber(GeoPoseStamped): ArduPilot -> TC
        self.global_position_subscriber = self.create_subscription(GeoPoseStamped, '/ap/geopose/filtered', self.global_position_callback, sensor_qos)
        # Publisher(GlobalPosition): TC -> AAV Software
        self.gps_publisher = self.create_publisher(DronePosition, '/AAV/current_gps_position', 10)

        #Subscriber(NewPosition): AAV Software -> TC
        self.new_position_subscriber = self.create_subscription(NewDronePosition, '/AAV/send_new_position', self.new_position_callback, 10)
        #Publisher(NewPosition): TC -> ArduPilot
        self.new_gps_publisher = self.create_publisher(GlobalPosition, '/ap/cmd_gps_pose', 10)

        self.set_mode = self.create_subscription(Mode, '/AAV/set_mode', self.set_mode_callback, 10)

    def check_rate_limit(self, last_publish_time: float) -> tuple:
        """
        Check if enough time has passed since the last publish.
        Returns: (should_publish, updated_time)
        """
        
       
        current_time = time.time()
        if current_time - last_publish_time >= self.rate_limit_interval:
            return True, current_time
        return False, last_publish_time
    
    def set_mode_callback(self, msg: Mode):
        # Prevent mode switching if drone is in position hold mode
        if self.current_mode == ArduPilotMode.POSHOLD:
            self.get_logger().warn(f"Cannot switch modes while in POSHOLD. Current mode: {self.current_mode.name}")
            return
        
        if msg.mode == ArduPilotMode.TAKEOFF.value:
            self.get_logger().info("Received takeoff command from AAV Software")
            success = self.takeoff(takeoff_altitude=30.0)
            if success:
                self.get_logger().info("Takeoff sequence executed successfully")
            else:
                self.get_logger().error("Takeoff sequence failed")
        else:
            self.call_mode_switch(msg.mode)

    def status_callback(self, msg: Status):
        ap_mode = ArduPilotMode(msg.mode)

        if ap_mode != self.current_mode:
            self.get_logger().info(f"Mode changed: {ap_mode.name} ({ap_mode.value})")

        self.current_mode = ap_mode

        mode_msg = Mode()
        mode_msg.mode = ap_mode.value
        
        self.mode_publisher.publish(mode_msg)

    def global_position_callback(self, msg: GeoPoseStamped):
     
        if (self.minimum_altitude is None) or (self.minimum_altitude == 0.0):
            self.minimum_altitude = msg.pose.position.altitude
        elif msg.pose.position.altitude < self.minimum_altitude:
            self.minimum_altitude = msg.pose.position.altitude
        
        gps_msg = DronePosition()
        gps_msg.latitude = msg.pose.position.latitude
        gps_msg.longitude = msg.pose.position.longitude
        gps_msg.altitude = msg.pose.position.altitude - self.minimum_altitude
        # Extract yaw from quaternion orientation
    
        q = msg.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        gps_msg.yaw = yaw   

        self.gps_publisher.publish(gps_msg)

    def new_position_callback(self, msg: NewDronePosition):
        new_gps_msg = GlobalPosition()

        new_gps_msg.header.frame_id = "map"

        new_gps_msg.coordinate_frame = 5  # GLOBAL (absolute altitude)

        new_gps_msg.latitude = msg.latitude
        new_gps_msg.longitude = msg.longitude

        if self.minimum_altitude is not None:
            new_gps_msg.altitude = msg.altitude + self.minimum_altitude
   
        # Rate limit new GPS position publishing
        should_publish, self.last_new_gps_publish_time = self.check_rate_limit(self.last_new_gps_publish_time)
        if should_publish:
            self.new_gps_publisher.publish(new_gps_msg)
            self.get_logger().info(f"Published new GPS position to ArduPilot: lat={new_gps_msg.latitude}, lon={new_gps_msg.longitude}, alt={new_gps_msg.altitude}, yaw={new_gps_msg.yaw}")

    def call_mode_switch(self, mode: int = 4) -> bool:
        try:
            client = self.create_client(ModeSwitch, '/ap/mode_switch')

            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('Service /ap/mode_switch not available')
                return False

            req = ModeSwitch.Request()
            req.mode = mode
            future = client.call_async(req)

            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is not None:
                self.get_logger().info(f'Mode switch completed: {future.result()}')
                success = True
            else:
                self.get_logger().error('Mode switch service call failed')
                success = False

            return success
        except Exception as e:
            self.get_logger().error(f'Mode switch failed with exception: {e}')
            return False

    def takeoff(self, takeoff_altitude: float = 30.0) -> bool:
        """
        Perform a takeoff sequence:
        1) switch ArduPilot to GUIDED
        2) arm motors
        3) publish a GlobalPosition with the desired takeoff altitude


        Returns True on success, False on failure.
        """
        # 1) Switch to GUIDED via service
        mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        if not mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Mode switch service not available")
            return False
        mode_req = ModeSwitch.Request()
        mode_req.mode = 4 # GUIDED mode value
        mode_fut = mode_client.call_async(mode_req)
        rclpy.spin_until_future_complete(self, mode_fut, timeout_sec=5.0)
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
        self.get_logger().info("Motors armed")
    
        # 3) Publish a new GlobalPosition with desired takeoff altitude
        takeoff_client = self.create_client(Takeoff, '/ap/experimental/takeoff')
        if not takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Takeoff service not available')
            return False
        tk_req = Takeoff.Request()
        if self.minimum_altitude is not None:
            tk_req.alt = float(takeoff_altitude) + self.minimum_altitude
        else:
            tk_req.alt = float(takeoff_altitude)
        tk_fut = takeoff_client.call_async(tk_req)
        rclpy.spin_until_future_complete(self, tk_fut, timeout_sec=5.0)
        self.get_logger().info(f'Takeoff initiated to {takeoff_altitude} meters')
        return True
   


def main(args=None):
    rclpy.init(args=args)
    node = TopicConverter()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()