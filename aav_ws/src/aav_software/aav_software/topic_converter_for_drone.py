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
from enum import Enum, IntEnum
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import Takeoff
from .topic_converter_for_simulation import ArduPilotMode


# TODO Update this file to work with MAVROS instead of ardupilot for the actual drone
# Resource: https://www.notion.so/vtaav/Ardupilot-to-MavROS-topic-conversions-32e623fcf7fe808495a9fc30ba85e564?source=copy_link



class TopicConverter(Node):
    def __init__(self):
        super().__init__("topic_converter_for_drone")
        self.get_logger().info("Topic Converter has been launched")

        self.minimum_altitude = None
        
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
      
        if ap_mode != self.last_mode:
            self.get_logger().info(f"Mode changed: {ap_mode.name} ({ap_mode.value})")
            self.last_mode = ap_mode

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