#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

from aav_msgs.msg import Mode, DronePosition, NewDronePosition

from .topic_converter_for_simulation import ArduPilotMode


# =========================
#  MODE MAPPING
# =========================

MODE_TO_STRING = {
    ArduPilotMode.GUIDED.value: "GUIDED",
    ArduPilotMode.LOITER.value: "LOITER",
    ArduPilotMode.RTL.value: "RTL",
    ArduPilotMode.LAND.value: "LAND",
    ArduPilotMode.POSHOLD.value: "POSHOLD",
    ArduPilotMode.STABILIZE.value: "STABILIZE",
}

STRING_TO_MODE = {
    "GUIDED": ArduPilotMode.GUIDED.value,
    "LOITER": ArduPilotMode.LOITER.value,
    "RTL": ArduPilotMode.RTL.value,
    "LAND": ArduPilotMode.LAND.value,
    "POSHOLD": ArduPilotMode.POSHOLD.value,
    "STABILIZE": ArduPilotMode.STABILIZE.value,
}


class TopicConverter(Node):
    def __init__(self):
        super().__init__("topic_converter_for_drone")
        self.get_logger().info("MAVROS Topic Converter (Hardcoded Modes) Started")

        self.home_altitude = None
        self.current_yaw = 0.0
        self.current_mode = None

        self.latest_setpoint = None
        self.last_setpoint_time = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # =========================
        # SUBSCRIBERS (MAVROS)
        # =========================

        self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            sensor_qos
        )

        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            sensor_qos
        )

        # =========================
        # SUBSCRIBERS (AAV)
        # =========================

        self.create_subscription(Mode, '/AAV/set_mode', self.set_mode_callback, 10)

        self.create_subscription(
            NewDronePosition,
            '/AAV/send_new_position',
            self.new_position_callback,
            10
        )

        # =========================
        # PUBLISHERS
        # =========================

        self.mode_pub = self.create_publisher(Mode, '/AAV/current_mode', 10)

        self.gps_pub = self.create_publisher(
            DronePosition,
            '/AAV/current_gps_position',
            10
        )

        self.setpoint_pub = self.create_publisher(
            GeoPoseStamped,
            '/mavros/setpoint_position/global',
            10
        )

        # Continuous setpoint publishing (REQUIRED by MAVROS)
        self.create_timer(0.2, self.publish_setpoint)

    # =========================
    # UTIL FUNCTIONS
    # =========================

    def quaternion_to_yaw(self, q):
        return math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y**2 + q.z**2)
        )

    def yaw_to_quaternion(self, yaw):
        return (
            math.sin(yaw / 2),
            math.cos(yaw / 2)
        )

    def call_service(self, client, req, name):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{name} not available")
            return None

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    # =========================
    # MAVROS CALLBACKS
    # =========================

    def state_callback(self, msg: State):
        self.current_mode = msg.mode

        if msg.mode in STRING_TO_MODE:
            mode_msg = Mode()
            mode_msg.mode = STRING_TO_MODE[msg.mode]
            self.mode_pub.publish(mode_msg)

    def pose_callback(self, msg: PoseStamped):
        self.current_yaw = self.quaternion_to_yaw(msg.pose.orientation)

    def gps_callback(self, msg: NavSatFix):
        if self.home_altitude is None:
            self.home_altitude = msg.altitude

        gps_msg = DronePosition()
        gps_msg.latitude = msg.latitude
        gps_msg.longitude = msg.longitude
        gps_msg.altitude = msg.altitude - self.home_altitude
        gps_msg.yaw = self.current_yaw

        self.gps_pub.publish(gps_msg)

    # =========================
    # AAV CALLBACKS
    # =========================

    def set_mode_callback(self, msg: Mode):

        if self.current_mode == "POSHOLD":
            self.get_logger().warn("Cannot switch out of POSHOLD")
            return

        if msg.mode == ArduPilotMode.TAKEOFF.value:
            self.takeoff(30.0)
            return

        if msg.mode not in MODE_TO_STRING:
            self.get_logger().error("Unknown mode")
            return

        mode_string = MODE_TO_STRING[msg.mode]

        client = self.create_client(SetMode, '/mavros/set_mode')

        req = SetMode.Request()
        req.custom_mode = mode_string

        self.call_service(client, req, "set_mode")

    def new_position_callback(self, msg: NewDronePosition):

        pose = GeoPoseStamped()
        pose.header.frame_id = "map"

        pose.pose.position.latitude = msg.latitude
        pose.pose.position.longitude = msg.longitude

        if self.home_altitude:
            pose.pose.position.altitude = msg.altitude + self.home_altitude
        else:
            pose.pose.position.altitude = msg.altitude

        yaw = getattr(msg, "yaw", self.current_yaw)
        z, w = self.yaw_to_quaternion(yaw)

        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        self.latest_setpoint = pose
        self.last_setpoint_time = time.time()

    def publish_setpoint(self):
        if self.latest_setpoint is None:
            return

        if time.time() - self.last_setpoint_time > 5:
            return

        self.latest_setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.latest_setpoint)

    # =========================
    # SERVICES
    # =========================

    def arm(self):
        client = self.create_client(CommandBool, '/mavros/cmd/arming')
        req = CommandBool.Request()
        req.value = True
        return self.call_service(client, req, "arming")

    def takeoff(self, altitude):

        # Set GUIDED
        self.set_mode_callback(Mode(mode=ArduPilotMode.GUIDED.value))

        # Arm
        self.arm()

        # Takeoff
        client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        req = CommandTOL.Request()
        req.altitude = altitude
        req.latitude = 0.0
        req.longitude = 0.0
        req.yaw = self.current_yaw

        self.call_service(client, req, "takeoff")


def main(args=None):
    rclpy.init(args=args)
    node = TopicConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()