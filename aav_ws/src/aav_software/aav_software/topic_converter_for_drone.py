#!/usr/bin/env python3
import math
import subprocess
import time

import rclpy
from aav_msgs.msg import DronePosition, Mode, NewDronePosition
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix

from .topic_converter_for_simulation import ArduPilotMode

# =========================
#  MODE MAPPING
# =========================

MODE_TO_STRING = {
    ArduPilotMode.STABILIZE.value: "STABILIZE",
    ArduPilotMode.ACRO.value: "ACRO",
    ArduPilotMode.ALT_HOLD.value: "ALT_HOLD",
    ArduPilotMode.GUIDED.value: "GUIDED",
    ArduPilotMode.AUTO.value: "AUTO",
    ArduPilotMode.LOITER.value: "LOITER",
    ArduPilotMode.RTL.value: "RTL",
    ArduPilotMode.CIRCLE.value: "CIRCLE",
    8: "POSITION",
    ArduPilotMode.LAND.value: "LAND",
    10: "OF_LOITER",
    ArduPilotMode.DRIFT.value: "DRIFT",
    ArduPilotMode.SPORT.value: "SPORT",
    ArduPilotMode.FLIP.value: "FLIP",
    ArduPilotMode.AUTOTUNE.value: "AUTOTUNE",
    ArduPilotMode.POSHOLD.value: "POSHOLD",
    ArduPilotMode.BRAKE.value: "BRAKE",
    ArduPilotMode.THROW.value: "THROW",
    ArduPilotMode.AVOID_ADSB.value: "AVOID_ADSB",
    ArduPilotMode.GUIDED_NOGPS.value: "GUIDED_NOGPS",
    ArduPilotMode.SMART_RTL.value: "SMART_RTL",
    ArduPilotMode.FLOWHOLD.value: "FLOWHOLD",
    ArduPilotMode.FOLLOW.value: "FOLLOW",
    ArduPilotMode.ZIGZAG.value: "ZIGZAG",
    ArduPilotMode.SYSTEMID.value: "SYSTEMID",
    ArduPilotMode.AUTOROTATE.value: "AUTOROTATE",
    ArduPilotMode.AUTO_RTL.value: "AUTO_RTL",
    ArduPilotMode.TURTLE.value: "TURTLE",
}

STRING_TO_MODE = {
    "STABILIZE": ArduPilotMode.STABILIZE.value,
    "ACRO": ArduPilotMode.ACRO.value,
    "ALT_HOLD": ArduPilotMode.ALT_HOLD.value,
    "GUIDED": ArduPilotMode.GUIDED.value,
    "AUTO": ArduPilotMode.AUTO.value,
    "LOITER": ArduPilotMode.LOITER.value,
    "RTL": ArduPilotMode.RTL.value,
    "CIRCLE": ArduPilotMode.CIRCLE.value,
    "LAND": ArduPilotMode.LAND.value,
    "DRIFT": ArduPilotMode.DRIFT.value,
    "APPROACH": ArduPilotMode.DRIFT.value,
    "SPORT": ArduPilotMode.SPORT.value,
    "FLIP": ArduPilotMode.FLIP.value,
    "AUTOTUNE": ArduPilotMode.AUTOTUNE.value,
    "POSHOLD": ArduPilotMode.POSHOLD.value,
    "BRAKE": ArduPilotMode.BRAKE.value,
    "THROW": ArduPilotMode.THROW.value,
    "AVOID_ADSB": ArduPilotMode.AVOID_ADSB.value,
    "GUIDED_NOGPS": ArduPilotMode.GUIDED_NOGPS.value,
    "SMART_RTL": ArduPilotMode.SMART_RTL.value,
    "FLOWHOLD": ArduPilotMode.FLOWHOLD.value,
    "FOLLOW": ArduPilotMode.FOLLOW.value,
    "ZIGZAG": ArduPilotMode.ZIGZAG.value,
    "SYSTEMID": ArduPilotMode.SYSTEMID.value,
    "AUTOROTATE": ArduPilotMode.AUTOROTATE.value,
    "AUTO_RTL": ArduPilotMode.AUTO_RTL.value,
    "TURTLE": ArduPilotMode.TURTLE.value,
    # MAVROS exposes these copter mode names for numeric values not in ArduPilot's standard list.
    "POSITION": 8,
    "OF_LOITER": 10,
}


class TopicConverter(Node):
    def __init__(self):
        super().__init__("topic_converter_for_drone")
        self.get_logger().info("MAVROS Topic Converter (Hardcoded Modes) Started")
        self.check_geoid_eval()

        self.minimum_altitude = None
        self.minimum_latitude = None
        self.minimum_longitude = None
        self.minimum_altitude_amsl = None
        self.current_latitude = None
        self.current_longitude = None
        self.current_yaw = 0.0
        self.current_mode = None

        self.last_setpoint_publish_time = 0.0
        self.setpoint_rate_limit_interval = 5.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # =========================
        # SUBSCRIBERS (MAVROS)
        # =========================

        self.create_subscription(State, "/mavros/state", self.state_callback, 10)

        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self.gps_callback, sensor_qos
        )

        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_callback, sensor_qos
        )

        # =========================
        # SUBSCRIBERS (AAV)
        # =========================

        self.create_subscription(Mode, "/AAV/set_mode", self.set_mode_callback, 10)

        self.create_subscription(
            NewDronePosition, "/AAV/send_new_position", self.new_position_callback, 10
        )

        # =========================
        # PUBLISHERS
        # =========================

        self.mode_pub = self.create_publisher(Mode, "/AAV/current_mode", 10)

        self.gps_pub = self.create_publisher(
            DronePosition, "/AAV/current_gps_position", 10
        )

        self.setpoint_pub = self.create_publisher(
            GeoPoseStamped, "/mavros/setpoint_position/global", 10
        )

    # =========================
    # UTIL FUNCTIONS
    # =========================

    def quaternion_to_yaw(self, q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

    def yaw_to_quaternion(self, yaw):
        return (math.sin(yaw / 2), math.cos(yaw / 2))

    def check_geoid_eval(self):
        """Check the executable and EGM96 dataset without needing GPS telemetry."""
        try:
            result = subprocess.run(
                ["GeoidEval", "-n", "egm96-5"],
                input="0 0\n",
                text=True,
                capture_output=True,
                check=True,
                timeout=1.0,
            )
            separation = float(result.stdout.strip())
            if not math.isfinite(separation):
                raise ValueError(f"returned nonfinite separation {separation!r}")
        except FileNotFoundError:
            self.get_logger().error(
                "Startup geoid check failed: GeoidEval executable not found. "
                "Install it with: sudo apt install geographiclib-tools. "
                "Position commands requiring altitude conversion will be rejected."
            )
        except subprocess.CalledProcessError as exc:
            details = (exc.stderr or exc.stdout or str(exc)).strip()
            self.get_logger().error(
                "Startup geoid check failed: GeoidEval could not load egm96-5 "
                f"({details}). Install the dataset with: "
                "sudo geographiclib-get-geoids egm96-5. "
                "Position commands requiring altitude conversion will be rejected."
            )
        except (ValueError, OSError, subprocess.SubprocessError) as exc:
            self.get_logger().error(
                f"Startup geoid check failed: {exc}. "
                "Install or repair them with: sudo apt install geographiclib-tools; "
                "sudo geographiclib-get-geoids egm96-5. "
                "Position commands requiring altitude conversion will be rejected."
            )
        else:
            self.get_logger().info(
                f"Startup geoid check passed: egm96-5 separation at (0, 0) "
                f"is {separation:.3f} m"
            )

    def get_minimum_altitude_amsl(self):
        """Convert the running minimum GPS altitude to AMSL."""
        if self.minimum_altitude is None:
            raise ValueError("waiting for a GPS reading")
        if self.minimum_altitude_amsl is None:
            # Incoming GPS is ellipsoid height; setpoint_position/global expects
            # AMSL. EGM96 separation is subtracted, including when it is negative.
            result = subprocess.run(
                ["GeoidEval", "-n", "egm96-5"],
                input=f"{self.minimum_latitude} {self.minimum_longitude}\n",
                text=True,
                capture_output=True,
                check=True,
                timeout=1.0,
            )
            minimum_amsl = self.minimum_altitude - float(result.stdout.strip())
            if not math.isfinite(minimum_amsl):
                raise ValueError("invalid minimum AMSL altitude")
            self.minimum_altitude_amsl = minimum_amsl
        return self.minimum_altitude_amsl

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
        # Match the simulation: keep the lowest observed filtered altitude as
        # the relative-altitude origin. MAVROS reports this altitude as ellipsoid
        # height, so invalidate the AMSL conversion whenever the minimum changes.
        if (
            self.minimum_altitude is None
            or msg.altitude < self.minimum_altitude
        ):
            self.minimum_altitude = msg.altitude
            self.minimum_latitude = msg.latitude
            self.minimum_longitude = msg.longitude
            self.minimum_altitude_amsl = None

        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude

        gps_msg = DronePosition()
        gps_msg.latitude = msg.latitude
        gps_msg.longitude = msg.longitude
        gps_msg.altitude = msg.altitude - self.minimum_altitude
        gps_msg.yaw = self.current_yaw

        self.gps_pub.publish(gps_msg)

    # =========================
    # AAV CALLBACKS
    # =========================

    def set_mode_callback(self, msg: Mode):

        if self.current_mode == "LOITER":
            self.get_logger().warn("Cannot switch out of LOITER")
            return

        if msg.mode == ArduPilotMode.TAKEOFF.value:
            self.takeoff(30.0)
            return

        if msg.mode not in MODE_TO_STRING:
            self.get_logger().error("Unknown mode")
            return

        mode_string = MODE_TO_STRING[msg.mode]

        client = self.create_client(SetMode, "/mavros/set_mode")

        req = SetMode.Request()
        req.custom_mode = mode_string

        self.call_service(client, req, "set_mode")

    def new_position_callback(self, msg: NewDronePosition):

        pose = GeoPoseStamped()
        pose.header.frame_id = "map"

        pose.pose.position.latitude = msg.latitude
        pose.pose.position.longitude = msg.longitude

        # Correct the requested relative altitude using the running minimum.
        try:
            pose.pose.position.altitude = msg.altitude + self.get_minimum_altitude_amsl()
        except FileNotFoundError:
            self.get_logger().error(
                "GeoidEval executable not found. Install it on the drone with: "
                "sudo apt install geographiclib-tools. Position command not sent.",
                throttle_duration_sec=5.0,
            )
            return
        except subprocess.CalledProcessError as exc:
            details = (exc.stderr or exc.stdout or str(exc)).strip()
            self.get_logger().error(
                f"GeoidEval failed: {details}. "
                "If the egm96-5.pgm dataset is missing, install it on the drone with: "
                "sudo geographiclib-get-geoids egm96-5. Position command not sent.",
                throttle_duration_sec=5.0,
            )
            return
        except (ValueError, OSError, subprocess.SubprocessError) as exc:
            self.get_logger().error(
                f"Cannot convert position altitude: {exc}. "
                "Check the GPS reading, geographiclib-tools and egm96-5 data.",
                throttle_duration_sec=5.0,
            )
            return

        yaw = getattr(msg, "yaw", self.current_yaw)
        z, w = self.yaw_to_quaternion(yaw)

        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        now = time.time()
        if (
            now - self.last_setpoint_publish_time
            < self.setpoint_rate_limit_interval
        ):
            return

        pose.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(pose)
        self.last_setpoint_publish_time = now

    # =========================
    # SERVICES
    # =========================

    def arm(self):
        client = self.create_client(CommandBool, "/mavros/cmd/arming")
        req = CommandBool.Request()
        req.value = True
        return self.call_service(client, req, "arming")

    def takeoff(self, altitude):

        # Set GUIDED
        self.set_mode_callback(Mode(mode=ArduPilotMode.GUIDED.value))

        # Arm
        self.arm()

        # Takeoff
        client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        req = CommandTOL.Request()
        req.altitude = altitude
        req.latitude = (
            self.current_latitude if self.current_latitude is not None else 0.0
        )
        req.longitude = (
            self.current_longitude if self.current_longitude is not None else 0.0
        )
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
