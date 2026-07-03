#!/usr/bin/env python3
import math
from typing import cast

import rclpy

# For GPS subscriber
# For Publisher
from aav_msgs.msg import DronePosition, TargetPosition
from pyproj import CRS, Transformer
from rclpy.node import Node
from sensor_msgs.msg import Image

# For YOLO Subscriber
from yolo_msgs.msg import DetectionArray, Point2D


class Craft:
    """Inputs from Autopilot for aircraft attitude and position at image capture."""

    def __init__(self):
        self.lat = 0.0  # degrees
        self.lon = 0.0  # degrees
        self.alt = 70.0  # meters AGL (relative to ground)
        self.roll = 0.0  # radians (unused in nadir-only approximation)
        self.pitch = 0.0  # radians (unused in nadir-only approximation)
        self.yaw = 0.0  # radians (ROS/ENU-style in your data: 0=East, +CCW)


class TargPos:
    """Info regarding position of target in various reference frames."""

    def __init__(self):
        self.x_norm = 0.5
        self.y_norm = 0.5
        self.x_dist = 0.0  # meters EAST  (+)
        self.y_dist = 0.0  # meters NORTH (+)
        self.lat = 0.0
        self.lon = 0.0


class Cam:
    """
    SIYI A8 Mini gimbaled camera.
    Using 16:9 resolution 1920x1080 and corresponding vertical FOV.
    Assumptions for this node:
      - Camera points straight down (nadir)
      - Camera yaw rotates with drone yaw
      - Camera roll/pitch relative to drone are ~0
    """

    def __init__(self):
        self.fov_hor = math.radians(81.0)
        self.fov_vert = math.radians(51.3)
        self.x_res = None
        self.y_res = None
        self.fov_hor_dist = 0.0
        self.fov_vert_dist = 0.0

    def update_resolution(self, width: int, height: int):
        self.x_res = int(width)
        self.y_res = int(height)

    def has_resolution(self):
        return self.x_res is not None and self.y_res is not None


def calc_targ_dist(craft: Craft, targ_pos: TargPos, cam: Cam):
    """
    Compute target ground offsets (EAST, NORTH) in meters using a nadir camera approximation.

    Pixel conventions:
      - x increases to the RIGHT
      - y increases DOWN

    Body conventions assumed:
      - forward = +X_body
      - right   = +Y_body

    Given your measured yaw:
      - craft.yaw is radians
      - 0 ~= East, +pi/2 ~= North, -pi/2 ~= South, -pi ~= West
      => ENU/ROS yaw: 0 along +East, positive CCW toward North

    Mapping:
      dx (image right)  -> +right
      dy (image down)   -> -forward
    Then rotate (forward,right) into ENU (east,north) using ENU yaw.
    """
    # Ground footprint (meters) at altitude AGL
    cam.fov_hor_dist = 2.0 * craft.alt * math.tan(cam.fov_hor / 2.0)
    cam.fov_vert_dist = 2.0 * craft.alt * math.tan(cam.fov_vert / 2.0)

    # Image -> ground in body axes (nadir camera)
    dx = (targ_pos.x_norm - 0.5) * cam.fov_hor_dist  # +right
    dy = (targ_pos.y_norm - 0.5) * cam.fov_vert_dist  # +down

    right = dx
    forward = -dy

    # Rotate body (forward,right) into ENU using ENU yaw (0=East, +CCW)
    psi = craft.yaw
    east = forward * math.cos(psi) + right * math.sin(psi)
    north = forward * math.sin(psi) - right * math.cos(psi)

    targ_pos.x_dist = east
    targ_pos.y_dist = north
    return targ_pos, cam


def calc_targ_loc(craft: Craft, targ_pos: TargPos):
    """
    Convert EN offsets (meters) to lat/lon using UTM as a local metric projection.
    Assumes targ_pos.x_dist is EAST (+) and targ_pos.y_dist is NORTH (+).
    """
    # Pick UTM zone based on craft longitude
    utm_zone = math.floor((craft.lon + 180.0) / 6.0) + 1
    is_northern = craft.lat >= 0.0

    crs_ll = CRS.from_epsg(4326)
    crs_utm = CRS.from_dict(
        {"proj": "utm", "zone": utm_zone, "datum": "WGS84", "south": not is_northern}
    )

    to_utm = Transformer.from_crs(crs_ll, crs_utm, always_xy=True)
    to_ll = Transformer.from_crs(crs_utm, crs_ll, always_xy=True)

    # lon,lat -> easting,northing
    craft_e, craft_n = to_utm.transform(craft.lon, craft.lat)

    # Apply EN offsets
    targ_e = craft_e + targ_pos.x_dist
    targ_n = craft_n + targ_pos.y_dist

    # back to lon,lat
    targ_lon, targ_lat = to_ll.transform(targ_e, targ_n)

    targ_pos.lon = targ_lon
    targ_pos.lat = targ_lat
    return craft, targ_pos


class ManavsMagicCode(Node):
    def __init__(self):
        super().__init__("manavs_magic_code")

        self.image_topic = cast(
            str,
            self.declare_parameter("image_topic", "/siyi_a8/image_raw").value,
        )

        self.gps_sub = self.create_subscription(
            DronePosition, "AAV/current_gps_position", self.update_craft_gps, 10
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.update_camera_resolution, 10
        )
        self.yolo_sub = self.create_subscription(
            DetectionArray, "/yolo/detections", self.update_targ_gps, 10
        )
        self.publisher = self.create_publisher(
            TargetPosition, "AAV/estimated_target_position", 10
        )

        self.craft = Craft()
        self.targ_pos = TargPos()
        self.cam = Cam()

        self.get_logger().info("Manav's Magic Code has been launched.")

    def update_craft_gps(self, msg_in: DronePosition):
        self.craft.lat = float(msg_in.latitude)
        self.craft.lon = float(msg_in.longitude)
        self.craft.alt = float(msg_in.altitude)  # AGL per your note

        # If you later add roll/pitch, keep them in radians.
        # For now (nadir + level assumption), leave them at 0.
        self.craft.roll = 0.0
        self.craft.pitch = 0.0

        # Your measured yaw values indicate radians already with ENU/ROS-style meaning.
        # So do NOT wrap with math.radians() here.
        self.craft.yaw = float(msg_in.yaw)

    def update_camera_resolution(self, msg_in: Image):
        previous_resolution = (self.cam.x_res, self.cam.y_res)
        self.cam.update_resolution(msg_in.width, msg_in.height)

        if previous_resolution != (self.cam.x_res, self.cam.y_res):
            self.get_logger().info(
                f"Camera resolution updated to {self.cam.x_res}x{self.cam.y_res} from {self.image_topic}"
            )

    def update_targ_gps(self, msg_in: DetectionArray):
        detections = list(msg_in.detections)
        if len(detections) == 0:
            return

        if not self.cam.has_resolution():
            self.get_logger().warning(
                "Waiting for camera image resolution before processing detections."
            )
            return

        x_res = self.cam.x_res
        y_res = self.cam.y_res
        assert x_res is not None
        assert y_res is not None

        # Assume first detection for now
        det = detections[0]
        center: Point2D = det.bbox.center.position

        # Normalize pixel coords
        # Expect center.x in [0..x_res], center.y in [0..y_res]
        self.targ_pos.x_norm = float(center.x) / float(x_res)
        self.targ_pos.y_norm = float(center.y) / float(y_res)

        # Clamp just in case upstream gives slightly out-of-range values
        self.targ_pos.x_norm = max(0.0, min(1.0, self.targ_pos.x_norm))
        self.targ_pos.y_norm = max(0.0, min(1.0, self.targ_pos.y_norm))

        # Compute target EN offsets + lat/lon
        self.targ_pos, self.cam = calc_targ_dist(self.craft, self.targ_pos, self.cam)
        self.craft, self.targ_pos = calc_targ_loc(self.craft, self.targ_pos)

        msg_out = TargetPosition()
        msg_out.object_label = det.class_name
        msg_out.longitude = float(self.targ_pos.lon)
        msg_out.latitude = float(self.targ_pos.lat)
        self.publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = ManavsMagicCode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
