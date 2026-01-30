#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# For Manav's Code
from math import *
import numpy as np
import pyproj
# For GPS subscriber
from aav_msgs import DronePosition
# For YOLO Subscriber
from yolo_msgs import Pose2d
# For Publisher
from aav_msgs import TargetPosition

"""
This code outputs the position (lat, lon) of a target after being inputted with various variables.
The following inputs are required: craft.roll, craft.pitch, craft.yaw, craft.alt, craft_lat, craft_lon, targ_pos.x_norm, targ_pos.y_norm.
The following values are hardcode: cam.fov_hor, cam.fov_vert.
Example values given in code should be overwritten somehow based on aircraft/image data.
"""

class ManavsMagicCode(Node):
    def __init__(self):
        super().__init__("manavs_magic_code")
        
        self.gps_sub = self.create_subscription(DronePosition, "AAV/current_gps_position", self.update_craft_gps, 10)
        self.yolo_sub = self.create_subscription(Pose2d, "/yolo/tracking", self.manavacadabra, 10)
        self.publisher = self.create_publisher(TargetPosition, "AAV/estimated_target_position", 10)

        self.craft = Craft()

        self.get_logger().info("Manav's Magic Code has been launched.")


    def update_craft_gps(self, msg_in: DronePosition):
        self.craft.lat = msg_in.latitude
        self.craft.lon = msg_in.longitude
        self.craft.alt = msg_in.altitude
        self.craft.roll = 0 # radians(msg_in.roll)
        self.craft.pitch = 0 # radians(msg_in.pitch)
        self.craft.yaw = radians(msg_in.yaw)


    def manavacadabra(self, msg_in: Pose2d):
        targ_pos, cam = calc_targ_dist(craft, targ_pos, cam)
        craft, targ_pos = calc_targ_loc(craft, targ_pos)
        msg_out = TargetPosition()
        msg_out.object_label = msg_in.object_label
        msg_out.longitude = targ_pos.lon
        msg_out.latitude = targ_pos.lat
        self.publisher.publish(msg_out)


""" Inputs from Autopilot for Aircraft Attitude and Position at Image Capture """
class Craft:
    lat = 0 # Input, Latitude
    lon = 0 # Input, Longitude
    alt = 70 # Input, Altitude (m)
    roll = radians(0) # Input, Roll (Radians) | Range from -PI/2 (Roll Left) to PI/2 (Roll Right)
    pitch = radians(0) # Input, Pitch (Radians) | Range from -PI/2 (Nose Down) to PI/2 (Nose Up)
    yaw = radians(0) # Input, Heading (Radians) | Range from 0 to 2PI (North) with Clockwise Rotation Being Positive


""" Info Regarding Position of Target in Various Reference Frames """
class targ_pos:
    x_norm = 0.5 # Input, Normalized Position of Target | 0 = Leftmost Edge, 0.5 = Middle, 1 = Rightmost Edge
    y_norm = 0.5 # Input, Normalized Position of Target, | 0 = Top Edge, 0.5 = Middle, 1 = Bottom Edge


""" Info Regarding Camera Specifications """
class cam:
    # TODO: Ensure this is the *actual* camera FOV
    fov_hor = radians(127) # Horizontal FOV of Camera (Radians)
    fov_vert = radians(95) # Horizontal FOV of Camera (Radians)


""" Calculate Distance (m) Between Image and Target Center """
def calc_targ_dist(craft, targ_pos, cam):
    # Calculate Image FOV Coverage in Terms of Distance (m)
    cam.fov_hor_dist = 2 * craft.alt * tan(cam.fov_hor/2)
    cam.fov_vert_dist = 2 * craft.alt * tan(cam.fov_vert/2)

    # Calculate Distance (m) Between Image and Target Center Assuming ZERO Attitude (Level and Facing North)
    targ_pos.x_dist = cam.fov_hor_dist * (targ_pos.x_norm - 0.5) # Positive is Left, Negative Right
    targ_pos.y_dist = -cam.fov_vert_dist * (targ_pos.y_norm - 0.5) # Positive is Up, Negative Down

    # Form Rotation Matrix for Attitude Integration
    Rx = np.array([[1, 0, 0], [0, cos(craft.roll), -sin(craft.roll)], [0, sin(craft.roll), cos(craft.roll)]])
    Ry = np.array([[cos(craft.pitch), 0, sin(craft.pitch)], [0, 1, 0], [-sin(craft.pitch), 0, cos(craft.pitch)]])
    Rz = np.array([[cos(craft.yaw), -sin(craft.yaw), 0], [sin(craft.yaw), cos(craft.yaw), 0], [0, 0, 1]])
    rotmat_rpy = np.matmul(np.matmul(Rz, Ry), Rx)

    # Apply Rotation Matrix to Distance Data for Attitude Integration
    targ_pos_mat = [targ_pos.y_dist, targ_pos.x_dist, craft.alt]
    targ_pos_trans = np.matmul(rotmat_rpy, targ_pos_mat)
    scale = targ_pos_mat[2]/targ_pos_trans[2]
    targ_pos_trans = targ_pos_trans * scale

    # Save New Distances Based on Attitude
    targ_pos.x_dist, targ_pos.y_dist = targ_pos_trans[1], targ_pos_trans[0]
    
    return targ_pos, cam


""" Calculate Target Location (Lat, Lon) """
def calc_targ_loc(craft, targ_pos):
    # Convert Aircraft Posiition from Lat/Lon to UTM
    UTM_zone = ceil((craft.lon + 180) / 6)
    proj_latlon = pyproj.Proj(proj='latlong',datum='WGS84')
    proj_xy = pyproj.Proj(proj="utm", zone=UTM_zone, datum='WGS84')
    craft.y_UTM, craft.x_UTM = pyproj.transform(proj_latlon, proj_xy, craft.lon, craft.lat)

    # Add Target Offset Distance to Aircraft UTM Position
    targ_pos.x_UTM = craft.x_UTM + targ_pos.y_dist
    targ_pos.y_UTM = craft.y_UTM + targ_pos.x_dist

    # Convert Target Position from UTM to Lat/Lon
    targ_pos.lon, targ_pos.lat = pyproj.transform(proj_xy, proj_latlon, targ_pos.y_UTM, targ_pos.x_UTM) 

    return craft, targ_pos


def main(args=None):
    rclpy.init(args=args)
    node = ManavsMagicCode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
