#!/usr/bin/env bash
set -e

echo "=== AAV postCreate.sh starting ==="

# ---- CONFIG ----
ROS_DISTRO=humble
WS=/workspaces/aavDroneSoftware/aav_ws
SRC=${WS}/src
PKG=${SRC}/ardupilot_msgs

# ---- ROS ENV ----
source /opt/ros/${ROS_DISTRO}/setup.bash

# ---- ENSURE WORKSPACE EXISTS ----
mkdir -p "${SRC}"

# ---- FETCH ardupilot_msgs ONLY (no full clone) ----
if [ ! -d "${PKG}" ]; then
    echo "Cloning ardupilot_msgs (only)..."
    apt-get update && apt-get install -y subversion
    svn export \
      https://github.com/ArduPilot/ardupilot/trunk/Tools/ros2/ardupilot_msgs \
      "${PKG}"
else
    echo "ardupilot_msgs already present, skipping clone"
fi

echo "=== AAV postCreate.sh complete ==="
