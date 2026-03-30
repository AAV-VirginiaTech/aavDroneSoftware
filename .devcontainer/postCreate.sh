#!/usr/bin/env bash
set -e

echo "=== AAV postCreate.sh starting ==="

# ---- CONFIG ----
ROS_DISTRO=humble
WS=/workspaces/aavDroneSoftware/aav_ws
SRC=${WS}/src
PKG=${SRC}/ardupilot_msgs
YOLO_PKG=${SRC}/yolo_msgs
MAVROS_PKG=${SRC}/mavros_msgs

# ---- ROS ENV ----
source /opt/ros/${ROS_DISTRO}/setup.bash

# ---- ENSURE WORKSPACE EXISTS ----
mkdir -p "${SRC}"

# ---- FETCH ardupilot_msgs ONLY (git sparse-checkout) ----
if [ ! -d "${PKG}" ]; then
  echo "Fetching ardupilot_msgs via sparse-checkout..."
  tmp=/tmp/ardupilot_sparse
  rm -rf "$tmp"
  git clone --filter=blob:none --no-checkout https://github.com/ArduPilot/ardupilot.git "$tmp"
  cd "$tmp"
  git sparse-checkout init --cone
  git sparse-checkout set Tools/ros2/ardupilot_msgs
  git checkout master

  mkdir -p "${SRC}"
  cp -a Tools/ros2/ardupilot_msgs "${PKG}"
  cd "$WS"  # return before removing the temp dir
  rm -rf "$tmp"
else
  echo "ardupilot_msgs already present, skipping fetch"
fi

# ---- FETCH yolo_msgs ONLY (git sparse-checkout) ----
if [ ! -d "${YOLO_PKG}" ]; then
  echo "Fetching yolo_msgs via sparse-checkout..."
  tmp=/tmp/yolo_sparse
  rm -rf "$tmp"
  git clone --filter=blob:none --no-checkout https://github.com/mgonzs13/yolo_ros.git "$tmp"
  cd "$tmp"
  git sparse-checkout init --cone
  git sparse-checkout set yolo_msgs
  git checkout

  mkdir -p "${SRC}"
  cp -a yolo_msgs "${YOLO_PKG}"
  cd "$WS"  # return before removing the temp dir
  rm -rf "$tmp"
else
  echo "yolo_msgs already present, skipping fetch"
fi

# ---- FETCH mavros_msgs ONLY (git sparse-checkout) ----
if [ ! -d "${MAVROS_PKG}" ]; then
  echo "Fetching mavros_msgs via sparse-checkout..."
  tmp=/tmp/mavros_sparse
  rm -rf "$tmp"
  git clone --filter=blob:none --no-checkout https://github.com/mavlink/mavros.git "$tmp"
  cd "$tmp"
  git sparse-checkout init --cone
  git sparse-checkout set mavros_msgs
  git checkout ros2

  mkdir -p "${SRC}"
  cp -a mavros_msgs "${MAVROS_PKG}"
  cd "$WS"  # return before removing the temp dir
  rm -rf "$tmp"
else
  echo "mavros_msgs already present, skipping fetch"
fi

# ---- UPDATE APT CACHE ----
echo "Updating apt cache..."
apt-get update

# ---- INSTALL DEPENDENCIES ----
echo "Installing ROS dependencies..."
rosdep install --from-paths "${SRC}" --ignore-src -r -y

# ---- BUILD WORKSPACE ----
echo "Building workspace..."
colcon build

echo "=== AAV postCreate.sh complete ==="
