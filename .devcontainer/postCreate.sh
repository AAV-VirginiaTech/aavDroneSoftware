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
  rm -rf "$tmp"
else
  echo "ardupilot_msgs already present, skipping fetch"
fi


echo "=== AAV postCreate.sh complete ==="
