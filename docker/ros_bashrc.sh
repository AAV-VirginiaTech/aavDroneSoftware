#!/bin/bash
# shellcheck disable=SC1090,SC1091

export PYTHONWARNINGS="ignore:easy_install command is deprecated"

# Base ROS
source /opt/ros/humble/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble

# Workspace overlay (only if built)
if [ -f /workspaces/aavDroneSoftware/aav_ws/install/setup.bash ]; then
  source /workspaces/aavDroneSoftware/aav_ws/install/setup.bash
fi
