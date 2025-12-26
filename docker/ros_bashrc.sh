#!/bin/bash
# shellcheck disable=SC1090,SC1091

export PYTHONWARNINGS="ignore:easy_install command is deprecated"

# To be appended to /root/.bashrc
source /opt/ros/humble/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh

export _colcon_cd_root=/opt/ros/humble
