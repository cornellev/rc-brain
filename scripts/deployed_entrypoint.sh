#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/cev/install/setup.bash

exec ros2 launch autonomy $1.py
