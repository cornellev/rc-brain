#!/bin/bash
set -e

# setup ros2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /root/hogsmeade/install/setup.bash

ros2 launch autonomy launch.py