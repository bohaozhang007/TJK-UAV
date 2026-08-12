#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/jkhk/planner/devel/setup.bash
export ROS_PACKAGE_PATH=/home/jkhk/TJK-UAV/ros:/home/jkhk/planner/release:${ROS_PACKAGE_PATH:-}

exec roslaunch i7_nav i7_interactive.launch
