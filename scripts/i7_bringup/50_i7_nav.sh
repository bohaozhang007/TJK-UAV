#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/jkhk/jkhk_robot/release/planner/setup.bash
export ROS_PACKAGE_PATH=/home/jkhk/TJK-UAV/ros:/home/jkhk/jkhk_robot/release/planner/share:${ROS_PACKAGE_PATH:-}

exec roslaunch i7_nav i7_interactive.launch
