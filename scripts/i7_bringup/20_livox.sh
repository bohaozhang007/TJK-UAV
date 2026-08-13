#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/jkhk/livox_ws/devel/setup.bash
exec roslaunch livox_ros_driver2 msg_MID360.launch
