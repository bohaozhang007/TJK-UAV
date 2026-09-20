#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/jkhk/jkhk_robot/release/slam/setup.bash
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${SCRIPT_DIR}/reuse_ros_component.py" --nodes /livox_lidar_publisher2 -- \
  roslaunch livox_ros_driver2 msg_MID360.launch
