#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /home/jkhk/jkhk_robot/release/slam/setup.bash
exec python3 -m app.robot.i7.bringup.launch_lio
