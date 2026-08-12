#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/jkhk/planner/devel/setup.bash

cd /home/jkhk/TJK-UAV
exec python3 -m src.robot.server \
  --robot i7 \
  --host 0.0.0.0 \
  --port 8765 \
  --vdir /home/jkhk/TJK-UAV/captures
