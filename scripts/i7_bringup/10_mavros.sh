#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

# PX4 serial link plus a MAVLink UDP copy for QGC at 192.168.9.63.
exec roslaunch mavros px4.launch \
  fcu_url:=/dev/ttyTHS0:921600 \
  gcs_url:=udp://0.0.0.0:14555@192.168.9.63:14550
