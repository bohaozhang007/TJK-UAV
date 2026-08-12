#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
exec roslaunch fast_lio mapping_mid360.launch rviz:=false
