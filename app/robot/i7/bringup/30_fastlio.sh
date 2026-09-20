#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/jkhk/jkhk_robot/release/slam/setup.bash

TRAJ_LOG_DIR="/home/jkhk/jkhk_robot/data/trajectories"
mkdir -p -- "${TRAJ_LOG_DIR}"
if [[ ! -w "${TRAJ_LOG_DIR}" ]]; then
  echo "Faster-LIO trajectory directory is not writable: ${TRAJ_LOG_DIR}" >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "${SCRIPT_DIR}/reuse_ros_component.py" --nodes /laserMapping,/lio_to_mavros -- \
  roslaunch faster_lio mapping_mid360.launch rviz:=false traj_log_dir:="${TRAJ_LOG_DIR}"
