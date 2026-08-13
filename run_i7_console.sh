#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/noetic/setup.bash"
PLANNER_SETUP="/home/jkhk/planner/devel/setup.bash"

for setup_file in "${ROS_SETUP}" "${PLANNER_SETUP}"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "Required setup file not found: ${setup_file}" >&2
    exit 1
  fi
done

source_setup() {
  local setup_file="$1"
  shift
  source "${setup_file}"
}

# Source inside a zero-argument function scope so console CLI arguments such as
# --help are not forwarded into catkin's setup scripts.
source_setup "${ROS_SETUP}"
source_setup "${PLANNER_SETUP}"
unset -f source_setup

export ROS_PACKAGE_PATH="${SCRIPT_DIR}/ros:/home/jkhk/planner/release:${ROS_PACKAGE_PATH:-}"
export PYTHONPATH="${SCRIPT_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"

I7_SERVER_HOST="${I7_SERVER_HOST:-0.0.0.0}"
I7_SERVER_PORT="${I7_SERVER_PORT:-8765}"
I7_CAPTURE_DIR="${I7_CAPTURE_DIR:-${SCRIPT_DIR}/captures}"

cd "${SCRIPT_DIR}"
exec python3 -m robot.server \
  --robot i7 \
  --host "${I7_SERVER_HOST}" \
  --port "${I7_SERVER_PORT}" \
  --vdir "${I7_CAPTURE_DIR}" \
  "$@"
