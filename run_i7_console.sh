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

SERVER_ARGS=()
if [[ -n "${I7_SERVER_HOST:-}" ]]; then
  SERVER_ARGS+=(--host "${I7_SERVER_HOST}")
fi
if [[ -n "${I7_SERVER_PORT:-}" ]]; then
  SERVER_ARGS+=(--port "${I7_SERVER_PORT}")
fi
if [[ -n "${I7_CAPTURE_DIR:-}" ]]; then
  SERVER_ARGS+=(--vdir "${I7_CAPTURE_DIR}")
fi

cd "${SCRIPT_DIR}"
exec python3 -m robot.server \
  --robot i7 \
  "${SERVER_ARGS[@]}" \
  "$@"
