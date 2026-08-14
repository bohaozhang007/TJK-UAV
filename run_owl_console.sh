#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WORKSPACE_SETUP="${SCRIPT_DIR}/../ros_ws/devel/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "Error: ROS Noetic setup not found: ${ROS_SETUP}" >&2
    exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
    echo "Error: ROS workspace setup not found: ${WORKSPACE_SETUP}" >&2
    echo "Build /home/visbot/ros_ws first, or update WORKSPACE_SETUP in this script." >&2
    exit 1
fi

source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"

export PYTHONPATH="${SCRIPT_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"

SERVER_ARGS=()
if [[ -n "${OWL_SERVER_HOST:-}" ]]; then
    SERVER_ARGS+=(--host "${OWL_SERVER_HOST}")
fi
if [[ -n "${OWL_SERVER_PORT:-}" ]]; then
    SERVER_ARGS+=(--port "${OWL_SERVER_PORT}")
fi

cd "${SCRIPT_DIR}"
exec python3 -m robot.server \
    --robot owl \
    "${SERVER_ARGS[@]}" \
    "$@"
