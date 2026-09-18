#!/usr/bin/env bash
# Passive sensor consumer; no EGO workspace or control handover needed.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
exec /usr/bin/python3 "$ROOT/scripts/owl_ego/sensor_heartbeat.py" "$@"
