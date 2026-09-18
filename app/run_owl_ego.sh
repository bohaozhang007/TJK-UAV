#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${OWL_EGO_V22_WS:-/home/visbot/owl_ego_v22_ws}"
CONFIG="${OWL_EGO_V22_CONFIG:-$ROOT/app/robot/config/owl_ego.yaml}"
MODE="${1:-check}"
if [[ $# -gt 0 ]]; then shift; fi
CONFIG="$(realpath -- "$CONFIG")"
source /opt/ros/noetic/setup.bash
[[ -f "$WS/devel/setup.bash" ]] || { echo 'Run bash app/robot/owl_ego/build.sh first'; exit 1; }
source "$WS/devel/setup.bash"
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
cd "$ROOT"
case "$MODE" in
  check) exec python3 app/robot/owl_ego/scripts/preflight.py --config "$CONFIG" "$@" ;;
  prepare)
    python3 app/robot/owl_ego/scripts/prepare.py --config "$CONFIG"
    exec python3 app/robot/owl_ego/scripts/preflight.py --config "$CONFIG" --require-flight-ready "$@" ;;
  frames) exec python3 app/robot/owl_ego/scripts/check_frames.py --config "$CONFIG" "$@" ;;
  bridge)
    python3 app/robot/owl_ego/scripts/prepare.py --config "$CONFIG"
    exec roslaunch owl_nav_v22 owl_ego.launch config:="$CONFIG" "$@" ;;
  server) exec python3 -m app.robot.server --robot owl_ego --config "$CONFIG" "$@" ;;
  *) echo 'Usage: bash app/run_owl_ego.sh [check|prepare|frames|bridge|server]'; exit 2 ;;
esac
