#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${I7_EGO_V22_WS:-$HOME/i7_ego_v22_ws}"
CONFIG="${I7_V22_CONFIG:-$ROOT/app/robot/config/i7.yaml}"
MODE="${1:-check}"
if [[ $# -gt 0 ]]; then shift; fi
CONFIG="$(realpath -- "$CONFIG")"
export I7_V22_CONFIG="$CONFIG"
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
cd "$ROOT"
case "$MODE" in
  stack) exec python3 -m app.robot.i7.bringup.stack_supervisor "$@" ;;
  hardware) exec bash app/robot/i7/bringup/run_i7_hardware.sh "$@" ;;
  console) exec python3 app/robot/owl_ego/scripts/console.py --output "logs/i7_v22_console/$(date +%Y%m%d-%H%M%S)-$$" "$@" ;;
  check)
    source /opt/ros/noetic/setup.bash
    exec python3 -m app.robot.i7.preflight --config "$CONFIG" "$@" ;;
esac
source /opt/ros/noetic/setup.bash
if [[ "$MODE" == sensors ]]; then
  exec python3 -m app.robot.i7.sensors --config "$CONFIG" "$@"
fi
[[ -f "$WS/devel/setup.bash" ]] || { echo 'Run bash app/robot/i7/build.sh first'; exit 1; }
source "$WS/devel/setup.bash"
case "$MODE" in
  bridge) exec roslaunch owl_nav_v22 owl_ego.launch robot:=i7 bridge_name:=i7_ego_v22_bridge config:="$CONFIG" "$@" ;;
  server) exec python3 -m app.robot.server --robot i7 --config "$CONFIG" "$@" ;;
  *) echo 'Usage: bash app/run_i7.sh [stack|check|hardware|sensors|bridge|server|console]'; exit 2 ;;
esac
