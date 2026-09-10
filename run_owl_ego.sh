#!/usr/bin/env bash
# check stays read-only; prepare/bridge perform guarded vendor control handover.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="${OWL_EGO_WS:-/home/visbot/owl_ego_ws}"
MODE="${1:-check}"
if [[ $# -gt 0 ]]; then shift; fi
CONFIG="${OWL_EGO_CONFIG:-$WS/owl_ego.yaml}"
source /opt/ros/noetic/setup.bash
[[ -f "$WS/devel/setup.bash" ]] || { echo 'Run scripts/owl_ego/build.sh first'; exit 1; }
source "$WS/devel/setup.bash"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
case "$MODE" in
  check) exec python3 "$ROOT/scripts/owl_ego/preflight.py" --config "$CONFIG" "$@" ;;
  prepare)
    python3 "$ROOT/scripts/owl_ego/prepare.py" --config "$CONFIG"
    exec python3 "$ROOT/scripts/owl_ego/preflight.py" --config "$CONFIG" --require-flight-ready "$@"
    ;;
  frames) exec python3 "$ROOT/scripts/owl_ego/check_frames.py" --config "$CONFIG" "$@" ;;
  record) exec python3 "$ROOT/scripts/owl_ego/record.py" --config "$CONFIG" "$@" ;;
  analyze) exec python3 "$ROOT/scripts/owl_ego/analyze_recording.py" "$@" ;;
  bridge)
    python3 "$ROOT/scripts/owl_ego/prepare.py" --config "$CONFIG"
    exec roslaunch owl_nav owl_ego.launch config:="$CONFIG" "$@"
    ;;
  server) exec python3 -m robot.server --robot owl_ego --config "$CONFIG" "$@" ;;
  *) echo 'Usage: run_owl_ego.sh [check|prepare|frames|record|analyze|bridge|server] [extra arguments]'; exit 2 ;;
esac
