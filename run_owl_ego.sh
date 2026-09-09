#!/usr/bin/env bash
# Default is a read-only audit. Explicit modes start passive bridge or HTTP server.
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
  bridge) exec roslaunch owl_nav owl_ego.launch config:="$CONFIG" "$@" ;;
  server) exec python3 -m robot.server --robot owl_ego --config "$CONFIG" "$@" ;;
  *) echo 'Usage: run_owl_ego.sh [check|bridge|server] [extra arguments]'; exit 2 ;;
esac
