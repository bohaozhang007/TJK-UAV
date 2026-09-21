#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/load_i7_config.sh"

STARTUP_GRACE_S="${I7_STARTUP_GRACE_S:-$(read_i7_bringup_config startup_grace_s)}"
SHUTDOWN_POLL_S="$(read_i7_bringup_config shutdown_poll_interval_s)"
INTERRUPT_SHUTDOWN_TIMEOUT_S="$(read_i7_bringup_config interrupt_shutdown_timeout_s)"
TERMINATE_SHUTDOWN_TIMEOUT_S="$(read_i7_bringup_config terminate_shutdown_timeout_s)"

COMPONENTS=(
  00_roscore.sh
  10_mavros.sh
  20_livox.sh
  30_fastlio.sh
  40_camera_relay.sh
)

PIDS=()
SHUTTING_DOWN=false
CHECK_PID=""
STACK=false
if [[ "${1:-}" == --stack ]]; then
  STACK=true
  exec 9>"/tmp/tjk-i7-stack-${UID}.lock"
  flock -n 9 || { echo 'Another i7 stack is already running.' >&2; exit 1; }
  python3 - 9>&- <<'PY'
import socket
with socket.socket() as sock:
    # Match ThreadingHTTPServer; closed connections may remain in TIME_WAIT.
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('127.0.0.1', 8765))
        sock.listen(1)
    except OSError as exc:
        raise SystemExit('Robot port 8765 unavailable: '+str(exc)
                         +'; check for a running Robot server before starting stack.')
PY
  (
    exec 9>&-
    source /opt/ros/noetic/setup.bash
    python3 - <<'PY'
import socket
import rosgraph
from urllib.parse import urlsplit, urlunsplit
from xmlrpc.client import ServerProxy
socket.setdefaulttimeout(1.)
master = rosgraph.Master('/i7_stack_check')
try:
    master.getPid()
except (OSError, rosgraph.MasterError):
    pass
else:
    names = {n for group in master.getSystemState() for _, nodes in group for n in nodes}
    conflicts = names & {'/i7_v22_sensors', '/i7_ego_v22_bridge', '/i7_ego_v22_robot'}
    active = []
    for name in sorted(conflicts):
        try:
            uri = master.lookupNode(name)
        except rosgraph.MasterError:
            continue
        address = urlsplit(uri)
        # Avoid multi-interface hostname resolution for local ROS nodes.
        if address.hostname in {socket.gethostname().lower(), socket.getfqdn().lower(), 'localhost'}:
            uri = urlunsplit((address.scheme, '127.0.0.1:'+str(address.port), address.path, '', ''))
        try:
            code, message, pid = ServerProxy(uri).getPid('/i7_stack_check')
            if code != 1:
                raise RuntimeError(message)
        except ConnectionRefusedError:
            print('Ignoring exited ROS node registration: '+name, flush=True)
            continue
        except Exception as exc:
            raise SystemExit('Cannot verify existing ROS node '+name+': '+str(exc))
        active.append(name)
    if active:
        raise SystemExit('Stop separate v22 services before starting stack: '+str(active))
PY
  )
fi

process_group_alive() {
  kill -0 -- "-$1" 2>/dev/null
}

check_started_components() {
  local index
  for ((index=0; index<${#PIDS[@]}; index++)); do
    if ! kill -0 "${PIDS[index]}" 2>/dev/null; then
      wait "${PIDS[index]}" 2>/dev/null || true
      echo "I7 component exited during startup: ${COMPONENTS[index]}" >&2
      exit 1
    fi
  done
}

signal_all() {
  local signal="$1"
  local index
  if [[ -n "$CHECK_PID" ]]; then
    kill "-${signal}" -- "-$CHECK_PID" 2>/dev/null || true
  fi
  for ((index=${#PIDS[@]} - 1; index >= 0; index--)); do
    kill "-${signal}" -- "-${PIDS[index]}" 2>/dev/null || true
  done
}

wait_for_shutdown() {
  local timeout_s="$1"
  local attempts
  attempts="$(python3 -c \
    'import math, sys; print(max(1, math.ceil(float(sys.argv[1]) / float(sys.argv[2]))))' \
    "${timeout_s}" "${SHUTDOWN_POLL_S}")"
  local attempt pid any_alive
  for ((attempt=0; attempt<attempts; attempt++)); do
    any_alive=false
    if [[ -n "$CHECK_PID" ]] && process_group_alive "$CHECK_PID"; then
      any_alive=true
    fi
    for pid in "${PIDS[@]}"; do
      if process_group_alive "${pid}"; then
        any_alive=true
        break
      fi
    done
    if [[ "${any_alive}" == false ]]; then
      return 0
    fi
    sleep "${SHUTDOWN_POLL_S}"
  done
  return 1
}

cleanup() {
  local exit_status=$?
  local pid
  if [[ "${SHUTTING_DOWN}" == true ]]; then
    return
  fi
  SHUTTING_DOWN=true
  trap - HUP INT TERM EXIT

  if ((${#PIDS[@]} > 0)); then
    echo "Stopping components and monitors started by this script (reused services stay running)..."
    signal_all INT
    if ! wait_for_shutdown "${INTERRUPT_SHUTDOWN_TIMEOUT_S}"; then
      signal_all TERM
      if ! wait_for_shutdown "${TERMINATE_SHUTDOWN_TIMEOUT_S}"; then
        signal_all KILL
      fi
    fi
    for pid in "${PIDS[@]}"; do
      wait "${pid}" 2>/dev/null || true
    done
    if [[ -n "$CHECK_PID" ]]; then wait "$CHECK_PID" 2>/dev/null || true; fi
  fi
  echo "I7 bringup stopped; reused services were left running."
  exit "${exit_status}"
}

trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM
trap cleanup EXIT

for component in "${COMPONENTS[@]}"; do
  component_path="${SCRIPT_DIR}/${component}"
  if [[ ! -x "${component_path}" ]]; then
    echo "I7 component is missing or not executable: ${component_path}" >&2
    exit 1
  fi
done

for component in "${COMPONENTS[@]}"; do
  component_path="${SCRIPT_DIR}/${component}"
  check_started_components
  echo "Starting ${component}..."
  setsid -- "${component_path}" 9>&- &
  pid=$!
  PIDS+=("${pid}")
  sleep "${STARTUP_GRACE_S}"
  check_started_components
done

if [[ "$STACK" == true ]]; then
  run_probe() {
    setsid -- timeout --signal=INT --kill-after=3s 45s "$@" 9>&- &
    CHECK_PID=$!
    while kill -0 "$CHECK_PID" 2>/dev/null; do
      check_started_components
      sleep .2
    done
    local status=0
    wait "$CHECK_PID" || status=$?
    CHECK_PID=""
    return "$status"
  }
  start_service() {
    local mode="$1"
    check_started_components
    echo "Starting ${mode}..."
    COMPONENTS+=("$mode")
    setsid -- bash app/run_i7.sh "$mode" 9>&- &
    PIDS+=("$!")
    sleep "$STARTUP_GRACE_S"
    check_started_components
  }

  start_service sensors
  ready=false
  for attempt in 1 2 3; do
    echo "Live deployment check ${attempt}/3..."
    if run_probe bash app/run_i7.sh check --live; then ready=true; fi
    if [[ "$ready" == true ]]; then break; fi
    sleep 1
  done
  if [[ "$ready" != true ]]; then
    echo 'Live checks failed; bridge/server were not started.' >&2
    exit 1
  fi
  start_service bridge
  run_probe bash -c 'source /opt/ros/noetic/setup.bash; python3 -c '\''import os, rospy; from app.robot.config_loader import load_robot_config; c=load_robot_config("i7", os.environ.get("I7_V22_CONFIG")); rospy.wait_for_service(c["topics"]["command"], timeout=30.)'\'''
  start_service server
  run_probe python3 -c '
import json, time, urllib.request
deadline = time.monotonic()+20.
while time.monotonic() < deadline:
    try:
        with urllib.request.urlopen("http://127.0.0.1:8765/v21/capabilities", timeout=1.) as response:
            result=json.load(response)
        if result.get("ok") and result.get("backend")=="i7":
            break
    except (OSError, ValueError):
        pass
    time.sleep(.2)
else:
    raise SystemExit("Robot server readiness timed out")
'
  check_started_components
  echo 'Live checks passed; services launched. Open console in another terminal. Keep this terminal open until landing and disarming.'
fi

if [[ "$STACK" != true ]]; then
  echo "I7 hardware launchers are running; verify readiness with check --live. Ctrl-C stops this script's components; reused services stay running."
fi
set +e
wait -n "${PIDS[@]}"
child_status=$?
set -e
echo "An I7 hardware component stopped (status=${child_status}); shutting down the stack." >&2
exit "${child_status}"
