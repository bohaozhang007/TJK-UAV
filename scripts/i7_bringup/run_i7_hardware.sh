#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STARTUP_GRACE_S="${I7_STARTUP_GRACE_S:-2}"

COMPONENTS=(
  00_roscore.sh
  10_mavros.sh
  20_livox.sh
  30_fastlio.sh
  40_camera_relay.sh
  50_i7_nav.sh
)

PIDS=()
SHUTTING_DOWN=false

process_group_alive() {
  kill -0 -- "-$1" 2>/dev/null
}

signal_all() {
  local signal="$1"
  local index
  for ((index=${#PIDS[@]} - 1; index >= 0; index--)); do
    kill "-${signal}" -- "-${PIDS[index]}" 2>/dev/null || true
  done
}

wait_for_shutdown() {
  local attempts="$1"
  local attempt pid any_alive
  for ((attempt=0; attempt<attempts; attempt++)); do
    any_alive=false
    for pid in "${PIDS[@]}"; do
      if process_group_alive "${pid}"; then
        any_alive=true
        break
      fi
    done
    if [[ "${any_alive}" == false ]]; then
      return 0
    fi
    sleep 0.1
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
    echo "Stopping all I7 hardware components..."
    signal_all INT
    if ! wait_for_shutdown 80; then
      signal_all TERM
      if ! wait_for_shutdown 30; then
        signal_all KILL
      fi
    fi
    for pid in "${PIDS[@]}"; do
      wait "${pid}" 2>/dev/null || true
    done
  fi
  echo "All I7 hardware components stopped."
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
  echo "Starting ${component}..."
  setsid -- "${component_path}" &
  pid=$!
  PIDS+=("${pid}")
  sleep "${STARTUP_GRACE_S}"
  if ! process_group_alive "${pid}"; then
    wait "${pid}" 2>/dev/null || true
    echo "I7 component exited during startup: ${component}" >&2
    exit 1
  fi
done

echo "All I7 hardware components are running. Press Ctrl-C to stop all six."
set +e
wait -n "${PIDS[@]}"
child_status=$?
set -e
echo "An I7 hardware component stopped (status=${child_status}); shutting down the stack." >&2
exit "${child_status}"
