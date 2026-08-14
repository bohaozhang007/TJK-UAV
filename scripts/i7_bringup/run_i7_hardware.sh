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
  local timeout_s="$1"
  local attempts
  attempts="$(python3 -c \
    'import math, sys; print(max(1, math.ceil(float(sys.argv[1]) / float(sys.argv[2]))))' \
    "${timeout_s}" "${SHUTDOWN_POLL_S}")"
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
    echo "Stopping all I7 hardware components..."
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
