#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/load_i7_config.sh"

MAVROS_FCU_URL="${I7_MAVROS_FCU_URL:-$(read_i7_bringup_config mavros_fcu_url)}"
MAVROS_GCS_URL="${I7_MAVROS_GCS_URL:-$(read_i7_bringup_config mavros_gcs_url)}"

# PX4 serial link plus the YAML-configured MAVLink UDP copy for QGC.
exec roslaunch mavros px4.launch \
  fcu_url:="${MAVROS_FCU_URL}" \
  gcs_url:="${MAVROS_GCS_URL}"
