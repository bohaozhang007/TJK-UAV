#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/load_i7_config.sh"

MEDIAMTX_BIN="${I7_MEDIAMTX_BIN:-$(read_i7_bringup_config mediamtx_binary)}"
if [[ ! -x "${MEDIAMTX_BIN}" ]]; then
  echo "MediaMTX executable not found: ${MEDIAMTX_BIN}" >&2
  exit 1
fi

# Source and local relay endpoint come from the I7 YAML. The Robot controller
# applies the configured output resize before serving frames.
export MTX_HLS=no
export MTX_RTMP=no
export MTX_SRT=no
export MTX_WEBRTC=no
export MTX_RTSPADDRESS="${I7_MEDIAMTX_RTSP_ADDRESS:-$(read_i7_bringup_config mediamtx_rtsp_address)}"
export MTX_PATHS_K40T_SOURCE="${I7_CAMERA_SOURCE_URL:-$(read_i7_bringup_config camera_source_url)}"
export MTX_PATHS_K40T_SOURCEONDEMAND="${I7_CAMERA_SOURCE_ON_DEMAND:-$(read_i7_bringup_config camera_source_on_demand)}"

exec "${MEDIAMTX_BIN}"
