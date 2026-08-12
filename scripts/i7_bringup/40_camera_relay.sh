#!/usr/bin/env bash
set -euo pipefail

MEDIAMTX_BIN=/home/jkhk/workspace/jetson-core/libs/mediamtx/arm64/mediamtx
if [[ ! -x "${MEDIAMTX_BIN}" ]]; then
  echo "MediaMTX executable not found: ${MEDIAMTX_BIN}" >&2
  exit 1
fi

# Source: K40T 1920x1080 RTSP. Consumers use local :8554/k40t; the Robot
# controller performs the agreed first-version resize to exactly 640x360.
export MTX_HLS=no
export MTX_RTMP=no
export MTX_SRT=no
export MTX_WEBRTC=no
export MTX_RTSPADDRESS=:8554
export MTX_PATHS_K40T_SOURCE=rtsp://192.168.144.64:558/live/single
export MTX_PATHS_K40T_SOURCEONDEMAND=yes

exec "${MEDIAMTX_BIN}"
