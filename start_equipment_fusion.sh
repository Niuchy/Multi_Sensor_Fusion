#!/usr/bin/env bash
# Start SB Fusion with live equipment DDS input; do not start the simulator.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export HOST="${HOST:-0.0.0.0}"
export PORT="${PORT:-8000}"
PUSH_REQUESTED="${EXTERNAL_WS_URL:-${SB_FUSION_EXTERNAL_WS_URL:-}}"
for arg in "$@"; do
  case "$arg" in
    --external-ws-url|--external-ws-url=*) PUSH_REQUESTED=1 ;;
  esac
done
if [[ -n "$PUSH_REQUESTED" ]]; then
  export EXTERNAL_WS_SERVER="${EXTERNAL_WS_SERVER:-0}"
else
  export EXTERNAL_WS_SERVER="${EXTERNAL_WS_SERVER:-1}"
fi
export EXTERNAL_WS_HOST="${EXTERNAL_WS_HOST:-0.0.0.0}"
export EXTERNAL_WS_PORT="${EXTERNAL_WS_PORT:-11409}"
export EXTERNAL_WS_PATH="${EXTERNAL_WS_PATH:-/ws/main}"
export SB_RADAR_INPUT_MODE="${SB_RADAR_INPUT_MODE:-device_tracks}"
export SB_AZIMUTH_CONVENTION="${SB_AZIMUTH_CONVENTION:-north_cw}"

# The shared RK3588 entry selects .rk3588-env, checks dependencies, rebuilds
# the equipment bridge, and forwards all arguments. No DGX-specific paths.

echo "============================================================"
echo "  SB Fusion 装备数据模式"
echo "  本地仿真器: disabled"
echo "  Web: http://<设备IP>:${PORT}/"
echo "  接入诊断: http://<设备IP>:${PORT}/debug.html"
echo "  WebSocket: ws://<设备IP>:${EXTERNAL_WS_PORT}${EXTERNAL_WS_PATH}"
echo "  雷达输入: ${SB_RADAR_INPUT_MODE}; 方位: ${SB_AZIMUTH_CONVENTION}"
echo "============================================================"

exec bash "${ROOT_DIR}/start_sb_fusion.sh" --equipment "$@"
