#!/usr/bin/env bash
#
# One-command launcher kept at the sb_fusion workspace root.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8000}"
EXTERNAL_WS_URL="${EXTERNAL_WS_URL:-${SB_FUSION_EXTERNAL_WS_URL:-}}"
EXTERNAL_WS_SERVER="${EXTERNAL_WS_SERVER:-0}"
EXTERNAL_WS_HOST="${EXTERNAL_WS_HOST:-0.0.0.0}"
EXTERNAL_WS_PORT="${EXTERNAL_WS_PORT:-11409}"
EXTERNAL_WS_PATH="${EXTERNAL_WS_PATH:-/ws/main}"

if [[ -n "${PYTHON:-}" ]]; then
  PY="$PYTHON"
elif [[ -x "${ROOT_DIR}/.rk3588-env/bin/python" ]]; then
  PY="${ROOT_DIR}/.rk3588-env/bin/python"
else
  PY="python3"
fi

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "错误: SB Fusion 原生DDS整栈必须在 Linux 上运行。" >&2
  exit 2
fi

case "$(uname -m)" in
  aarch64|arm64|x86_64|amd64) ;;
  *)
    echo "错误: 不支持的架构: $(uname -m)" >&2
    exit 2
    ;;
esac

if ! "$PY" -c "import sys; raise SystemExit(sys.version_info < (3, 10))"; then
  echo "错误: 需要 Python 3.10 或更高版本。" >&2
  if [[ "$(uname -m)" == "aarch64" || "$(uname -m)" == "arm64" ]]; then
    echo "RK3588/Kylin 请先运行: ./setup_rk3588.sh" >&2
  fi
  exit 3
fi

if ! "$PY" -c "import fastapi,uvicorn,numpy,scipy,websockets,yaml" \
  >/dev/null 2>&1; then
  echo "[依赖] 安装融合运行依赖..."
  "$PY" -m pip install -r "${ROOT_DIR}/dds_swarm_fusion/requirements-dds.txt"
fi

BUILD_ARGS=()
for arg in "$@"; do
  if [[ "$arg" == "--equipment" || "$arg" == "--external-simulator" ]]; then
    BUILD_ARGS=(--equipment-only)
  fi
done
bash "${ROOT_DIR}/build_native.sh" "${BUILD_ARGS[@]}"

echo "============================================================"
echo "  启动 SB Fusion"
echo "  HOST=${HOST} PORT=${PORT}"
echo "  PYTHON=${PY}"
echo "  EXTERNAL_WS_URL=${EXTERNAL_WS_URL:-disabled}"
echo "  EXTERNAL_WS_SERVER=${EXTERNAL_WS_SERVER}"
echo "============================================================"

RUN_ARGS=(
  --host "$HOST"
  --port "$PORT"
)
if [[ -n "$EXTERNAL_WS_URL" ]]; then
    RUN_ARGS+=(--external-ws-url "$EXTERNAL_WS_URL")
fi
case "${EXTERNAL_WS_SERVER}" in
  1|[tT][rR][uU][eE]|[yY][eE][sS]|[oO][nN])
    if [[ -n "$EXTERNAL_WS_URL" ]]; then
      echo "错误: EXTERNAL_WS_SERVER 与 EXTERNAL_WS_URL 不能同时启用。" >&2
      exit 4
    fi
    RUN_ARGS+=(
      --external-ws-server
      --external-ws-host "$EXTERNAL_WS_HOST"
      --external-ws-port "$EXTERNAL_WS_PORT"
      --external-ws-path "$EXTERNAL_WS_PATH"
    )
    ;;
esac

RUN_ARGS+=("$@")

exec "$PY" "${ROOT_DIR}/run_sb_fusion.py" "${RUN_ARGS[@]}"
