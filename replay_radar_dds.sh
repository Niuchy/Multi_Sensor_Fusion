#!/usr/bin/env bash
# Same isolated Python and ARM64/OpenSSL bootstrap as the live service.
# A log is required. Without --publish, this never sends DDS data.
set -Eeuo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${PYTHON:-}" ]]; then
  PY="$PYTHON"
elif [[ -x "${ROOT_DIR}/.rk3588-env/bin/python" ]]; then
  PY="${ROOT_DIR}/.rk3588-env/bin/python"
else
  PY=python3
fi
exec "$PY" "${ROOT_DIR}/run_sb_fusion.py" replay-radar "$@"
