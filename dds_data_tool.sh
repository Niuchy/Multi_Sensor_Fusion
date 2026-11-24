#!/usr/bin/env bash
# Independent utility. Does not build, install packages, or launch fusion nodes.
set -euo pipefail
CAPTURE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${PYTHON:-}" ]]; then
  CAPTURE_PYTHON="$PYTHON"
elif [[ -x "$CAPTURE_ROOT/.rk3588-env/bin/python" ]]; then
  CAPTURE_PYTHON="$CAPTURE_ROOT/.rk3588-env/bin/python"
elif [[ -x "$CAPTURE_ROOT/.venv/bin/python" ]]; then
  CAPTURE_PYTHON="$CAPTURE_ROOT/.venv/bin/python"
elif [[ -x "$HOME/miniconda3/envs/sb_fusion/bin/python" ]]; then
  CAPTURE_PYTHON="$HOME/miniconda3/envs/sb_fusion/bin/python"
else
  CAPTURE_PYTHON="$(command -v python3)"
fi
exec "$CAPTURE_PYTHON" "$CAPTURE_ROOT/tools/dds_data_tool.py" "$@"
