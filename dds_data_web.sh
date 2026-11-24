#!/usr/bin/env bash
# Independent web UI; no builds, dependency installs, or fusion process control.
set -euo pipefail
DDS_UI_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${PYTHON:-}" ]]; then
  DDS_UI_PYTHON="$PYTHON"
elif [[ -x "$DDS_UI_ROOT/.rk3588-env/bin/python" ]]; then
  DDS_UI_PYTHON="$DDS_UI_ROOT/.rk3588-env/bin/python"
elif [[ -x "$DDS_UI_ROOT/.venv/bin/python" ]]; then
  DDS_UI_PYTHON="$DDS_UI_ROOT/.venv/bin/python"
elif [[ -x "$HOME/miniconda3/envs/sb_fusion/bin/python" ]]; then
  DDS_UI_PYTHON="$HOME/miniconda3/envs/sb_fusion/bin/python"
else
  DDS_UI_PYTHON="$(command -v python3)"
fi
exec "$DDS_UI_PYTHON" "$DDS_UI_ROOT/tools/dds_data_web.py" "$@"
