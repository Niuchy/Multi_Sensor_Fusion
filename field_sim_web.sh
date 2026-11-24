#!/usr/bin/env bash
# Independent control page; no builds, dependency installs or fusion restarts.
set -euo pipefail
FIELD_UI_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${PYTHON:-}" ]]; then
  FIELD_UI_PYTHON="$PYTHON"
elif [[ -x "$FIELD_UI_ROOT/.rk3588-env/bin/python" ]]; then
  FIELD_UI_PYTHON="$FIELD_UI_ROOT/.rk3588-env/bin/python"
elif [[ -x "$FIELD_UI_ROOT/.venv/bin/python" ]]; then
  FIELD_UI_PYTHON="$FIELD_UI_ROOT/.venv/bin/python"
elif [[ -x "$HOME/miniconda3/envs/sb_fusion/bin/python" ]]; then
  FIELD_UI_PYTHON="$HOME/miniconda3/envs/sb_fusion/bin/python"
else
  FIELD_UI_PYTHON="$(command -v python3)"
fi
exec "$FIELD_UI_PYTHON" "$FIELD_UI_ROOT/tools/field_sim_web.py" "$@"
