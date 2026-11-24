#!/usr/bin/env bash
# Backward-compatible entry point retained for existing DGX deployments.
set -Eeuo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${ROOT_DIR}/build_native.sh" "$@"
