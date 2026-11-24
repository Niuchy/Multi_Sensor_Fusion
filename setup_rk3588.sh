#!/usr/bin/env bash
# Prepare an isolated Python/C++ environment for RK3588 Kylin and build bridges.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_DIR="${ROOT_DIR}/.rk3588-env"
CONDA_BIN="${CONDA:-}"

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "错误: RK3588 初始化脚本只能在 Linux 上运行。" >&2
  exit 2
fi

case "$(uname -m)" in
  aarch64|arm64) ;;
  *)
    echo "错误: 当前设备不是 ARM64/RK3588: $(uname -m)" >&2
    exit 2
    ;;
esac

if [[ -z "$CONDA_BIN" ]]; then
  for candidate in \
    "${HOME}/miniconda3/bin/conda" \
    "${HOME}/anaconda3/bin/conda" \
    conda; do
    if [[ "$candidate" == */* ]]; then
      [[ -x "$candidate" ]] || continue
    elif ! command -v "$candidate" >/dev/null 2>&1; then
      continue
    fi
    CONDA_BIN="$candidate"
    break
  done
fi

if [[ -z "$CONDA_BIN" ]] || ! command -v "$CONDA_BIN" >/dev/null 2>&1; then
  cat >&2 <<'EOF'
错误: 未找到 ARM64 Miniconda/Conda。
请先安装 ARM64 版 Miniconda，再重新运行 ./setup_rk3588.sh。
EOF
  exit 3
fi

ENV_PACKAGES=(
  "python=3.12"
  pip
  "numpy>=1.24,<3"
  "scipy>=1.10,<2"
  "pyyaml>=6,<7"
  "fastapi>=0.100,<1"
  "uvicorn>=0.23,<1"
  "websockets>=11,<17"
  gxx_linux-aarch64
)

echo "============================================================"
echo "  SB Fusion RK3588/Kylin setup"
echo "============================================================"
echo "  项目目录: ${ROOT_DIR}"
echo "  Conda: ${CONDA_BIN}"
echo "  隔离环境: ${ENV_DIR}"

if [[ ! -x "${ENV_DIR}/bin/python" ]]; then
  echo "[1/3] 创建 RK3588 隔离环境（conda-forge）..."
  "$CONDA_BIN" create \
    --yes \
    --prefix "$ENV_DIR" \
    --override-channels \
    --channel conda-forge \
    "${ENV_PACKAGES[@]}"
elif ! "${ENV_DIR}/bin/python" -c \
  "import fastapi,uvicorn,numpy,scipy,websockets,yaml" >/dev/null 2>&1; then
  echo "[1/3] 补齐 RK3588 隔离环境依赖..."
  "$CONDA_BIN" install \
    --yes \
    --prefix "$ENV_DIR" \
    --override-channels \
    --channel conda-forge \
    "${ENV_PACKAGES[@]}"
else
  echo "[1/3] 隔离环境已就绪。"
fi

CXX_BIN=""
for candidate in \
  "${ENV_DIR}/bin/aarch64-conda-linux-gnu-c++" \
  "${ENV_DIR}/bin/aarch64-conda-linux-gnu-g++"; do
  if [[ -x "$candidate" ]]; then
    CXX_BIN="$candidate"
    break
  fi
done
if [[ -z "$CXX_BIN" ]]; then
  echo "错误: Conda AArch64 C++ 编译器未安装成功。" >&2
  exit 4
fi

echo "[2/3] 在 RK3588 上原生构建 DDS 桥接库..."
CXX="$CXX_BIN" bash "${ROOT_DIR}/build_native.sh" --equipment-only

echo "[3/3] 验证 Python 依赖和动态库..."
LIB_DIR="${ROOT_DIR}/zy-anti-drone/thirdparty/linux-arm64/lib"
export LD_PRELOAD="${LIB_DIR}/libcrypto.so.3:${LIB_DIR}/libssl.so.3${LD_PRELOAD:+:${LD_PRELOAD}}"
export LD_LIBRARY_PATH="${LIB_DIR}:${ENV_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export OPENSSL_MODULES="${LIB_DIR}/ossl-modules"
"${ENV_DIR}/bin/python" -c \
  "import fastapi,uvicorn,numpy,scipy,websockets,yaml; print('Python依赖验证通过')"

if ldd "${ROOT_DIR}/libcompany_dds_bridge.so" 2>&1 | \
  grep -Eq "not found|version .* not found"; then
  echo "错误: 融合桥接库仍有未满足的动态链接依赖:" >&2
  ldd "${ROOT_DIR}/libcompany_dds_bridge.so" >&2
  exit 5
fi

echo
echo "RK3588 环境初始化完成。启动命令:"
echo "  cd ${ROOT_DIR}"
echo "  ./start_equipment_fusion.sh"
