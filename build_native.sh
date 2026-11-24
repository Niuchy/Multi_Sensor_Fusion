#!/usr/bin/env bash
#
# Build all native bridges needed by the ARM64/x86_64 SB Fusion stack.
# Native compilation keeps the generated bridge compatible with the target OS.
# Outputs created by this script are kept in the sb_fusion workspace root.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ZY_DIR="${ROOT_DIR}/zy-anti-drone"
DDS_DIR="${ROOT_DIR}/dds_swarm_fusion"
HOST_OS="$(uname -s)"
HOST_ARCH_RAW="$(uname -m)"
CXX_BIN="${CXX:-}"
BUILD_SIMULATOR=1
case "${1:-}" in
  "") ;;
  --equipment-only) BUILD_SIMULATOR=0 ;;
  *) echo "用法: $0 [--equipment-only]" >&2; exit 2 ;;
esac

normalize_arch() {
  case "$1" in
    aarch64|arm64) printf '%s' "aarch64" ;;
    x86_64|amd64) printf '%s' "x86_64" ;;
    *)
      echo "错误: 不支持的主机架构: $1" >&2
      exit 1
      ;;
  esac
}

if [[ "$HOST_OS" != "Linux" ]]; then
  echo "错误: 正式构建必须在 Linux 上执行，当前系统为 ${HOST_OS}。" >&2
  exit 2
fi

ARCH="$(normalize_arch "$HOST_ARCH_RAW")"
if [[ "$ARCH" == "aarch64" ]]; then
  PLATFORM="linux-arm64"
  EXPECTED_FILE_ARCH="ARM aarch64"
else
  PLATFORM="linux-x86"
  EXPECTED_FILE_ARCH="x86-64"
fi

if [[ -z "$CXX_BIN" ]]; then
  CXX_CANDIDATES=(
    "${ROOT_DIR}/.rk3588-env/bin/aarch64-conda-linux-gnu-c++"
    "${ROOT_DIR}/.rk3588-env/bin/aarch64-conda-linux-gnu-g++"
    "g++"
    "c++"
  )
  for candidate in "${CXX_CANDIDATES[@]}"; do
    if [[ "$candidate" == */* ]]; then
      [[ -x "$candidate" ]] || continue
    elif ! command -v "$candidate" >/dev/null 2>&1; then
      continue
    fi
    CXX_BIN="$candidate"
    break
  done
fi

PLATFORM_DIR="${ZY_DIR}/thirdparty/${PLATFORM}"
LIB_DIR="${PLATFORM_DIR}/lib"
SOURCE="${DDS_DIR}/company_bridge/company_dds_bridge.cpp"
OUTPUT="${ROOT_DIR}/libcompany_dds_bridge.so"

if [[ -z "$CXX_BIN" ]] || ! command -v "$CXX_BIN" >/dev/null 2>&1; then
  echo "错误: 找不到 C++ 编译器 '$CXX_BIN'。" >&2
  echo "RK3588/Kylin 请先运行: ./setup_rk3588.sh" >&2
  echo "其他 Linux 可安装: sudo apt-get install build-essential" >&2
  exit 3
fi

REQUIRED_FILES=(
  "$SOURCE"
  "${DDS_DIR}/company_bridge/company_wire.h"
  "${ZY_DIR}/thirdparty/PubSub/include/TopicAPI.h"
  "${ZY_DIR}/thirdparty/PubSub/include/proto.h"
  "${ZY_DIR}/include/common_msgs/fusion_proto.h"
  "${LIB_DIR}/libPubSubTools.so"
  "${LIB_DIR}/libcore.so"
  "${LIB_DIR}/libfastrtps.so"
  "${LIB_DIR}/libfastcdr.so"
  "${LIB_DIR}/libtinyxml2.so.9"
  "${LIB_DIR}/libsqlite3.so"
  "${LIB_DIR}/libfoonathan_memory-0.7.3.so"
)
if [[ "$BUILD_SIMULATOR" == 1 ]]; then
  REQUIRED_FILES+=("${ZY_DIR}/bridge/build.sh")
fi

if [[ "$ARCH" == "aarch64" ]]; then
  REQUIRED_FILES+=(
    "${LIB_DIR}/libssl.so.3"
    "${LIB_DIR}/libcrypto.so.3"
  )
fi

for required in "${REQUIRED_FILES[@]}"; do
  if [[ ! -e "$required" ]]; then
    echo "错误: 缺少构建文件: $required" >&2
    exit 4
  fi
done

if command -v file >/dev/null 2>&1; then
  for library in \
    "${LIB_DIR}/libPubSubTools.so" \
    "${LIB_DIR}/libcore.so" \
    "${LIB_DIR}/libfastrtps.so" \
    "${LIB_DIR}/libfastcdr.so"; do
    DESCRIPTION="$(file -L "$library")"
    if [[ "$DESCRIPTION" != *"$EXPECTED_FILE_ARCH"* ]]; then
      echo "错误: 动态库架构不匹配: $DESCRIPTION" >&2
      exit 5
    fi
  done
fi

echo "============================================================"
echo "  SB Fusion native bridge build"
echo "============================================================"
echo "  主机: ${HOST_OS}/${HOST_ARCH_RAW}"
echo "  平台: ${PLATFORM}"
echo "  编译器: ${CXX_BIN}"

echo
if [[ "$BUILD_SIMULATOR" == 1 ]]; then
  echo "[1/2] 构建公司仿真/DDS桥接库"
  (
    cd "${ZY_DIR}/bridge"
    CXX="$CXX_BIN" ./build.sh
  )
else
  echo "[1/2] 实装模式：跳过不使用的仿真桥接库"
fi

INCLUDES=(
  "-I${ZY_DIR}/thirdparty/PubSub/include"
  "-I${ZY_DIR}/thirdparty/common/include"
  "-I${ZY_DIR}/include/common_msgs"
  "-I${ZY_DIR}/bridge"
  "-I${PLATFORM_DIR}/include"
)

# The ARM delivery omits nlohmann/json.hpp. It is header-only, so the
# repository's x86 include copy is safe as a source-level fallback.
if [[ ! -f "${PLATFORM_DIR}/include/nlohmann/json.hpp" ]]; then
  HEADER_FALLBACK="${ZY_DIR}/thirdparty/linux-x86/include"
  if [[ -f "${HEADER_FALLBACK}/nlohmann/json.hpp" ]]; then
    INCLUDES+=("-I${HEADER_FALLBACK}")
  elif [[ ! -f "/usr/include/nlohmann/json.hpp" ]]; then
    echo "错误: 缺少 nlohmann/json.hpp。" >&2
    echo "请安装: sudo apt-get install nlohmann-json3-dev" >&2
    exit 4
  fi
fi

RPATH="\$ORIGIN/zy-anti-drone/thirdparty/${PLATFORM}/lib"
if [[ -d "${ROOT_DIR}/.rk3588-env/lib" ]]; then
  RPATH="${RPATH}:\$ORIGIN/.rk3588-env/lib"
fi
TEMP_OUTPUT="${ROOT_DIR}/.libcompany_dds_bridge.so.tmp.$$"

echo
echo "[2/2] 构建根目录融合侧 Python/DDS 桥接库"
if ! "$CXX_BIN" \
  -std=c++17 \
  -O2 \
  -g \
  -fPIC \
  -Wall \
  -Wextra \
  -Wno-unused-parameter \
  "${INCLUDES[@]}" \
  -shared \
  -Wl,--no-undefined \
  -Wl,-z,origin \
  -Wl,--disable-new-dtags \
  "-Wl,-rpath,${RPATH}" \
  "-Wl,-rpath-link,${LIB_DIR}" \
  "-L${LIB_DIR}" \
  -o "$TEMP_OUTPUT" \
  "$SOURCE" \
  -Wl,--no-as-needed \
  -lPubSubTools \
  -lcore \
  -lfastrtps \
  -lfastcdr \
  -Wl,--as-needed \
  -lpthread \
  -ldl \
  -lrt; then
  rm -f "$TEMP_OUTPUT"
  exit 1
fi

if command -v file >/dev/null 2>&1; then
  DESCRIPTION="$(file -L "$TEMP_OUTPUT")"
  if [[ "$DESCRIPTION" != *"$EXPECTED_FILE_ARCH"* ]]; then
    echo "错误: 输出架构不正确: $DESCRIPTION" >&2
    rm -f "$TEMP_OUTPUT"
    exit 5
  fi
fi

mv -f "$TEMP_OUTPUT" "$OUTPUT"

echo
echo "构建完成:"
if command -v file >/dev/null 2>&1; then
  if [[ "$BUILD_SIMULATOR" == 1 ]]; then
    file -L "${ZY_DIR}/bridge/libad_proto.so"
  fi
  file -L "$OUTPUT"
fi
