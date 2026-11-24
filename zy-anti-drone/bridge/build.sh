#!/usr/bin/env bash
#
# 构建 zy-anti-drone 的三个 Python/DDS 桥接动态库。
#
# Linux/aarch64 (including NVIDIA DGX Spark and RK3588):
#   ./build.sh
#   ./build.sh aarch64
#
# 其他用法:
#   ./build.sh x86_64       在 Linux x86_64 主机上构建
#   ./build.sh check        只检查当前架构对应的源码和交付文件
#   ./build.sh check arm64  只检查 ARM64 版本
#   ./build.sh clean        清理三个构建产物

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
THIRDPARTY_DIR="$PROJECT_ROOT/thirdparty"
COMMON_INCLUDE_DIR="$PROJECT_ROOT/include/common_msgs"

HOST_OS="$(uname -s)"
HOST_ARCH_RAW="$(uname -m)"

usage() {
    cat <<EOF
用法:
  $0 [aarch64|arm64|x86_64|amd64]
  $0 check [aarch64|arm64|x86_64|amd64]
  $0 clean
EOF
}

normalize_arch() {
    case "$1" in
        aarch64|arm64)
            printf '%s' "aarch64"
            ;;
        x86_64|amd64)
            printf '%s' "x86_64"
            ;;
        *)
            printf '[错误] 不支持的架构: %s\n' "$1" >&2
            usage >&2
            exit 1
            ;;
    esac
}

ACTION="build"
TARGET_INPUT="$HOST_ARCH_RAW"

case "${1:-}" in
    "")
        ;;
    check)
        ACTION="check"
        TARGET_INPUT="${2:-$HOST_ARCH_RAW}"
        ;;
    clean)
        ACTION="clean"
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        TARGET_INPUT="$1"
        ;;
esac

if [[ "$ACTION" == "clean" ]]; then
    rm -f \
        "$SCRIPT_DIR/libad_proto.so" \
        "$SCRIPT_DIR/libad_proto_receiver.so" \
        "$SCRIPT_DIR/libad_proto_fusion.so"
    find "$SCRIPT_DIR" -maxdepth 1 -type f -name '.*.tmp.*' -delete
    printf '[clean] 已清理 bridge 动态库和临时文件。\n'
    exit 0
fi

ARCH="$(normalize_arch "$TARGET_INPUT")"
case "$ARCH" in
    aarch64)
        PLATFORM="linux-arm64"
        EXPECTED_FILE_ARCH="ARM aarch64"
        ;;
    x86_64)
        PLATFORM="linux-x86"
        EXPECTED_FILE_ARCH="x86-64"
        ;;
esac

PLATFORM_DIR="$THIRDPARTY_DIR/$PLATFORM"
LIB_DIR="$PLATFORM_DIR/lib"

INCLUDES=(
    "-I$SCRIPT_DIR"
    "-I$COMMON_INCLUDE_DIR"
    "-I$THIRDPARTY_DIR/PubSub/include"
    "-I$THIRDPARTY_DIR/common/include"
    "-I$PLATFORM_DIR/include"
)

# ARM64 交付包漏了 nlohmann/json.hpp。它是架构无关的纯头文件，
# 因此允许复用仓库中 x86 目录里的同版本头文件；不会链接 x86 库。
if [[ ! -f "$PLATFORM_DIR/include/nlohmann/json.hpp" ]]; then
    HEADER_FALLBACK="$THIRDPARTY_DIR/linux-x86/include"
    if [[ -f "$HEADER_FALLBACK/nlohmann/json.hpp" ]]; then
        INCLUDES+=("-I$HEADER_FALLBACK")
    elif [[ ! -f "/usr/include/nlohmann/json.hpp" ]]; then
        cat >&2 <<EOF
[错误] 缺少头文件 nlohmann/json.hpp。
请保留 thirdparty/linux-x86/include/nlohmann，或在目标设备上安装:
  sudo apt-get install nlohmann-json3-dev
EOF
        exit 3
    fi
fi

SOURCES=(
    "$SCRIPT_DIR/ad_proto_bridge.cpp"
    "$SCRIPT_DIR/ad_proto_receiver.cpp"
    "$SCRIPT_DIR/ad_proto_fusion_bridge.cpp"
)

REQUIRED_HEADERS=(
    "$COMMON_INCLUDE_DIR/dev_proto.h"
    "$COMMON_INCLUDE_DIR/fusion_proto.h"
    "$SCRIPT_DIR/ad_proto_receiver.h"
    "$THIRDPARTY_DIR/PubSub/include/TopicAPI.h"
    "$THIRDPARTY_DIR/PubSub/include/PubSubTools.h"
    "$THIRDPARTY_DIR/PubSub/include/proto.h"
    "$THIRDPARTY_DIR/PubSub/include/MessageType.h"
    "$THIRDPARTY_DIR/common/include/pxsb_common.h"
)

REQUIRED_LIBS=(
    "$LIB_DIR/libPubSubTools.so"
    "$LIB_DIR/libcore.so"
    "$LIB_DIR/libfastrtps.so"
    "$LIB_DIR/libfastcdr.so"
    "$LIB_DIR/libtinyxml2.so.9"
    "$LIB_DIR/libsqlite3.so"
    "$LIB_DIR/libfoonathan_memory-0.7.3.so"
    "$LIB_DIR/libssl.so.3"
    "$LIB_DIR/libcrypto.so.3"
)

check_files() {
    local missing=0
    local path

    for path in "${SOURCES[@]}" "${REQUIRED_HEADERS[@]}" "${REQUIRED_LIBS[@]}"; do
        if [[ ! -e "$path" ]]; then
            printf '[错误] 缺少文件: %s\n' "$path" >&2
            missing=1
        fi
    done

    if [[ "$missing" -ne 0 ]]; then
        exit 3
    fi
}

check_library_architecture() {
    local lib
    local description

    if ! command -v file >/dev/null 2>&1; then
        printf '[警告] 未安装 file，跳过动态库架构检查。\n' >&2
        return
    fi

    for lib in "${REQUIRED_LIBS[@]}"; do
        description="$(file -L "$lib")"
        if [[ "$description" != *"$EXPECTED_FILE_ARCH"* ]]; then
            printf '[错误] 动态库架构与目标 %s 不匹配:\n  %s\n' \
                "$ARCH" "$description" >&2
            exit 5
        fi
    done
}

check_files
check_library_architecture

CXX="${CXX:-g++}"
if ! command -v "$CXX" >/dev/null 2>&1; then
    printf '[错误] 找不到 C++ 编译器: %s\n' "$CXX" >&2
    printf '请在 DGX 上安装: sudo apt-get install build-essential\n' >&2
    exit 4
fi

CXXFLAGS=(
    -std=c++17
    -fPIC
    -O2
    -g
    -Wall
    -Wextra
    -Wno-unused-parameter
)

syntax_check() {
    local source

    printf '[check] 编译器: %s\n' "$("$CXX" --version | head -n 1)"
    printf '[check] 目标平台: %s (%s)\n' "$ARCH" "$PLATFORM"
    for source in "${SOURCES[@]}"; do
        printf '[check] %s\n' "$(basename "$source")"
        "$CXX" \
            "${CXXFLAGS[@]}" \
            -fsyntax-only \
            "${INCLUDES[@]}" \
            "$source"
    done
    printf '[check] 三个桥接源码检查通过，%s 依赖文件完整。\n' "$ARCH"
}

if [[ "$ACTION" == "check" ]]; then
    syntax_check
    exit 0
fi

if [[ "$HOST_OS" != "Linux" ]]; then
    printf '[错误] 正式构建必须在 Linux 上执行，当前系统为 %s。\n' \
        "$HOST_OS" >&2
    exit 2
fi

HOST_ARCH="$(normalize_arch "$HOST_ARCH_RAW")"
if [[ "$HOST_ARCH" != "$ARCH" ]]; then
    cat >&2 <<EOF
[错误] 当前主机架构为 ${HOST_ARCH}，目标架构为 ${ARCH}。
本脚本用于原生构建，不混用其他架构的编译器和动态库。
EOF
    exit 2
fi

RPATH="\$ORIGIN/../thirdparty/$PLATFORM/lib"
PROJECT_RUNTIME_LIB="$PROJECT_ROOT/../.rk3588-env/lib"
if [[ -d "$PROJECT_RUNTIME_LIB" ]]; then
    RPATH="${RPATH}:\$ORIGIN/../../.rk3588-env/lib"
fi

LDFLAGS=(
    -shared
    -Wl,--no-undefined
    -Wl,-z,origin
    -Wl,--disable-new-dtags
    "-Wl,-rpath,$RPATH"
    "-Wl,-rpath-link,$LIB_DIR"
)

LIBS=(
    -Wl,--no-as-needed
    -lPubSubTools
    -lcore
    -lfastrtps
    -lfastcdr
    -Wl,--as-needed
    -lpthread
    -ldl
    -lrt
)

build_shared() {
    local source="$1"
    local output="$2"
    local symbol_regex="$3"
    local temp_output="$SCRIPT_DIR/.$(basename "$output").tmp.$$"
    local description

    printf '\n[编译] %s -> %s\n' "$(basename "$source")" "$(basename "$output")"

    if ! "$CXX" \
        "${CXXFLAGS[@]}" \
        "${INCLUDES[@]}" \
        "${LDFLAGS[@]}" \
        "-L$LIB_DIR" \
        -o "$temp_output" \
        "$source" \
        "${LIBS[@]}"; then
        rm -f "$temp_output"
        return 1
    fi

    description="$(file -L "$temp_output")"
    if [[ "$description" != *"$EXPECTED_FILE_ARCH"* ]]; then
        printf '[错误] 输出文件架构错误: %s\n' "$description" >&2
        rm -f "$temp_output"
        return 1
    fi

    mv -f "$temp_output" "$output"
    printf '[完成] %s\n' "$description"

    if command -v nm >/dev/null 2>&1; then
        printf '  导出符号:\n'
        nm -D --defined-only "$output" 2>/dev/null |
            awk -v pattern="$symbol_regex" \
                '$3 ~ pattern { print "    " $3 }'
    fi
}

printf '%s\n' '============================================================'
printf '%s\n' '  zy-anti-drone DDS 桥接库构建'
printf '%s\n' '============================================================'
printf '  主机       : %s/%s\n' "$HOST_OS" "$HOST_ARCH_RAW"
printf '  目标       : %s (%s)\n' "$ARCH" "$PLATFORM"
printf '  编译器     : %s\n' "$("$CXX" --version | head -n 1)"
printf '  依赖目录   : %s\n' "$LIB_DIR"

build_shared \
    "$SCRIPT_DIR/ad_proto_bridge.cpp" \
    "$SCRIPT_DIR/libad_proto.so" \
    '^ad_proto_'

build_shared \
    "$SCRIPT_DIR/ad_proto_receiver.cpp" \
    "$SCRIPT_DIR/libad_proto_receiver.so" \
    '^ad_proto_receive_'

build_shared \
    "$SCRIPT_DIR/ad_proto_fusion_bridge.cpp" \
    "$SCRIPT_DIR/libad_proto_fusion.so" \
    '^(ad_proto_fusion_|ad_proto_send_fusion_)'

printf '\n%s\n' '============================================================'
printf '  三个 %s DDS 桥接库构建完成\n' "$ARCH"
printf '%s\n' '============================================================'
printf '运行 Python 前建议设置:\n'
printf '  export LD_LIBRARY_PATH="%s:${LD_LIBRARY_PATH:-}"\n' "$LIB_DIR"
