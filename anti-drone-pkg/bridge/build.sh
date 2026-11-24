#!/bin/bash
# ============================================================================
# 反无人机协议桥接层编译脚本
#
# 将 ad_proto_bridge.cpp 编译为 libad_proto.so，
# 链接 thirdparty/ 下的 PubSubTools (FastDDS) 和 dev_proto.h 协议定义。
#
# 用法:
#   ./build.sh              # 自动检测当前架构 (x86_64)
#   ./build.sh x86_64        # 编译 x86_64 版本
#   ./build.sh aarch64       # 交叉编译 ARM64 版本
#   ./build.sh clean         # 清理编译产物
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
HOST_OS="$(uname -s)"

# 当前交付包只包含 Linux x86-64 预编译依赖。macOS 的 arm64 不是
# Linux aarch64，不能据此选择 linux-arm64 交叉编译目标。
if [ "${1:-}" != "clean" ] && [ "$HOST_OS" != "Linux" ]; then
    echo "[错误] 当前主机为 ${HOST_OS}/$(uname -m)，本构建脚本仅支持 Linux。"
    echo "       当前交付包没有 macOS dylib，也没有 PubSubTools/libcore 实现源码。"
    echo "       不能通过安装 aarch64-linux-gnu-g++ 在 macOS 上生成可加载的库。"
    exit 2
fi

# --- 架构选择 -----------------------------------------------------------
TARGET_ARCH="${1:-$(uname -m)}"
# 统一架构名称
case "$TARGET_ARCH" in
    x86_64|amd64)   ARCH="x86_64";  PLATFORM="linux-x86"  ;;
    aarch64|arm64)  ARCH="aarch64"; PLATFORM="linux-arm64" ;;
    clean)
        echo "[clean] 清理编译产物..."
        rm -f "$SCRIPT_DIR/libad_proto.so" "$SCRIPT_DIR"/*.o
        echo "[clean] 完成"
        exit 0
        ;;
    *) echo "错误: 不支持的架构 '$TARGET_ARCH' (支持: x86_64, aarch64, clean)"; exit 1 ;;
esac

# --- 路径定义 -----------------------------------------------------------
THIRDPARTY_DIR="$PROJECT_ROOT/thirdparty"
INCLUDE_DIR="$PROJECT_ROOT/include"

# 平台相关目录
PLATFORM_DIR="$THIRDPARTY_DIR/$PLATFORM"

# 头文件搜索路径
INCLUDES=(
    "-I$INCLUDE_DIR/common_msgs"             # dev_proto.h
    "-I$THIRDPARTY_DIR/PubSub/include"       # TopicAPI.h, PubSubTools.h, proto.h, DiscoveryServer.h
    "-I$THIRDPARTY_DIR/common/include"       # pxsb_common.h, pxsb_conf.h 等
    "-I$PLATFORM_DIR/include"                # fastdds, fastcdr, foonathan_memory 等
)

# 库文件搜索路径
LIB_DIRS=(
    "-L$PLATFORM_DIR/lib"
)

# 输出
OUTPUT="$SCRIPT_DIR/libad_proto.so"
SOURCE="$SCRIPT_DIR/ad_proto_bridge.cpp"

# --- 交付完整性检查 -----------------------------------------------------
REQUIRED_FILES=(
    "$SOURCE"
    "$SCRIPT_DIR/ad_proto_receiver.cpp"
    "$SCRIPT_DIR/ad_proto_fusion_bridge.cpp"
    "$INCLUDE_DIR/common_msgs/dev_proto.h"
)
MISSING_FILES=()
for required in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$required" ]; then
        MISSING_FILES+=("$required")
    fi
done
if [ ! -d "$PLATFORM_DIR" ]; then
    MISSING_FILES+=("$PLATFORM_DIR")
fi
if [ ${#MISSING_FILES[@]} -gt 0 ]; then
    echo "[错误] 当前交付包不包含重新编译所需的完整文件:"
    for missing in "${MISSING_FILES[@]}"; do
        echo "  - $missing"
    done
    echo "请向公司获取完整 bridge 源码和目标平台的 PubSubTools/FastDDS 库。"
    exit 3
fi

# --- 编译器和标志 -------------------------------------------------------
if [ "$ARCH" = "aarch64" ] && [ "$(uname -m)" != "aarch64" ]; then
    # 交叉编译 ARM64
    CXX="${CROSS_CXX:-aarch64-linux-gnu-g++}"
    echo "[build] 交叉编译模式: $ARCH (CXX=$CXX)"
else
    CXX="${CXX:-g++}"
    echo "[build] 本地编译模式: $ARCH (CXX=$CXX)"
fi
if ! command -v "$CXX" >/dev/null 2>&1; then
    echo "[错误] 编译器不存在: $CXX"
    exit 4
fi

CXXFLAGS=(
    -std=c++17
    -fPIC
    -shared
    -O2
    -g
    -Wall
    -Wextra
    -Wno-unused-parameter
)

# 链接的第三方库（按依赖顺序）
LIBS=(
    -lPubSubTools        # DDS 发布订阅封装
    -lpthread
    -ldl
)

# RPATH: 运行时从 platform/lib/ 和 PubSub/lib/ 查找 .so 依赖
# $ORIGIN = bridge/ 所在目录
RPATH="\$ORIGIN/../thirdparty/$PLATFORM/lib"

# --- 前置检查 -----------------------------------------------------------
echo "============================================================"
echo "  反无人机协议桥接层 — 编译 libad_proto.so"
echo "============================================================"
echo "  目标架构   : $ARCH ($PLATFORM)"
echo "  编译器     : $($CXX --version | head -1)"
echo "  源文件     : $SOURCE"
echo "  输出       : $OUTPUT"
echo "============================================================"

# 检查必要的头文件
MISSING_HEADERS=()
for hdr in \
    "$INCLUDE_DIR/common_msgs/dev_proto.h" \
    "$THIRDPARTY_DIR/PubSub/include/TopicAPI.h" \
    "$THIRDPARTY_DIR/PubSub/include/PubSubTools.h" \
    "$THIRDPARTY_DIR/PubSub/include/proto.h" \
    "$THIRDPARTY_DIR/PubSub/include/DiscoveryServer.h" \
    "$THIRDPARTY_DIR/common/include/pxsb_common.h"; do
    if [ ! -f "$hdr" ]; then
        MISSING_HEADERS+=("$hdr")
    fi
done

if [ ${#MISSING_HEADERS[@]} -gt 0 ]; then
    echo "[错误] 缺少头文件:"
    for h in "${MISSING_HEADERS[@]}"; do
        echo "  - $h"
    done
    exit 1
fi

# 检查必要的库文件
for lib in \
    "$PLATFORM_DIR/lib/libPubSubTools.so" \
    "$PLATFORM_DIR/lib/libfastrtps.so"; do
    if [ ! -f "$lib" ]; then
        echo "[警告] 库文件不存在: $lib (链接可能失败)"
    fi
done

# 确保输出目录存在
mkdir -p "$(dirname "$OUTPUT")"

# --- 编译 ---------------------------------------------------------------
echo ""
echo "[编译] $SOURCE ..."

set -x
$CXX \
    "${CXXFLAGS[@]}" \
    "${INCLUDES[@]}" \
    "${LIB_DIRS[@]}" \
    -Wl,-rpath,"$RPATH" \
    -o "$OUTPUT" \
    "$SOURCE" \
    "${LIBS[@]}"
set +x

# --- 验证 ---------------------------------------------------------------
echo ""
echo "============================================================"
echo "  编译完成!"
echo "============================================================"

if [ -f "$OUTPUT" ]; then
    echo "  文件       : $OUTPUT"
    echo "  大小       : $(ls -lh "$OUTPUT" | awk '{print $5}')"
    echo "  类型       : $(file "$OUTPUT" | cut -d: -f2-)"
    echo ""
    echo "  导出符号:"
    nm -D "$OUTPUT" 2>/dev/null | grep -E " T ad_proto_" | while read -r line; do
        printf "    %s\n" "$line"
    done || true
    echo ""
    echo "  依赖库:"
    readelf -d "$OUTPUT" 2>/dev/null | grep NEEDED | while read -r line; do
        printf "    %s\n" "$line"
    done || true
    echo ""
    echo "  RPATH:"
    readelf -d "$OUTPUT" 2>/dev/null | grep -E "RPATH|RUNPATH" | while read -r line; do
        printf "    %s\n" "$line"
    done || true
else
    echo "[错误] 编译失败，未生成 .so 文件"
    exit 1
fi

# ── 编译接收端 libad_proto_receiver.so ──
echo ""
echo "============================================================"
echo "  编译 libad_proto_receiver.so (接收端)"
echo "============================================================"

RX_SOURCE="$SCRIPT_DIR/ad_proto_receiver.cpp"
RX_OUTPUT="$SCRIPT_DIR/libad_proto_receiver.so"

$CXX \
    "${CXXFLAGS[@]}" \
    "${INCLUDES[@]}" \
    "-I$SCRIPT_DIR" \
    "${LIB_DIRS[@]}" \
    -Wl,-rpath,"$RPATH" \
    -o "$RX_OUTPUT" \
    "$RX_SOURCE" \
    "${LIBS[@]}"

if [ -f "$RX_OUTPUT" ]; then
    echo "  文件 : $RX_OUTPUT"
    echo "  大小 : $(ls -lh "$RX_OUTPUT" | awk '{print $5}')"
    echo "  导出符号:"
    nm -D "$RX_OUTPUT" 2>/dev/null | grep -E " T ad_proto_receive_" | while read -r line; do
        printf "    %s\n" "$line"
    done || true
else
    echo "[错误] 接收端编译失败"
    exit 1
fi

# ── 编译融合桥接 libad_proto_fusion.so ──
echo ""
echo "============================================================"
echo "  编译 libad_proto_fusion.so (融合层)"
echo "============================================================"

FUSION_SOURCE="$SCRIPT_DIR/ad_proto_fusion_bridge.cpp"
FUSION_OUTPUT="$SCRIPT_DIR/libad_proto_fusion.so"

$CXX \
    "${CXXFLAGS[@]}" \
    "${INCLUDES[@]}" \
    "${LIB_DIRS[@]}" \
    -Wl,-rpath,"$RPATH" \
    -o "$FUSION_OUTPUT" \
    "$FUSION_SOURCE" \
    "${LIBS[@]}"

if [ -f "$FUSION_OUTPUT" ]; then
    echo "  文件 : $FUSION_OUTPUT"
    echo "  大小 : $(ls -lh "$FUSION_OUTPUT" | awk '{print $5}')"
    echo "  导出符号:"
    nm -D "$FUSION_OUTPUT" 2>/dev/null | grep -E " T ad_proto_fusion_" | while read -r line; do
        printf "    %s\n" "$line"
    done || true
else
    echo "[错误] 融合桥接编译失败"
    exit 1
fi

echo ""
echo "用法提示:"
echo "  export LD_LIBRARY_PATH=$PLATFORM_DIR/lib:\$LD_LIBRARY_PATH"
echo "  export LD_LIBRARY_PATH=$SCRIPT_DIR:\$LD_LIBRARY_PATH"
