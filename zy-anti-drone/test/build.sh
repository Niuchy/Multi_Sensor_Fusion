#!/bin/bash

# ============================================================================
# 融合多目标消息测试 - 统一构建脚本
#
# 用法:
#   ./build.sh              # 默认 x86_64 原生编译
#   ./build.sh x86_64       # 显式指定 x86_64
#   ./build.sh aarch64      # ARM64 交叉编译
#   ./build.sh clean        # 清理所有构建产物
#
# 交叉编译前置条件:
#   sudo apt install g++-aarch64-linux-gnu
# ============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

TARGET_ARCH="${1:-x86_64}"

# ============================================================================
# 清理
# ============================================================================
if [ "$TARGET_ARCH" = "clean" ]; then
    echo "[clean] 清理构建目录..."
    rm -rf build build-arm64
    echo "[clean] 完成"
    exit 0
fi

# ============================================================================
# 参数校验
# ============================================================================
if [ "$TARGET_ARCH" != "x86_64" ] && [ "$TARGET_ARCH" != "aarch64" ]; then
    echo "错误: 不支持的架构 '$TARGET_ARCH'，请使用 x86_64 或 aarch64"
    echo "用法: $0 [x86_64|aarch64|clean]"
    exit 1
fi

# ============================================================================
# 构建配置
# ============================================================================
if [ "$TARGET_ARCH" = "aarch64" ]; then
    BUILD_DIR="build-arm64"
    CMAKE_ARGS="-DCMAKE_TOOLCHAIN_FILE=toolchain-arm64.cmake -DTARGET_ARCH=aarch64"

    # 检查交叉编译器
    if ! command -v aarch64-linux-gnu-g++ &>/dev/null; then
        echo "=============================================="
        echo "  错误: 未找到 ARM64 交叉编译器"
        echo "=============================================="
        echo ""
        echo "  请安装交叉编译工具链:"
        echo "    sudo apt install g++-aarch64-linux-gnu"
        echo ""
        echo "  或从源码安装:"
        echo "    https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads"
        echo ""
        exit 1
    fi

    CROSS_COMPILER=$(which aarch64-linux-gnu-g++)
    echo "[build] 目标架构: ARM64 (aarch64)"
    echo "[build] 交叉编译器: ${CROSS_COMPILER}"
else
    BUILD_DIR="build"
    CMAKE_ARGS="-DTARGET_ARCH=x86_64"
    echo "[build] 目标架构: x86_64 (native)"
fi

echo "[build] 构建目录: ${BUILD_DIR}"

# ============================================================================
# CMake 配置
# ============================================================================
echo "[cmake] 配置..."
cmake -B "${BUILD_DIR}" ${CMAKE_ARGS} -S .

# ============================================================================
# 编译
# ============================================================================
echo "[make] 编译..."
cmake --build "${BUILD_DIR}" -j$(nproc 2>/dev/null || echo 4)

# ============================================================================
# 验证产物
# ============================================================================
echo ""
echo "=============================================="
echo "  构建完成!"
echo "=============================================="
echo ""

for bin in publisher subscriber; do
    BIN_PATH="${BUILD_DIR}/${bin}"
    if [ -f "$BIN_PATH" ]; then
        echo "[ok] ${BIN_PATH}"
        echo "     $(file "${BIN_PATH}" | cut -d: -f2-)"
    else
        echo "[!!] ${BIN_PATH} 不存在!"
        exit 1
    fi
done

echo ""
echo "运行方法:"
echo "  source setup.sh ${TARGET_ARCH}"
echo "  ${BUILD_DIR}/publisher &"
echo "  ${BUILD_DIR}/subscriber"
