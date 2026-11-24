#!/bin/bash

# ============================================================================
# 融合多目标消息测试 - 环境变量配置
#
# 用法:
#   source setup.sh              # 自动检测当前平台 (x86_64)
#   source setup.sh x86_64       # 强制 x86_64
#   source setup.sh aarch64      # 强制 ARM64 (用于交叉编译后运行)
# ============================================================================

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 自动检测或手动指定目标架构
TARGET_ARCH="${1:-x86_64}"

if [ "$TARGET_ARCH" = "aarch64" ]; then
    PLATFORM="linux-arm64"
else
    PLATFORM="linux-x86"
fi

THIRDPARTY_PLATFORM="${PROJECT_ROOT}/thirdparty/${PLATFORM}"

# 设置 LD_LIBRARY_PATH（所有 .so 均在平台目录的 lib/ 下）
export LD_LIBRARY_PATH="\
${THIRDPARTY_PLATFORM}/lib:\
${LD_LIBRARY_PATH}"

echo "[setup] 平台: ${TARGET_ARCH} (${PLATFORM})"
echo "[setup] LD_LIBRARY_PATH:"
echo "  ${THIRDPARTY_PLATFORM}/lib"
