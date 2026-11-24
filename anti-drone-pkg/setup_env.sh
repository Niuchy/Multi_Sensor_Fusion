#!/bin/bash
# 反无人机数据生成系统 — 环境配置
# 用法:
#   source setup_env.sh              # 自动 (x86_64)
#   source setup_env.sh x86_64       # x86_64
#   source setup_env.sh aarch64      # ARM64

TARGET_ARCH="${1:-x86_64}"

if [ "$TARGET_ARCH" = "aarch64" ]; then
    PLATFORM="linux-arm64"
else
    PLATFORM="linux-x86"
fi

# data_generator 目录
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 项目根目录 (data_generator/ 的上级)
ROOT_DIR="$(cd "$PROJECT_DIR/.." && pwd)"

# 自动检测目录布局: 迁移包 vs 源码树
# 迁移包: bridge/ 在包内，thirdparty/ 也在包内
# 源码树: bridge/ 和 thirdparty/ 都在项目根 (data_generator/../)
if [ -d "$PROJECT_DIR/bridge" ] && [ -d "$PROJECT_DIR/thirdparty" ]; then
    # 迁移包模式: bridge/ 和 thirdparty/ 都在包根
    BASE_DIR="$PROJECT_DIR"
    BRIDGE_DIR="$PROJECT_DIR/bridge"
elif [ -d "$ROOT_DIR/bridge" ] && [ -d "$ROOT_DIR/thirdparty" ]; then
    # 源码树模式
    BASE_DIR="$ROOT_DIR"
    BRIDGE_DIR="$ROOT_DIR/bridge"
elif [ -d "$PROJECT_DIR/thirdparty/bridge" ]; then
    # 旧版迁移包 (bridge 在 thirdparty/bridge/)
    BASE_DIR="$PROJECT_DIR"
    BRIDGE_DIR="$PROJECT_DIR/thirdparty/bridge"
else
    BASE_DIR="$ROOT_DIR"
    BRIDGE_DIR="$ROOT_DIR/bridge"
fi

# 激活 conda (可选，不存在也不报错)
if [ -f "$HOME/miniconda3/bin/activate" ]; then
    source "$HOME/miniconda3/bin/activate" 2>/dev/null || true
elif [ -f "$HOME/anaconda3/bin/activate" ]; then
    source "$HOME/anaconda3/bin/activate" 2>/dev/null || true
elif command -v conda &> /dev/null; then
    eval "$(conda shell.bash hook)" 2>/dev/null || true
fi

# 激活 anti-drone 环境（如果存在）
conda activate anti-drone 2>/dev/null || true

# 第三方库路径（所有 .so 统一在平台 lib/ 下）
export LD_LIBRARY_PATH="$BASE_DIR/thirdparty/$PLATFORM/lib:$LD_LIBRARY_PATH"

# Bridge 库路径 (优先，确保 libad_proto.so 被找到)
export LD_LIBRARY_PATH="$BRIDGE_DIR:$LD_LIBRARY_PATH"

# Python 路径
export PYTHONPATH="$PROJECT_DIR/src:$PYTHONPATH"

echo "[setup] 平台: $TARGET_ARCH ($PLATFORM)"
echo "[setup] 项目根: $PROJECT_DIR"
echo "[setup] LD_LIBRARY_PATH:"
echo "  $BASE_DIR/thirdparty/$PLATFORM/lib"
echo "  $BRIDGE_DIR"
