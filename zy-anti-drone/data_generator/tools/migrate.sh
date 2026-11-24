#!/bin/bash
# 反无人机数据生成系统 — 迁移打包脚本
# 用法: bash tools/migrate.sh [目标路径]
#
# 打包后的目录结构与源码树一致，确保 RPATH 正确:
#   bridge/              ← .so 文件 (RPATH: $ORIGIN/../thirdparty/linux-x86/lib)
#   thirdparty/linux-x86/lib/  ← 所有依赖 .so
#   tools/dds_receiver   ← (RPATH: $ORIGIN/../../thirdparty/linux-x86/lib)

set -e
TARGET="${1:-/tmp/anti-drone-migrate}"
PROJECT="$(cd "$(dirname "$0")/.." && pwd)"
ROOT_DIR="$(cd "$PROJECT/.." && pwd)"

echo "========================================"
echo "  反无人机数据生成系统 — 项目迁移打包"
echo "========================================"
echo ""

# 1. 清理临时文件
echo "[1/5] 清理临时文件..."
find "$PROJECT" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find "$PROJECT" -type f -name "*.pyc" -delete 2>/dev/null || true
find "$PROJECT" -type f -name ".DS_Store" -delete 2>/dev/null || true

# 2. 确认 bridge 和 tools 已编译
echo "[2/5] 检查编译产物..."
MISSING=""
for f in "$ROOT_DIR/bridge/libad_proto.so" \
         "$ROOT_DIR/bridge/libad_proto_receiver.so" \
         "$ROOT_DIR/bridge/libad_proto_fusion.so" \
         "$PROJECT/tools/dds_receiver"; do
    if [ ! -f "$f" ]; then
        MISSING="$MISSING  $f\n"
    fi
done
if [ -n "$MISSING" ]; then
    echo "  缺少以下编译产物，请先编译:"
    echo -e "$MISSING"
    echo "  cd bridge && bash build.sh"
    echo "  cd data_generator/tools && bash build.sh"
    exit 1
fi
echo "  所有编译产物已就绪"

# 3. 创建迁移包目录
echo "[3/5] 创建迁移包..."
rm -rf "$TARGET"
mkdir -p "$TARGET"

# 4. 复制文件
echo "[4/5] 复制项目文件..."

# ── 核心代码 ──
cp -r "$PROJECT/src"         "$TARGET/"
cp -r "$PROJECT/config"     "$TARGET/"
cp -r "$PROJECT/tools"      "$TARGET/"
cp "$PROJECT/setup_env.sh"  "$TARGET/"
chmod +x "$TARGET/setup_env.sh"

# ── 第三方库目录 (与源码树布局一致) ──
mkdir -p "$TARGET/thirdparty/linux-x86/lib"
mkdir -p "$TARGET/thirdparty/linux-x86/include"
mkdir -p "$TARGET/thirdparty/PubSub/include"
mkdir -p "$TARGET/thirdparty/common/include"

# Bridge: 复制到顶层 bridge/ (保持 RPATH 正确: $ORIGIN/../thirdparty/linux-x86/lib)
mkdir -p "$TARGET/bridge"
cp "$ROOT_DIR/bridge/"*.so   "$TARGET/bridge/" 2>/dev/null || true
cp "$ROOT_DIR/bridge/"*.h    "$TARGET/bridge/" 2>/dev/null || true
cp "$ROOT_DIR/bridge/ad_proto_receiver.h" "$TARGET/bridge/" 2>/dev/null || true

# 编译脚本 (目标机器上可选择重编译)
cp "$ROOT_DIR/bridge/build.sh" "$TARGET/bridge/" 2>/dev/null || true

# PubSub 头文件
cp -r "$ROOT_DIR/thirdparty/PubSub/include/"* "$TARGET/thirdparty/PubSub/include/"

# Common 头文件
cp -r "$ROOT_DIR/thirdparty/common/include/"* "$TARGET/thirdparty/common/include/"

# 预编译 .so (含 libPubSubTools, libcore, libfastrtps 等所有依赖)
cp "$ROOT_DIR/thirdparty/linux-x86/lib/"*.so* "$TARGET/thirdparty/linux-x86/lib/" 2>/dev/null || true

# FastDDS 头文件 (目标机器上重新编译需要)
cp -r "$ROOT_DIR/thirdparty/linux-x86/include/"* "$TARGET/thirdparty/linux-x86/include/"

# ── 清理不需要的文件 ──
# 保留 .cpp 源码 (目标机器重编译 dds_receiver 需要)
rm -f "$TARGET/tools/build.sh" "$TARGET/tools/migrate.sh" 2>/dev/null || true

# 4. 生成目标机器安装脚本
echo "[5/5] 生成目标机器安装脚本..."
cat > "$TARGET/install.sh" << 'INSTALL_EOF'
#!/bin/bash
# 目标机器安装脚本
# 用法: bash install.sh [--with-conda]

set -e
WITH_CONDA=false
if [ "${1:-}" = "--with-conda" ]; then
    WITH_CONDA=true
fi

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
echo "============================================"
echo "  反无人机数据生成系统 — 安装"
echo "============================================"
echo "  安装目录: $PROJECT_DIR"
echo ""

# ── 检查系统依赖 ──
echo "[1/4] 检查系统依赖..."
MISSING_LIBS=""
for lib in libc.so.6 libstdc++.so.6 libpthread.so.0 libdl.so.2; do
    if ! ldconfig -p 2>/dev/null | grep -q "$lib" && ! find /usr/lib /lib -name "$lib" 2>/dev/null | grep -q .; then
        MISSING_LIBS="$MISSING_LIBS  $lib\n"
    fi
done
if [ -n "$MISSING_LIBS" ]; then
    echo "  警告: 缺少以下系统库，DDS 可能无法运行:"
    echo -e "$MISSING_LIBS"
fi
echo "  系统依赖检查完成"

# ── 验证 .so 文件 ──
echo "[2/4] 验证运行时库..."
ERRORS=0
for so in "$PROJECT_DIR/bridge/libad_proto.so" \
          "$PROJECT_DIR/thirdparty/linux-x86/lib/libPubSubTools.so" \
          "$PROJECT_DIR/thirdparty/linux-x86/lib/libfastrtps.so.2.14"; do
    if [ -f "$so" ]; then
        echo "  ✓ $(basename $so)"
    else
        echo "  ✗ 缺失: $so"
        ERRORS=$((ERRORS + 1))
    fi
done
if [ $ERRORS -gt 0 ]; then
    echo ""
    echo "  错误: 缺少必要的 .so 文件，迁移包可能不完整"
    exit 1
fi

# 验证 dds_receiver 可以加载
export LD_LIBRARY_PATH="$PROJECT_DIR/bridge:$PROJECT_DIR/thirdparty/linux-x86/lib:$LD_LIBRARY_PATH"
if ldd "$PROJECT_DIR/tools/dds_receiver" 2>&1 | grep -q "not found"; then
    echo "  警告: dds_receiver 有未解析的依赖:"
    ldd "$PROJECT_DIR/tools/dds_receiver" 2>&1 | grep "not found" | while read line; do
        echo "    $line"
    done
    echo "  可能需要在目标机器上重新编译:"
    echo "    cd $PROJECT_DIR && bash tools/build_dds_receiver.sh"
else
    echo "  ✓ dds_receiver 依赖完整"
fi

# ── Python 环境 (可选) ──
echo "[3/4] 配置 Python 环境..."
if $WITH_CONDA; then
    echo "  安装 miniconda..."
    if ! command -v conda &> /dev/null; then
        if [ ! -f "$HOME/miniconda3/bin/conda" ]; then
            curl -sL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o /tmp/miniconda.sh
            bash /tmp/miniconda.sh -b -p "$HOME/miniconda3"
            "$HOME/miniconda3/bin/conda" init bash
        fi
        source "$HOME/miniconda3/bin/activate"
    fi
    conda create -n anti-drone python=3.12 -y 2>/dev/null || true
    conda activate anti-drone 2>/dev/null || true
    pip install numpy PyQt6 pyyaml pyqtgraph scipy 2>/dev/null || \
    pip install numpy pyyaml --quiet
    echo "  conda 环境 'anti-drone' 已就绪"
else
    echo "  跳过 conda 安装 (使用 --with-conda 启用)"
    echo "  手动安装 Python 依赖:"
    echo "    pip install numpy PyQt6 pyyaml pyqtgraph scipy"
    echo "    或 (仅无 GUI 模式):"
    echo "    pip install numpy pyyaml"
fi

# ── 生成编译脚本 (目标机器上重新编译 dds_receiver) ──
echo "[4/4] 生成辅助脚本..."
cat > "$PROJECT_DIR/tools/build_dds_receiver.sh" << 'BUILD_EOF'
#!/bin/bash
# 在目标机器上重新编译 dds_receiver (当预编译二进制不兼容时)
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
THIRDPARTY="$HERE/thirdparty/linux-x86"
BRIDGE_DIR="$HERE/bridge"

echo "重新编译 dds_receiver (静态链接 libstdc++/libgcc + glibc 兼容层)..."
echo "  如果目标机器缺少 gcc，请先安装: apt install build-essential 或 yum install gcc-c++"

# 用 gcc 编译 glibc 兼容层 (C 链接，避免 C++ name mangling)
gcc -std=c11 -O2 -c -o "$HERE/tools/glibc_compat.o" "$HERE/tools/glibc_compat.c"

g++ -std=c++17 -O2 -g \
    -static-libstdc++ -static-libgcc \
    -I"$THIRDPARTY/include" \
    -I"$THIRDPARTY/include/fastcdr" \
    -I"$THIRDPARTY/include/fastdds" \
    -I"$THIRDPARTY/include/fastdds/rtps" \
    -I"$HERE/thirdparty/PubSub/include" \
    -I"$HERE/thirdparty/common/include" \
    -I"$BRIDGE_DIR" \
    -I"$HERE/include" \
    -L"$THIRDPARTY/lib" \
    -Wl,-rpath,'$ORIGIN/../../thirdparty/linux-x86/lib' \
    -o "$HERE/tools/dds_receiver" \
    "$HERE/tools/dds_receiver.cpp" \
    "$HERE/tools/glibc_compat.o" \
    -lPubSubTools -lfastrtps -lfastcdr -lcore -lfoonathan_memory-0.7.3 \
    -lsqlite3 -ltinyxml2 -lpthread -ldl

echo "编译完成: $HERE/tools/dds_receiver"
BUILD_EOF
chmod +x "$PROJECT_DIR/tools/build_dds_receiver.sh"

# 保存 dds_receiver 源码 (编译需要)
if [ ! -f "$PROJECT_DIR/tools/dds_receiver.cpp" ]; then
    echo "  注意: dds_receiver.cpp 未包含在包中，编译脚本需要源码"
fi

echo ""
echo "============================================"
echo "  安装完成!"
echo "============================================"
echo ""
echo "  使用前设置环境:"
echo "    source $PROJECT_DIR/setup_env.sh"
echo ""
echo "  运行方式:"
echo "    # 无 GUI 模式 (仅终端输出)"
echo "    python src/main.py --no-gui -o config/scenarios/single_target.yaml"
echo ""
echo "    # 带 GUI 模式"
echo "    python src/main.py -o config/scenarios/multi_target.yaml"
echo ""
echo "    # 接收端 (另一个终端)"
echo "    ./tools/dds_receiver"
echo ""
echo "  如果 dds_receiver 无法运行:"
echo "    bash tools/build_dds_receiver.sh  # 在目标机器上重编译"
echo ""
INSTALL_EOF
chmod +x "$TARGET/install.sh"

# 打包为 .tar.gz
TARBALL="${TARGET}.tar.gz"
echo ""
echo "[打包] 生成 ${TARBALL}..."
cd "$(dirname "$TARGET")"
tar czf "$TARBALL" "$(basename "$TARGET")"

# 验证包内容
echo ""
echo "============================================"
echo "  打包完成"
echo "============================================"
echo "  目录: $TARGET"
echo "  压缩包: $TARBALL ($(du -sh "$TARBALL" | cut -f1))"
echo ""
echo "  包内容概览:"
echo "    bridge/           ← 协议桥接 .so (含 RPATH)"
echo "    thirdparty/       ← 运行时依赖库"
echo "    src/              ← Python 源码"
echo "    config/           ← 场景配置"
echo "    tools/            ← dds_receiver, dds_log_reader"
echo "    setup_env.sh      ← 环境设置脚本"
echo "    install.sh        ← 安装脚本"
echo ""
echo "  迁移步骤 (在目标机器上):"
echo "    1. 拷贝: scp $TARBALL user@target:/path/"
echo "    2. 解压: tar xzf $(basename "$TARBALL")"
echo "    3. 安装: cd $(basename "$TARGET") && bash install.sh"
echo "    4. 设置: source setup_env.sh"
echo "    5. 运行: python src/main.py --no-gui -o config/scenarios/single_target.yaml"
echo "============================================"
