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
