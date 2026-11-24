#!/bin/bash
# ==============================================================
# 编译 DDS Receiver 和 DDS Log Reader
# ==============================================================
set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROOT_DIR="$(cd "$PROJECT_ROOT/.." && pwd)"
THIRDPARTY="$ROOT_DIR/thirdparty/linux-x86"

# ── DDS Receiver ──
echo "=============================================="
echo "  Building DDS Receiver"
echo "=============================================="

OUTPUT="$PROJECT_ROOT/tools/dds_receiver"
SOURCE="$PROJECT_ROOT/tools/dds_receiver.cpp"
COMPAT="$PROJECT_ROOT/tools/glibc_compat.c"
COMPAT_O="$PROJECT_ROOT/tools/glibc_compat.o"

# 用 gcc 编译兼容层 (C 链接，避免 C++ name mangling)
gcc -std=c11 -O2 -c -o "$COMPAT_O" "$COMPAT"

g++ -std=c++17 -O2 -g \
    -static-libstdc++ -static-libgcc \
    -I"$THIRDPARTY/include" \
    -I"$THIRDPARTY/include/fastcdr" \
    -I"$THIRDPARTY/include/fastdds" \
    -I"$THIRDPARTY/include/fastdds/rtps" \
    -I"$ROOT_DIR/thirdparty/PubSub/include" \
    -I"$ROOT_DIR/thirdparty/common/include" \
    -I"$ROOT_DIR/bridge" \
    -I"$ROOT_DIR/include/common_msgs" \
    -L"$THIRDPARTY/lib" \
    -Wl,--disable-new-dtags \
    -Wl,-rpath,'$ORIGIN/../../thirdparty/linux-x86/lib' \
    -Wl,-rpath-link,"$THIRDPARTY/lib" \
    -o "$OUTPUT" "$SOURCE" "$COMPAT_O" \
    -lPubSubTools -lfastrtps -lfastcdr -lcore -lfoonathan_memory-0.7.3 \
    -lsqlite3 -ltinyxml2 -lpthread -ldl

echo "Build success: $OUTPUT"

# ── DDS Log Reader (零运行时依赖) ──
echo ""
echo "=============================================="
echo "  Building DDS Log Reader"
echo "=============================================="

LOGROUT="$PROJECT_ROOT/tools/dds_log_reader"
LOGSRC="$PROJECT_ROOT/tools/dds_log_reader.cpp"

g++ -std=c++17 -O2 -g \
    -I"$ROOT_DIR/include/common_msgs" \
    -o "$LOGROUT" "$LOGSRC"

echo "Build success: $LOGROUT"

echo ""
echo "用法:"
echo "  cd $PROJECT_ROOT"
echo "  source setup_env.sh"
echo "  ./tools/dds_receiver"
echo "  ./tools/dds_log_reader [output/dds_send.log]"
