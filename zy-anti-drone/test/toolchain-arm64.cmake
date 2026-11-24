# ============================================================================
# ARM64 (aarch64) 交叉编译工具链文件
#
# 用法:
#   1. 安装交叉编译器:
#      sudo apt install g++-aarch64-linux-gnu
#
#   2. 使用 cmake 构建:
#      cmake -B build-arm64 \
#            -DCMAKE_TOOLCHAIN_FILE=toolchain-arm64.cmake \
#            -DTARGET_ARCH=aarch64 \
#            ..
#      cmake --build build-arm64
#
#   3. 或使用 build.sh:
#      ./build.sh aarch64
# ============================================================================

# 目标系统
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 交叉编译器前缀 (可根据实际工具链调整)
set(CROSS_COMPILE_PREFIX "aarch64-linux-gnu" CACHE STRING "Cross-compiler prefix")

# C 编译器
set(CMAKE_C_COMPILER   ${CROSS_COMPILE_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE_PREFIX}-g++)

# 其他工具
set(CMAKE_AR      ${CROSS_COMPILE_PREFIX}-ar      CACHE FILEPATH "Archiver")
set(CMAKE_LINKER  ${CROSS_COMPILE_PREFIX}-ld      CACHE FILEPATH "Linker")
set(CMAKE_NM      ${CROSS_COMPILE_PREFIX}-nm      CACHE FILEPATH "NM")
set(CMAKE_OBJCOPY ${CROSS_COMPILE_PREFIX}-objcopy CACHE FILEPATH "Objcopy")
set(CMAKE_OBJDUMP ${CROSS_COMPILE_PREFIX}-objdump CACHE FILEPATH "Objdump")
set(CMAKE_RANLIB  ${CROSS_COMPILE_PREFIX}-ranlib  CACHE FILEPATH "Ranlib")
set(CMAKE_STRIP   ${CROSS_COMPILE_PREFIX}-strip   CACHE FILEPATH "Strip")

# 交叉编译时不要在宿主机上运行测试程序
set(CMAKE_CROSSCOMPILING TRUE)

# 搜索路径（避免污染宿主机路径）
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
