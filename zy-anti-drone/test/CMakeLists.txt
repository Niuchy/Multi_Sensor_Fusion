cmake_minimum_required(VERSION 3.10)

project(fusion-test)

set(CMAKE_CXX_STANDARD 14)

# ============================================================================
# 目标架构选择: x86_64 (native) 或 aarch64 (ARM64 交叉编译)
# 用法: cmake -B build -DTARGET_ARCH=aarch64 ..
# ============================================================================
set(TARGET_ARCH "x86_64" CACHE STRING "Target architecture: x86_64 or aarch64")
set_property(CACHE TARGET_ARCH PROPERTY STRINGS "x86_64" "aarch64")

# 根据架构选择第三方库平台目录
if(TARGET_ARCH STREQUAL "aarch64")
    set(THIRDPARTY_PLATFORM "linux-arm64")
else()
    set(THIRDPARTY_PLATFORM "linux-x86")
endif()

# Project root
set(PROJECT_ROOT ${CMAKE_SOURCE_DIR}/..)

# 第三方库平台根目录（含 include/ 和 lib/）
set(THIRDPARTY_PLATFORM_ROOT ${PROJECT_ROOT}/thirdparty/${THIRDPARTY_PLATFORM})

# ============================================================================
# Include 目录
# ============================================================================
include_directories(
    # 项目自有头文件
    ${PROJECT_ROOT}/thirdparty/PubSub/include
    ${PROJECT_ROOT}/thirdparty/common/include
    ${PROJECT_ROOT}/bridge
    ${PROJECT_ROOT}/include/common_msgs
    # 第三方库头文件（平台相关）
    ${THIRDPARTY_PLATFORM_ROOT}/include
    ${THIRDPARTY_PLATFORM_ROOT}/include/fastcdr
    ${THIRDPARTY_PLATFORM_ROOT}/include/foonathan_memory
)

# ============================================================================
# Link 目录
# ============================================================================
link_directories(
    ${THIRDPARTY_PLATFORM_ROOT}/lib
)

# libPubSubTools.so 和 libcore.so 等均在平台目录的 lib/ 下

# ============================================================================
# 可执行目标
# ============================================================================
add_executable(publisher publisher.cpp)
target_link_libraries(publisher PubSubTools core fastrtps fastcdr)

add_executable(subscriber subscriber.cpp)
target_link_libraries(subscriber PubSubTools core fastrtps fastcdr)

# ============================================================================
# 打印构建信息
# ============================================================================
message(STATUS "========================================")
message(STATUS "  Target Architecture : ${TARGET_ARCH}")
message(STATUS "  Thirdparty Platform : ${THIRDPARTY_PLATFORM}")
message(STATUS "  Platform Root       : ${THIRDPARTY_PLATFORM_ROOT}")
message(STATUS "  C++ Compiler        : ${CMAKE_CXX_COMPILER}")
message(STATUS "========================================")
