#!/bin/bash

# 设置错误时退出
set -e

# 设置工作目录
cd "$(dirname "$0")"

echo "Building servo_control package..."

# 创建构建目录
mkdir -p build
cd build

# 清理之前的构建
rm -rf *

# 配置CMake项目
cmake ..

# 编译项目
make -j$(nproc)

echo "构建完成！" 