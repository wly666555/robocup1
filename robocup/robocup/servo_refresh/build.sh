#!/bin/bash

# 设置错误时退出
set -e

echo "Building servo_control package..."

# 创建构建目录
mkdir -p build
cd build

# 清理之前的构建
rm -rf *

# 设置ROS2环境
source /opt/ros/foxy/setup.bash

# 设置Unitree SDK环境（如果存在）
if [ -f "/home/unitree/unitree_sdk2/setup.bash" ]; then
    source /home/unitree/unitree_sdk2/setup.bash
fi

# 配置CMake
cmake ../src/servo_control \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 编译
make -j$(nproc)

echo "Build completed successfully!"
echo "Executable location: build/servo_control_node" 