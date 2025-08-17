#!/bin/bash

# 设置错误时退出
set -e

echo "启动舵机控制节点..."

# 设置工作目录
cd "$(dirname "$0")"

# 设置ROS2环境
source /opt/ros/foxy/setup.bash

# 设置工作空间环境
source ../../install/setup.bash

# 启动舵机控制节点
ros2 run servo_control servo_control_node

echo "舵机控制节点已启动" 