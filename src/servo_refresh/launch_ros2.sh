#!/bin/bash

# 设置工作目录
cd "$(dirname "$0")"

# 检查是否在ROS2环境中
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 environment not found. Please source ROS2 setup.bash first."
    echo "Example: source /opt/ros/foxy/setup.bash"
    exit 1
fi

# 检查robot_interfaces包是否已构建
if [ ! -d "../robot_interfaces/build" ]; then
    echo "Building robot_interfaces package..."
    cd ../robot_interfaces
    colcon build
    cd ../servo_refresh
fi

# 构建servo_control包
echo "Building servo_control package..."
colcon build

# 设置环境
source install/setup.bash

echo "Starting servo_control node with ROS2 support..."
echo "Topics:"
echo "  - servo/motor_cmd (subscribe)"
echo "  - servo/motor_states (publish)"
echo ""
echo "Press Ctrl+C to stop"

# 运行节点
ros2 run servo_control servo_control_node 