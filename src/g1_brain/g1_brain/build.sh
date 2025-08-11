#!/bin/bash

# 宇树机器人足球系统 g1_brain 构建脚本

echo "开始构建 g1_brain 包..."

# 设置ROS2环境
source /opt/ros/foxy/setup.bash

# 设置工作空间
export ROS2_WORKSPACE=/home/c/Desktop/unitree_football/ROS2_node

# 首先构建robot_interfaces包
echo "构建 robot_interfaces 包..."
cd $ROS2_WORKSPACE/robot_interfaces
rm -rf build/ install/ log/
colcon build

# 然后构建g1_brain包
echo "构建 g1_brain 包..."
cd $ROS2_WORKSPACE/g1_brain
rm -rf build/ install/ log/

# 设置robot_interfaces的安装路径
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$ROS2_WORKSPACE/robot_interfaces/install
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS2_WORKSPACE/robot_interfaces
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROS2_WORKSPACE/robot_interfaces/install/robot_interfaces/lib

colcon build

# 检查构建结果
if [ $? -eq 0 ]; then
    echo "构建成功！"
    echo "可以运行以下命令启动节点："
    echo "source install/setup.bash"
    echo "ros2 launch g1_brain g1_brain_launch.py"
else
    echo "构建失败！"
    exit 1
fi 