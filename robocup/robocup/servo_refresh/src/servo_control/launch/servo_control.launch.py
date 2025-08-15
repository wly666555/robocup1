#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('servo_control')
    
    # 声明启动参数
    DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'config.yaml'),
        description='Path to configuration file'
    ),
    
    # 创建节点
    servo_control_node = Node(
        package='servo_control',
        executable='servo_control_node',
        name='servo_control_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('rt/g1_comp_servo/state', 'rt/g1_comp_servo/state'),
            ('rt/g1_comp_servo/cmd', 'rt/g1_comp_servo/cmd'),
        ]
    )
    
    return LaunchDescription([
        servo_control_node
    ]) 