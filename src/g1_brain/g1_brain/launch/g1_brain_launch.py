#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('g1_brain')
    
    # 声明launch参数
    config_file = LaunchConfiguration('config_file')
    
    # 声明参数
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'g1_brain_config.yaml'),
        description='Path to the config file'
    )
    
    # 创建g1_brain节点
    g1_brain_node = Node(
        package='g1_brain',
        executable='g1_brain',
        name='g1_brain_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('servo/motor_states', '/servo/motor_states'),
            ('servo/motor_cmd', '/servo/motor_cmd'),
            ('vision/detections', '/vision/detections')
        ]
    )
    
    return LaunchDescription([
        declare_config_file_cmd,
        g1_brain_node,   # 必须加上这一行
    ])
