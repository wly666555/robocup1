#!/usr/bin/env python3

import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def handle_configuration(context, *args, **kwargs):

    config_path = os.path.join(os.path.dirname(__file__), '../config')
    config_file = os.path.join(config_path, 'g1_brain_config.yaml') 
    config_local_file = os.path.join(config_path, 'g1_brain_config_local.yaml') 
    

    behavior_trees_dir = os.path.join(os.path.dirname(__file__), '../behavior_trees')
    def make_tree_path(name):
        if not name.endswith('.xml'):
            name += '.xml'
        return os.path.join(behavior_trees_dir, name)
    tree = context.perform_substitution(LaunchConfiguration('tree'))
    tree_path = make_tree_path(tree)

    # 这里的 config 覆盖 config_file 中相同字段(如有) 用于在 launch 时快速指定参数, 而不需要频繁修改 config.yaml
    config = {
            # 加载哪个行为树文件
            "tree_file_path": tree_path,
    }
    start_pos = context.perform_substitution(LaunchConfiguration('pos'))
    if not start_pos == '':
        config['game.playerStartPos'] = start_pos
    role = context.perform_substitution(LaunchConfiguration('role'))
    if not role == '':
        config['game.player_role'] = role

    return [
        Node(
            package ='g1_brain',
            executable='g1_brain_node',
            name='g1_brain_node',
            output='screen',
            parameters=[
                config_file,
                config_local_file,
                config
            ]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        # 需要可以通过 ros2 launch brain launch.py param:=value 形式提供的参数, 需要在这里用 DeclarelaunchArgument 声明, 然后在 handle_configuration 处理
        DeclareLaunchArgument(
            'tree', 
            default_value='game.xml',
            description='Specify behavior tree file name. DO NOT include full path, file should be in src/brain/config/behavior_trees'
        ),
        DeclareLaunchArgument(
            'pos', 
            default_value='left',
            description='如果需要覆盖 config.yaml 中的 game.player_start_pos, 可以在 launch 时指定参数 pos:=left'
        ),
        DeclareLaunchArgument(
            'role', 
            default_value='striker',
            description='如果需要覆盖 config.yaml 中的 game.player_role, 可以在 launch 时指定参数 role:=striker'
        ),
        OpaqueFunction(function=handle_configuration) # 转到 handle_configuration 中继续处理
    ])