from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='football_detect',
            executable='football_detect_node',
            name='football_detect',
            output='screen',
            parameters=[
                {'engine_file': '/home/unitree/wly666/src/vision/vision/weight/weight.engine'},
                {'show_image': True}
            ]
        )
    ])
