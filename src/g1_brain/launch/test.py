from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g1_brain',         # 你的包名
            executable='cam_find_ball', # 你的可执行文件名
            name='cam_find_ball_node',  # 节点名
            output='screen',
            # parameters=['config/params.yaml'],  # 如果有参数文件可以加
        ),
    ])
