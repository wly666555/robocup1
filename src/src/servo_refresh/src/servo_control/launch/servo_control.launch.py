from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_control',
            executable='servo_control_node',
            name='servo_control_node',
            output='screen',
            remappings=[
                ('rt/g1_comp_servo/state', 'rt/g1_comp_servo/state'),
                ('rt/g1_comp_servo/cmd', 'rt/g1_comp_servo/cmd'),
            ]
        )
    ])
