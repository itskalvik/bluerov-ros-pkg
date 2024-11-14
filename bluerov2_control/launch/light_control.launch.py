from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bluerov2_control',
            executable='light_service_node.py',
            name='light_control_service',
            output='screen'
        )
    ])

