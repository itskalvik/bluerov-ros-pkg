# light_control.launch.py

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('bluerov2_control'),
                            'launch',
                            'mavros.launch'
                        ])
                    ]),
                    launch_arguments={
                    }.items()
                )
            ]
        ),

        # Light Control Service Node
        Node(
            package='bluerov2_control',
            executable='light_service_node.py',
            name='light_control_service',
            output='screen',
            parameters=[
                {'light_channel': 9},
                {'light_on_value': 1500},
                {'light_off_value': 1100}
            ]
        ),
    ])
