from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[
                {'fcu_url': '/dev/ttyACM0:115200'},
                {'gcs_url': ''},
                {'target_system_id': 1},
                {'target_component_id': 1},
                {'pluginlists':'/home/ros2_ws/src/bluerov2_control/launch/apm_pluginlist.yaml'},
                {'enable_sim_time': False}
            ]
        ),
    ])
