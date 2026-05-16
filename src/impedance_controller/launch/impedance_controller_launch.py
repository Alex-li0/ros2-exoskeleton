import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('impedance_controller'),
        'config', 'impedance_params.yaml')

    return LaunchDescription([
        Node(
            package='impedance_controller',
            executable='impedance_controller_node',
            name='impedance_controller_node',
            output='screen',
            parameters=[config_file],
        )
    ])
