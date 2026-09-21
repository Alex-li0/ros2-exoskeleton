#!/usr/bin/env python3
"""Start only the D435i source streams used by offline algorithms."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('exo_data_recorder')
    default_config = os.path.join(share, 'config', 'realsense_raw.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera_config', default_value=default_config),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[LaunchConfiguration('camera_config')],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
