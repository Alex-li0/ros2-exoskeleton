#!/usr/bin/env python3
"""Start feedback visualization without CPG or the impedance controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_share = get_package_share_directory('exoskeleton_hardware')
    display_share = get_package_share_directory('robot_display')
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='是否启动RViz显示真实关节反馈'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                hardware_share, 'launch', 'exo_bringup.launch.py')),
            launch_arguments={
                'start_impedance_controller': 'false',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                display_share, 'rviz', 'robot_display.rviz')],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            output='screen',
        ),
    ])
