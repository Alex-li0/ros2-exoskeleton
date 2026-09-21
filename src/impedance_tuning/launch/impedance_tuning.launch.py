#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORKSPACE_SRC = '/home/lsy/vins_ws/src'


def generate_launch_description():
    controllers = os.path.join(
        WORKSPACE_SRC, 'exoskeleton_hardware', 'config', 'exo_controllers.yaml')
    cpg = os.path.join(WORKSPACE_SRC, 'cpg_controller', 'config', 'cpg.yaml')
    hardware_share = get_package_share_directory('exoskeleton_hardware')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                hardware_share, 'launch', 'exo_bringup.launch.py')),
            launch_arguments={
                'controllers_config': controllers,
                'start_impedance_controller': 'false',
            }.items()),
        Node(
            package='cpg_controller', executable='legs_cpg',
            name='adaptive_cpg_node', output='screen', parameters=[cpg],
            remappings=[('/joint_states', '/cpg/joint_states_debug')]),
        Node(
            package='impedance_tuning', executable='impedance_tuning_gui',
            parameters=[{'controllers_config': controllers}], output='screen',
            # Qt是调参任务的主界面；关闭它时一并停止CPG、
            # controller_manager、robot_state_publisher和RViz，避免留下无窗口后台进程。
            on_exit=Shutdown(reason='阻抗调参界面已关闭')),
        # 先让GUI开始发布“未武装/零输出”心跳，再激活阻抗控制器。
        TimerAction(
            period=2.0,
            actions=[Node(
                package='controller_manager', executable='spawner',
                arguments=['exo_impedance_controller', '-c', '/controller_manager'],
                output='screen')]),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('robot_display'),
                'rviz', 'robot_display.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'),
    ])
