#!/usr/bin/env python3
# exo_bringup.launch.py — 外骨骼控制栈 (硬件接口 + controller_manager + 控制器)
#
# 用法:
#   ros2 launch exoskeleton_hardware exo_bringup.launch.py
#   # 自定义 URDF / 控制器配置 (默认用包内自带)
#   ros2 launch exoskeleton_hardware exo_bringup.launch.py \
#       urdf:=/path/to/exo.urdf.xacro controllers_config:=/path/to/exo_controllers.yaml

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 与工作区"免编译修改"习惯一致: 默认直接读 src, 改 yaml/urdf 无需重编译
    WORKSPACE_SRC_DIR = "/home/lsy/vins_ws/src"

    # 默认 URDF (含 <ros2_control> 标签) 与控制器配置
    urdf_file = os.path.join(
        WORKSPACE_SRC_DIR, 'exoskeleton_hardware', 'urdf', 'exo_ros2_control.urdf.xacro')
    controllers_yaml = os.path.join(
        WORKSPACE_SRC_DIR, 'exoskeleton_hardware', 'config', 'exo_controllers.yaml')

    return LaunchDescription([
        # 参数声明 (exo_tune.launch.py 可传入自定义值)
        DeclareLaunchArgument(
            'urdf', default_value=urdf_file,
            description='URDF 文件路径（需包含 <ros2_control> 标签）'),
        DeclareLaunchArgument(
            'controllers_config', default_value=controllers_yaml,
            description='控制器 YAML 配置文件路径'),
        DeclareLaunchArgument(
            'start_impedance_controller', default_value='true',
            description='是否加载并激活阻抗控制器'),

        DeclareLaunchArgument(
            'read_only', default_value='false',
            description='只接收反馈；STM32也必须使用禁用电机调度的测试固件'),

        # robot_state_publisher（发布 URDF 到 /robot_description 和 TF）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('urdf'),
                         ' read_only:=', LaunchConfiguration('read_only')]), value_type=str)}],
            output='screen'),

        # ros2_control node (controller_manager + hardware interface)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': ParameterValue(
                    Command(['xacro ', LaunchConfiguration('urdf'),
                             ' read_only:=', LaunchConfiguration('read_only')]), value_type=str)},
                LaunchConfiguration('controllers_config'),
                {'thread_priority': 50},  # RT 调度优先级
            ],
            output='screen',
        ),

        # 加载并激活 joint_state_broadcaster (spawner 默认 load + activate)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            output='screen',
        ),

        # 加载并激活阻抗控制器 (订阅 /cpg/joint_ref 参考, 输出 effort 到硬件接口)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['exo_impedance_controller', '-c', '/controller_manager'],
            condition=IfCondition(LaunchConfiguration('start_impedance_controller')),
            output='screen',
        ),
    ])
