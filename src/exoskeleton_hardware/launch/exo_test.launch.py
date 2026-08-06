#!/usr/bin/env python3
# exo_test.launch.py
# 测试 launch: 控制栈 + CPG 步态参考 + RVIZ2 可视化
#
# 可视化的是【真实电机角度】:
#   - joint_state_broadcaster 发布 /joint_states (来自 STM32 编码器反馈, 含软件归零矫正)
#   - robot_state_publisher 用真实角度驱动机器人模型
#   - 当前只有 motor2=left_knee 有真实数据, 其余关节保持 0
#
# 用法:
#   ros2 launch exoskeleton_hardware exo_test.launch.py
#   启动步态:  ros2 topic pub /cpg/state_cmd std_msgs/msg/String "data: start" --once
#   停止步态:  ros2 topic pub /cpg/state_cmd std_msgs/msg/String "data: stop" --once

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # RVIZ 配置文件 (默认 robot_display 的, 可用 rviz_config:=path 覆盖)
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('robot_display'), 'rviz', 'robot_display.rviz'),
        description='Path to RVIZ config file')

    # 是否启动 CPG 步态参考节点 (默认 true)
    start_cpg_arg = DeclareLaunchArgument(
        'start_cpg', default_value='true',
        description='Whether to start the CPG gait reference node')

    # CPG 配置文件 (默认全幅 cpg.yaml; 力矩饱和验证用 cpg_test_gentle.yaml 缓参考)
    cpg_config_arg = DeclareLaunchArgument(
        'cpg_config',
        default_value='/home/lsy/vins_ws/src/cpg_controller/config/cpg.yaml',
        description='Path to CPG config file (use cpg_test_gentle.yaml for gentle reference)')

    # ==========================================================
    # 1. 控制栈: 硬件接口 + controller_manager + joint_state_broadcaster
    #    + 阻抗控制器 + robot_state_publisher (exo_bringup.launch.py)
    # ==========================================================
    exo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('exoskeleton_hardware'),
            'launch', 'exo_bringup.launch.py')))

    # ==========================================================
    # 2. CPG 步态参考 (发布 /cpg/joint_ref 给阻抗控制器)
    #    注意: CPG 自身也会发布计算出的 /joint_states, 这里重映射到
    #    /cpg/joint_states_debug, 避免与硬件 joint_state_broadcaster 的
    #    真实 /joint_states 冲突 (可视化必须用真实电机角度)
    # ==========================================================
    cpg_node = Node(
        package='cpg_controller',
        executable='legs_cpg',
        name='adaptive_cpg_node',
        output='screen',
        parameters=[LaunchConfiguration('cpg_config')],
        remappings=[('/joint_states', '/cpg/joint_states_debug')],
        condition=IfCondition(LaunchConfiguration('start_cpg')))

    # ==========================================================
    # 3. RVIZ2 可视化
    # ==========================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen')

    return LaunchDescription([
        rviz_config_arg,
        start_cpg_arg,
        cpg_config_arg,
        exo_bringup,
        cpg_node,
        rviz_node,
    ])
