#!/usr/bin/env python3
# exo_tune.launch.py — 调参 + 跟踪监控 一体化
#
# 所有控制参数 (kp/kd/ref_scale/torque_limit/update_rate) 都在
#   src/exoskeleton_hardware/config/exo_controllers.yaml 里改, 带注释, 免编译
#
# 用法 (无需任何终端参数):
#   ros2 launch exoskeleton_hardware exo_tune.launch.py
#
# 会自动:
#   1. 启动控制栈 (exo_bringup: 硬件 + controller_manager + 控制器, 读 exo_controllers.yaml)
#   2. 启动 CPG 步态 + RVIZ 可视化
#   3. 启动监控 (等步态 start 后采集 duration 秒, 弹出误差/延迟图表并保存 PNG)
#
# 步态控制 (另开终端):
#   start: ros2 topic pub /cpg/state_cmd std_msgs/msg/String "data: start" --once
#   stop : ros2 topic pub /cpg/state_cmd std_msgs/msg/String "data: stop" --once
#
# 监控关节选择: 按电机号 (默认 motor 2 = 当前接的真实电机), 画图从
#   exo_controllers.yaml 的 joint_motor_map 解析对应关节, 改配置文件即跟随。
#   也支持 motor:=3 / motor:=4 换其他电机。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 与工作区"免编译修改"习惯一致, 直接读 src 路径
WORKSPACE_SRC_DIR = "/home/lsy/vins_ws/src"


def generate_launch_description():
    return LaunchDescription([
        # 监控采集时长 [s] (默认 15s; 可选项, 一般不用改)
        DeclareLaunchArgument('duration', default_value='15.0'),
        # 监控展示电机 (默认 2 = 当前接的真实电机; 画图按 exo_controllers.yaml
        # 的 joint_motor_map 解析出对应关节显示, 改配置即跟随)
        DeclareLaunchArgument('motor', default_value='2'),

        # 1. 控制栈 (读 exo_controllers.yaml, 改参数直接编辑该文件)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('exoskeleton_hardware'),
                'launch', 'exo_bringup.launch.py'))),

        # 2. CPG 步态参考 (/joint_states 重映射避免与硬件 JSB 冲突)
        Node(
            package='cpg_controller', executable='legs_cpg',
            name='adaptive_cpg_node', output='screen',
            parameters=[os.path.join(WORKSPACE_SRC_DIR, 'cpg_controller', 'config', 'cpg.yaml')],
            remappings=[('/joint_states', '/cpg/joint_states_debug')]),

        # 3. RVIZ 可视化 (真实电机角度)
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('robot_display'), 'rviz', 'robot_display.rviz')],
            output='screen'),

        # 4. 监控 (等待步态启动后采集, 按电机号选关节)
        Node(
            package='exoskeleton_hardware', executable='plot_control_monitor',
            arguments=['--duration', LaunchConfiguration('duration'),
                       '--motor', LaunchConfiguration('motor'),
                       '--wait-for-start'],
            output='screen'),
    ])
