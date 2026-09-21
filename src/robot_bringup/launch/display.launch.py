import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 定义工作空间的绝对路径
WORKSPACE_SRC_DIR = "/home/lsy/vins_ws/src"


def generate_launch_description():
    # 改为你自己的包名和URDF文件名
    package_name = 'robot_display'
    urdf_file_name = 'robot_display.urdf'

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 0. 创建一个空的 LaunchDescription 对象

    # ==========================================================
    # 1. 声明 rviz_config 参数，预留给总启动文件调用的接口
    # ==========================================================
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=(
            f'{WORKSPACE_SRC_DIR}/robot_bringup/rviz/robot_display.rviz'),
        description='Path to the RViz config file'
    )

    # ==========================================================
    # 2. 启动 robot_state_publisher 节点
    # ==========================================================
    robot_display_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # ==========================================================
    # 3. 启动 rviz2 节点
    # ==========================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", LaunchConfiguration("rviz_config"),
            "--ros-args",
            "--log-level",
            "warn",
        ]
    )

    # ==========================================================
    # 4. 返回打包好的 LaunchDescription
    # ==========================================================
    ld = LaunchDescription()

    ld.add_action(rviz_config_arg)
    ld.add_action(robot_display_node)
    ld.add_action(rviz_node)

    return ld
