import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 定义路径
    # 使用绝对路径，确保 OpenCV 一定能找到文件
    openvins_config = '/home/lsy/vins_ws/src/vins_launch/config/d435i_openvins.yaml'
    rviz_config_file = '/home/lsy/vins_ws/src/vins_launch/rviz/config.rviz'

    # 2. 定义启动参数
    # 既然你自己启动了相机，这里我们加个开关，默认不启动相机，只启动算法
    start_camera_arg = DeclareLaunchArgument(
        'start_camera',
        default_value='false', # 默认 false，因为你说你已经单独启动了相机
        description='Whether to start the camera driver node'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # 3. 节点定义
    
    # A. 相机驱动 (只有当 start_camera=true 时才启动)
    # 这样你就不用去改 launch 文件代码来隔离相机了
    depth_camera_pkg = get_package_share_directory('depth_camera')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depth_camera_pkg, 'launch', 'depth_camera.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('start_camera'))
    )

    # B. OpenVINS 算法节点
    ov_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf', 
        name='ov_msckf',
        output='screen',
        # 【关键修正】这里必须用字典形式 {'key': 'value'}
        parameters=[{'config_path': openvins_config}]
    )

    # C. 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        # start_camera_arg,
        # use_rviz_arg,
        # camera_launch,
        ov_node,
        # rviz_node
    ])