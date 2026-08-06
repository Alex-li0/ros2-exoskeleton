import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('exo_state_estimator')
    config_file = os.path.join(pkg_dir, 'config', 'estimator_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to estimator parameters YAML file'
        ),

        Node(
            package='exo_state_estimator',
            executable='ct_exo_state_node',
            name='ct_exo_state_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
            ],
            remappings=[
                # 独立 IMU 传感器 (非 VIO 内置的 D435i IMU)
                # 当 use_imu_factor=true 时需要配置正确的 topic
                ('/imu/standalone', '/imu/pelvis'),

                # VIO 来自 OpenVINS
                # ('/ov_msckf/odomimu', '/ov_msckf/odomimu'),

                # 关节状态 (来自 CPG 控制器或硬件)
                # ('/cpg/joint_ref', '/joint_states/hardware'),

                # 高程图
                # ('/elevation_mapping_node/elevation_map_raw',
                #  '/elevation_mapping_node/elevation_map_raw'),
            ],
        ),
    ])
