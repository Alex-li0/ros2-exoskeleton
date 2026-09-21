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
                # VIO 来自 OpenVINS
                # ('/ov_msckf/odomimu', '/ov_msckf/odomimu'),

                # imu_topic 和 joint_state_topic 在 estimator_params.yaml 中配置。

                # 高程图
                # ('/elevation_mapping_node/elevation_map_raw',
                #  '/elevation_mapping_node/elevation_map_raw'),
            ],
        ),
    ])
