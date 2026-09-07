import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    terrain_share = get_package_share_directory('terrain_adaptive_cpg')
    cpg_share = get_package_share_directory('cpg_controller')
    return LaunchDescription([
        Node(
            package='terrain_adaptive_cpg',
            executable='terrain_parameter_scheduler',
            name='terrain_parameter_scheduler',
            output='screen',
            parameters=[os.path.join(terrain_share, 'config', 'terrain_scheduler.yaml')],
        ),
        Node(
            package='cpg_controller',
            executable='legs_cpg',
            name='adaptive_cpg_node',
            output='screen',
            parameters=[os.path.join(cpg_share, 'config', 'cpg.yaml')],
        ),
    ])
