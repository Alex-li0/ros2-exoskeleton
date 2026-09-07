import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('terrain_adaptive_cpg')
    with open(os.path.join(share, 'config', 'record_topics.yaml'), encoding='utf-8') as stream:
        topics = yaml.safe_load(stream)['topics']
    output = LaunchConfiguration('output')
    return LaunchDescription([
        DeclareLaunchArgument('output', default_value='terrain_validation_bag'),
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', output, *topics], output='screen'),
    ])
