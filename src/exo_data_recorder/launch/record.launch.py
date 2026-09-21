#!/usr/bin/env python3
"""Record an explicit raw/debug topic allow-list without starting algorithms."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _record(context):
    share = get_package_share_directory('exo_data_recorder')
    config_path = LaunchConfiguration('topics_config').perform(context)
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    topics = list(config['required_topics']) + list(config.get('optional_topics', {}))
    topics = list(dict.fromkeys(topics))
    command = [
        'ros2', 'bag', 'record',
        '--output', LaunchConfiguration('output').perform(context),
        '--storage', 'sqlite3',
        '--max-bag-duration', LaunchConfiguration('split_duration').perform(context),
        '--max-cache-size', LaunchConfiguration('max_cache_size').perform(context),
        '--qos-profile-overrides-path',
        os.path.join(share, 'config', 'rosbag2_qos_overrides.yaml'),
        *topics,
    ]
    return [ExecuteProcess(cmd=command, output='screen', sigterm_timeout='20')]


def generate_launch_description():
    share = get_package_share_directory('exo_data_recorder')
    default_topics = os.path.join(share, 'config', 'raw_topics.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('output', default_value='raw_exo_bag'),
        DeclareLaunchArgument('topics_config', default_value=default_topics),
        DeclareLaunchArgument('split_duration', default_value='300'),
        DeclareLaunchArgument('max_cache_size', default_value='1073741824'),
        OpaqueFunction(function=_record),
    ])
