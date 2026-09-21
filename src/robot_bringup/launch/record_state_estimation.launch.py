#!/usr/bin/env python3
"""Record raw exoskeleton/perception data plus online outputs for later replay."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_recording(context):
    share = get_package_share_directory('robot_bringup')
    config_path = LaunchConfiguration('topics_config').perform(context)
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    topics = list(config['required_topics']) + list(config.get('optional_topics', {}))
    # 保持顺序并消除 YAML 中可能的重复项。
    topics = list(dict.fromkeys(topics))

    cmd = [
        'ros2', 'bag', 'record',
        '--output', LaunchConfiguration('output').perform(context),
        '--storage', LaunchConfiguration('storage').perform(context),
        '--max-bag-duration', LaunchConfiguration('split_duration').perform(context),
        '--max-cache-size', LaunchConfiguration('max_cache_size').perform(context),
        '--qos-profile-overrides-path',
        os.path.join(share, 'config', 'rosbag2_qos_overrides.yaml'),
        *topics,
    ]
    return [ExecuteProcess(cmd=cmd, output='screen', sigterm_timeout='20')]


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'output', default_value='state_estimation_bag',
            description='Output bag directory; it must not already exist'),
        DeclareLaunchArgument(
            'topics_config',
            default_value=os.path.join(share, 'config', 'state_estimation_record_topics.yaml')),
        DeclareLaunchArgument('storage', default_value='sqlite3'),
        DeclareLaunchArgument(
            'split_duration', default_value='300',
            description='Split bag every N seconds to limit damage after an interrupted run'),
        DeclareLaunchArgument(
            'max_cache_size', default_value='1073741824',
            description='Recorder cache in bytes (default 1 GiB)'),
        OpaqueFunction(function=_launch_recording),
    ])
