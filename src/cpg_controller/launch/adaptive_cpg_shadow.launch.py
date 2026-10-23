"""Read the existing flat_walk prior and start only the isolated shadow CPG."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _node(context):
    package_dir = get_package_share_directory('cpg_controller')
    prior_path = LaunchConfiguration('prior_file').perform(context)
    with open(prior_path, encoding='utf-8') as source:
        parameters = yaml.safe_load(source)['adaptive_cpg_node']['ros__parameters']
    # All six channels preserve the common time/phase coordinates of flat_walk.
    gait = parameters['gait_library']['flat_walk']
    prior = {'omega_initial': float(gait['omega'])}
    for name, model in gait['joints'].items():
        for key in ['w_offset', 'w_x', 'w_y', 'limit_min', 'limit_max']:
            prior['joint.' + name + '.' + key] = model[key]
    prior['enable_learning'] = LaunchConfiguration('enable_learning').perform(context).lower() == 'true'
    prior['shape_requires_lock'] = LaunchConfiguration('shape_requires_lock').perform(context).lower() == 'true'
    prior['events_enabled'] = LaunchConfiguration('events_enabled').perform(context).lower() == 'true'
    return [Node(
        package='cpg_controller', executable='adaptive_cpg_shadow',
        namespace=LaunchConfiguration('namespace'), name='adaptive_cpg_shadow', output='screen',
        parameters=[os.path.join(package_dir, 'config', 'adaptive_learning.yaml'), prior],
    )]


def generate_launch_description():
    package_dir = get_package_share_directory('cpg_controller')
    return LaunchDescription([
        DeclareLaunchArgument('enable_learning', default_value='false'),
        DeclareLaunchArgument('events_enabled', default_value='false'),
        DeclareLaunchArgument('shape_requires_lock', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='cpg_adaptive'),
        DeclareLaunchArgument('prior_file', default_value=os.path.join(package_dir, 'config', 'cpg.yaml')),
        OpaqueFunction(function=_node),
    ])
