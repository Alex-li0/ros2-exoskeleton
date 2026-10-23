"""Explicit synthetic experiment: shadow learner plus deformed measurements."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _demo(context):
    package_dir = get_package_share_directory('cpg_controller')
    namespace = LaunchConfiguration('namespace').perform(context)
    prior = LaunchConfiguration('prior_file').perform(context)
    output = LaunchConfiguration('output').perform(context)
    arguments = ['--case', LaunchConfiguration('case').perform(context),
                 '--duration', LaunchConfiguration('duration').perform(context),
                 '--namespace', namespace, '--prior-file', prior, '--events']
    if output:
        arguments += ['--output', output]
    replay = Node(package='cpg_controller', executable='replay_adaptive_measurements.py',
                  namespace=namespace, name='adaptive_measurement_replay', output='screen', arguments=arguments)
    learner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'adaptive_cpg_shadow.launch.py')),
        launch_arguments={'namespace': namespace, 'prior_file': prior,
                          'enable_learning': 'true', 'events_enabled': 'true',
                          'shape_requires_lock': LaunchConfiguration('shape_requires_lock').perform(context)}.items())
    return [learner, replay, RegisterEventHandler(OnProcessExit(
        target_action=replay, on_exit=[EmitEvent(event=Shutdown(reason='Synthetic replay finished'))]))]


def generate_launch_description():
    package_dir = get_package_share_directory('cpg_controller')
    return LaunchDescription([
        DeclareLaunchArgument('case', default_value='combined'),
        DeclareLaunchArgument('duration', default_value='50'),
        DeclareLaunchArgument('shape_requires_lock', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='cpg_adaptive'),
        DeclareLaunchArgument('prior_file', default_value=os.path.join(package_dir, 'config', 'cpg.yaml')),
        DeclareLaunchArgument('output', default_value='', description='Optional replay trace CSV path'),
        OpaqueFunction(function=_demo),
    ])
