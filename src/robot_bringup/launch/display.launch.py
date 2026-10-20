import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    display_share = get_package_share_directory('robot_display')
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(display_share, 'rviz', 'robot_display.rviz'),
            description='Path to the RViz config file'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(display_share, 'launch', 'display.launch.py')
            ),
            launch_arguments={
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items()
        ),
    ])
