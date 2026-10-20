"""Start only the optional TTL-USB insole reader, for calibration and recording."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('left_port', default_value='', description='Left TTL-USB path'),
        DeclareLaunchArgument('right_port', default_value='', description='Right TTL-USB path'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('stale_timeout_s', default_value='0.2'),
        Node(
            package='exoskeleton_hardware', executable='foot_pressure_reader',
            output='screen', parameters=[{
                'left_port': ParameterValue(LaunchConfiguration('left_port'), value_type=str),
                'right_port': ParameterValue(LaunchConfiguration('right_port'), value_type=str),
                'baud_rate': ParameterValue(LaunchConfiguration('baud_rate'), value_type=int),
                'stale_timeout_s': ParameterValue(
                    LaunchConfiguration('stale_timeout_s'), value_type=float),
            }],
        ),
    ])
