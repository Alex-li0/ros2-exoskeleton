import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    package_name = 'elevation_mapping_cupy'

    # Define paths
    core_param_path = os.path.join('/home/lsy/vins_ws/src/elevation_mapping_cupy/mapping_launch/config/core_param.yaml')

    # Declare launch arguments
    robot_param_arg = DeclareLaunchArgument(
        'robot_config',
        # default_value='turtle_bot/turle_bot_simple.yaml',
        default_value='/home/lsy/vins_ws/src/elevation_mapping_cupy/mapping_launch/config/base.yaml',
        description='Name of the robot-specific config file within '
                    'config/setups/'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Get launch configurations
    robot_param_path = PathJoinSubstitution(['/home/lsy/vins_ws/src/elevation_mapping_cupy/mapping_launch/config/base.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Verify core config exists
    if not os.path.exists(core_param_path):
        raise FileNotFoundError(
            f"Config file {core_param_path} does not exist")

    # Define nodes
    elevation_mapping_node = Node(
        package=package_name,
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        output='screen',
        parameters=[
            core_param_path,
            robot_param_path,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        robot_param_arg,
        use_sim_time_arg,
        elevation_mapping_node,
    ])