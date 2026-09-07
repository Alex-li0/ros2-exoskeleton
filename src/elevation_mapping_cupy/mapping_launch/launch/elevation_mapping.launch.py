import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'elevation_mapping_cupy'

    # Define paths
    mapping_launch_share = get_package_share_directory('mapping_launch')
    core_param_path = os.path.join(
        mapping_launch_share, 'config', 'core_param.yaml'
    )
    default_robot_param_path = os.path.join(
        mapping_launch_share, 'config', 'base.yaml'
    )
    default_plugin_config_path = os.path.join(
        mapping_launch_share, 'config', 'plugin_config.yaml'
    )

    # Declare launch arguments
    robot_param_arg = DeclareLaunchArgument(
        'robot_config',
        default_value=default_robot_param_path,
        description='Path to the robot-specific parameter YAML file'
    )

    plugin_config_arg = DeclareLaunchArgument(
        'plugin_config',
        default_value=default_plugin_config_path,
        description='Path to the elevation-map plugin YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Get launch configurations
    robot_param_path = LaunchConfiguration('robot_config')
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
            {
                'use_sim_time': use_sim_time,
                'plugin_config_file': LaunchConfiguration('plugin_config'),
            }
        ]
    )

    return LaunchDescription([
        robot_param_arg,
        plugin_config_arg,
        use_sim_time_arg,
        elevation_mapping_node,
    ])
