import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'elevation_mapping_cupy'
    bringup_share = get_package_share_directory('robot_bringup')

    # 1. 声明 core_config 参数，允许外部传入替换
    core_param_arg = DeclareLaunchArgument(
        'core_config',
        default_value=os.path.join(
            bringup_share, 'config', 'mapping_launch', 'core_param.yaml'
        ),
        description='Path to core parameter YAML file'
    )

    # 2. 声明 robot_config 参数，允许外部传入替换
    robot_param_arg = DeclareLaunchArgument(
        'robot_config',
        default_value=os.path.join(
            bringup_share, 'config', 'mapping_launch', 'base.yaml'
        ),
        description='Path to robot-specific config YAML file'
    )

    plugin_config_arg = DeclareLaunchArgument(
        'plugin_config',
        default_value=os.path.join(
            bringup_share, 'config', 'mapping_launch', 'plugin_config.yaml'
        ),
        description='Path to elevation-map plugin YAML file'
    )

    # 3. 声明 use_sim_time 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # 定义节点
    elevation_mapping_node = Node(
        package=package_name,
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        output='screen',
        parameters=[
            # 关键修改：直接将 LaunchConfiguration 放入 parameters 列表中
            # 节点启动时，ROS 2 底层会自动去解析这些占位符并加载 YAML 文件
            LaunchConfiguration('core_config'),
            LaunchConfiguration('robot_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'plugin_config_file': LaunchConfiguration('plugin_config'),
            }
        ]
    )

    return LaunchDescription([
        core_param_arg,
        robot_param_arg,
        plugin_config_arg,
        use_sim_time_arg,
        elevation_mapping_node,
    ])
