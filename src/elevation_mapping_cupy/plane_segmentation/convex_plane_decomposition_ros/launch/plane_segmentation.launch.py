from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_parameters = PathJoinSubstitution(
        [FindPackageShare("convex_plane_decomposition_ros"), "config", "plane_segmentation.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("convex_plane_decomposition_ros"), "rviz", "plane_segmentation.rviz"]
    )
    parameters = LaunchConfiguration("parameters")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters",
                default_value=default_parameters,
                description="Plane segmentation parameter YAML",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Launch RViz2 with the plane segmentation displays",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz2 configuration file",
            ),
            Node(
                package="convex_plane_decomposition_ros",
                executable="plane_segmentation_node",
                name="plane_segmentation_node",
                output="screen",
                parameters=[parameters, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="plane_segmentation_rviz",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
