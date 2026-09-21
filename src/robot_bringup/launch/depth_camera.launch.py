from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # 获取 YAML 配置文件路径
    # 注意：这里使用的是绝对路径，如果以后移动了工作空间，记得修改这里
    config_file_path = (
        '/home/lsy/vins_ws/src/robot_bringup/config/depth_camera/'
        'camera_high_res.yaml')
    # 将camera_config的yaml文件ROS参数化
    # 定义 Launch 参数 (允许命令行覆盖)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_config',
            default_value=config_file_path,
            description='Path to the camera YAML file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # 启动 Realsense 节点
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='',
            output='screen',
            # 这里的 parameters 列表会将 YAML 文件内容加载进去
            parameters=[LaunchConfiguration('camera_config'),
                        {'use_sim_time': use_sim_time}],
            # 增加 respawn 可以在相机掉线(USB松动)时自动重启
            respawn=True,
            respawn_delay=2.0
        ),
    ])
