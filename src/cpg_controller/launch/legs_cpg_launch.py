import os

# 引入 ROS 2 的路径解析工具
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    # 1. 动态获取你当前功能包 (cpg_controller) 的共享目录
    cpg_pkg_dir = get_package_share_directory('cpg_controller')
    
    # yaml文件是固定路径
    legs_cpgconfig_file = '/home/lsy/vins_ws/src/cpg_controller/config/cpg.yaml'
    rviz_config_file = '/home/lsy/vins_ws/src/cpg_controller/rviz/cpg_test.rviz'
    # launch文件是动态路径
    robot_display_dir = get_package_share_directory('robot_display')
    robot_display_launch_file = os.path.join(robot_display_dir, 'launch', 'display.launch.py')


    # 3. 定义 CPG 节点
    single_leg_cpg_node = Node(
        package='cpg_controller',
        executable='legs_cpg',
        name='adaptive_cpg_node', 
        output='screen',
        parameters=[legs_cpgconfig_file]  
    )

    # 4. Include 现成的 launch 文件，并传入你的 rviz 配置文件路径
    include_robot_display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_display_launch_file),
        launch_arguments={'rviz_config': rviz_config_file}.items()
    )

    # 5. 添加到 LaunchDescription
    ld.add_action(single_leg_cpg_node)
    ld.add_action(include_robot_display_launch)

    return ld