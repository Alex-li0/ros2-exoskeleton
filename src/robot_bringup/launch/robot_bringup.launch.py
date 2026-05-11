import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# 定义工作空间的绝对路径，保持你免编译修改的习惯
WORKSPACE_SRC_DIR = "/home/lsy/vins_ws/src"

def generate_launch_description():

    # ========================================================================
    # 1. 声明所有模块的 YAML 配置文件接口 (对外暴露的参数)
    # ========================================================================
    
    # [深度相机] 配置文件路径
    camera_yaml_arg = DeclareLaunchArgument(
        'camera_yaml',
        default_value=f'{WORKSPACE_SRC_DIR}/robot_bringup/config/depth_camera/camera_high_res.yaml',
        description='Path to depth camera config file'
    )
    
    # [VINS] 配置文件路径
    vins_yaml_arg = DeclareLaunchArgument(
        'vins_yaml',
        default_value=f'{WORKSPACE_SRC_DIR}/vins_launch/config/vins_launch.yaml',
        description='Path to VINS config file'
    )
    
    # [高程建图] Core 配置文件路径
    mapping_core_yaml_arg = DeclareLaunchArgument(
        'mapping_core_yaml',
        default_value=f'{WORKSPACE_SRC_DIR}/elevation_mapping_cupy/mapping_launch/config/core_param.yaml',
        description='Path to elevation mapping core param file'
    )
    
    # [高程建图] Robot 配置文件路径
    mapping_robot_yaml_arg = DeclareLaunchArgument(
        'mapping_robot_yaml',
        default_value=f'{WORKSPACE_SRC_DIR}/elevation_mapping_cupy/mapping_launch/config/base.yaml',
        description='Path to elevation mapping robot param file'
    )

    # [显示/Rviz] 配置文件路径 (假设你的 display.launch.py 里留了 rviz_config 这个接口)
    display_rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=f'{WORKSPACE_SRC_DIR}/robot_bringup/rviz/default.rviz',
        description='Path to Rviz config file'
    )


    # ========================================================================
    # 2. 包含 (Include) 子启动文件，并将上面的参数映射传给它们
    # ========================================================================

    # 包含 1: 深度相机启动文件
    include_depth_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f'{WORKSPACE_SRC_DIR}/robot_bringup/launch/depth_camera.launch.py'),
        launch_arguments={
            # 左边 'camera_config' 是底层深度相机launch里定义的参数名
            # 右边 LaunchConfiguration('camera_yaml') 是提取我们在顶层声明的路径传给它
            'camera_config': LaunchConfiguration('camera_yaml'),
        }.items()
    )

    # 包含 2: VINS 里程计启动文件
    include_vins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f'{WORKSPACE_SRC_DIR}/vins_launch/launch/vins_launch.py'),
        launch_arguments={
            'yaml_file_path': LaunchConfiguration('vins_yaml'),
            # 如果你在跑真实相机，通常不需要在 VINS 里再播放包了，可以在这里直接覆盖关闭
            'bag_enable': 'false', 
        }.items()
    )

    # 包含 3: 高程建图启动文件 (注意文件名替换为你实际的 mapping.launch.py)
    include_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f'{WORKSPACE_SRC_DIR}/elevation_mapping_cupy/mapping_launch/launch/elevation_mapping.launch.py'),
        launch_arguments={
            'core_config': LaunchConfiguration('mapping_core_yaml'),
            'robot_config': LaunchConfiguration('mapping_robot_yaml'),
            'use_sim_time': 'false' # 真实机器人设为 false
        }.items()
    )

    # 包含 4: Rviz 显示启动文件
    include_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f'{WORKSPACE_SRC_DIR}/robot_bringup/launch/display.launch.py'),
        launch_arguments={
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items()
    )

    # ========================================================================
    # 3. 将所有动作打包返回
    # ========================================================================
    return LaunchDescription([
        # 声明的参数
        camera_yaml_arg,
        vins_yaml_arg,
        mapping_core_yaml_arg,
        mapping_robot_yaml_arg,
        display_rviz_arg,
        
        # 启动的子文件
        include_depth_camera,
        include_vins,
        include_mapping,
        include_display
    ])