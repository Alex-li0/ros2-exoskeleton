from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import yaml

launch_args = [
    # 新增：指定参数文件的路径（默认指向源码路径，方便修改免编译
    DeclareLaunchArgument(
        name="yaml_file_path",
        default_value="/home/lsy/vins_ws/src/vins_launch/config/vins_launch.yaml",
        description="Full path to the YAML configuration file"
    ),

    DeclareLaunchArgument(name="namespace", default_value="ov_msckf", description="namespace"),
    DeclareLaunchArgument(
        name="ov_enable", default_value="true", description="enable OpenVINS node"
    ),
    DeclareLaunchArgument(
        name="rviz_enable", default_value="true", description="enable rviz node"
    ),
    # --- 新增：控制 Bag 播放的参数 ---
    DeclareLaunchArgument(
        name="bag_enable", default_value="true", description="enable ros2 bag play"
    ),
    DeclareLaunchArgument(
        name="bag_path",
        default_value="/home/lsy/vins_ws/src/vins_launch/bags/V1_01_easy",
        description="path to rosbag",
    ),
    # ---------------------------------
    DeclareLaunchArgument(
        name="config",
        default_value="euroc_mav",
        description="euroc_mav, tum_vi, rpng_aruco...",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined based on provided 'config' above",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="true",
        description="if we have more than 1 camera, if we should try to track stereo constraints between pairs",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="how many cameras we have 1 = mono, 2 = stereo, >2 = binocular (all mono tracking)",
    ),
    DeclareLaunchArgument(
        name="save_total_state",
        default_value="false",
        description="record the total state with calibration and features to a txt file",
    )
]

def launch_setup(context):
    # ---------------------------------------------------------
    # 2. 读取 YAML 文件
    # ---------------------------------------------------------
    yaml_file_path = LaunchConfiguration("yaml_file_path").perform(context)
    yaml_config = {}
    if os.path.exists(yaml_file_path):
        print(f"[INFO] Loading config from: {yaml_file_path}")
        with open(yaml_file_path, 'r') as f:
            yaml_config = yaml.safe_load(f)
    else:
        print(f"[WARN] Yaml file not found at {yaml_file_path}, using LaunchArguments defaults.")

    # ---------------------------------------------------------
    # 3. 辅助函数：优先取 YAML，没有则取 LaunchConfiguration
    # ---------------------------------------------------------
    def get_val(key):
        # 如果 YAML 里有这个 key，直接使用 YAML 的值（实现免编译修改）
        if key in yaml_config:
            return yaml_config[key]
        # 如果 YAML 里没有，从 Launch 参数里获取
        return LaunchConfiguration(key)
    
    

    # config_path = LaunchConfiguration("config_path").perform(context)
    config_path = get_val("config_path")
    if not config_path:
        configs_dir = os.path.join("/home/lsy/vins_ws/src/vins_launch/config")
        available_configs = os.listdir(configs_dir)
        config = get_val("config")
        if config in available_configs:
            config_path = os.path.join(
                            "/home/lsy/vins_ws/src/vins_launch/config",
                            config,"estimator_config.yaml"
                        )
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting OpenVINS".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    else:
        if not os.path.isfile(config_path):
            return [
                LogInfo(
                    msg="ERROR: config_path file: '{}' - does not exist. - not starting OpenVINS".format(
                        config_path)
                    )
            ]
            
    # OpenVINS 节点
    node1 = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        namespace=LaunchConfiguration("namespace"),
        output='screen',
        parameters=[
            {"verbosity": get_val("verbosity")},
            {"use_stereo": get_val("use_stereo")},
            {"max_cameras": get_val("max_cameras")},
            {"save_total_state": get_val("save_total_state")},
            {"config_path": config_path},
        ],
    )

    # Rviz2 节点
    node2 = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(get_val("rviz_enable")),
        arguments=[
            "-d",
            get_val("rviz_config_path"),
            "--ros-args",
            "--log-level",
            "warn",
            ],
    )

    # --- 新增：播放 Bag 的进程 ---
    # 执行命令：ros2 bag play <path>
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', get_val('bag_path')],
        output='screen',
        condition=IfCondition(get_val("bag_enable"))
    )

    # --- 新增：imu 和 camera_link 静态tf发布的进程 ---
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vins_static_tf_bridge',
        arguments=[
            '--x', '-0.006',
            '--y', '0.005',
            '--z', '0.012',
            '--qx', '0.5',
            '--qy', '-0.5',
            '--qz', '0.5',
            '--qw', '0.5',
            '--frame-id', 'imu',
            '--child-frame-id', 'camera_link'
        ]
    )

    return [node1, node2, bag_play, static_tf_node]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
