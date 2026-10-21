# ROS 2 下肢外骨骼工作空间

面向下肢外骨骼研究的 ROS 2 Humble 工作空间，包含步态参考、阻抗控制、硬件通信、状态估计、地形感知和数据采集。

当前仓库用于私有保存与整理开发历史。许可证、资料公开范围及路径可移植性尚未全部整理完成，不能将上传完成视为公开发布准备完成。

## 模块与实现范围

源码共 22 个 ROS 包：10 个项目顶层包，以及 12 个 OpenVINS、建图、平面分割和配套包。工作空间根目录 `docs/` 在本机完整仓库中继续通过 Git 管理，其文件和历史不包含在 GitHub 发布仓库中。

| 范围 | 主要包 | 当前用途 |
| --- | --- | --- |
| 步态与控制 | `cpg_controller`、`exoskeleton_controllers` | CPG 关节参考、ros2_control 阻抗与虚拟管道控制 |
| 硬件与调参 | `exoskeleton_hardware`、`impedance_tuning` | STM32 通信、关节反馈、足底采集及 Qt 调参 |
| 状态估计 | `exo_state_estimator` | 基于 GTSAM 的融合估计与事件门控接触因子 |
| 地形感知 | `open_vins`、`elevation_mapping_cupy` 下的算法包 | VIO、高程建图与平面分割，包含上游源码和本地移植修改 |
| 地形调度 | `terrain_adaptive_cpg` | 带有效性检查的调度框架，实测标定模型仍待补充 |
| 启动与录制 | `robot_bringup`、`vins_launch`、`exo_data_recorder`、`robot_display` | 模块组合、数据采集及模型显示 |

当前控制链为 `legs_cpg → /cpg/joint_ref → ExoImpedanceController → ExoHardwareInterface → STM32`，控制输出为电机 `iq`，尚不能视为已标定的 N·m。

地形调度的实现边界见[框架说明](src/terrain_adaptive_cpg/README.md)。当前未实现或未标定的能力不能作为已完成的实机地形自适应控制结果。

## 环境与构建

当前开发系统为 Ubuntu 22.04、ROS 2 Humble。系统依赖包含 ros2_control、RealSense 驱动、OpenCV、Eigen、Ceres、GTSAM/GTSAM_UNSTABLE、grid_map，以及建图所需的 CUDA/CuPy 和 Python 库。具体依赖以各包 `package.xml`、`CMakeLists.txt` 为准；目前没有完整的依赖锁定文件或已验证的可复现容器。

以下命令用于准备依赖后的独立源码副本；`rosdep` 未覆盖或无法解析的依赖仍需单独配置。当前不提供已验证的一键安装流程。

```bash
source /opt/ros/humble/setup.bash
cd /path/to/vins_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
```

仓库没有包含本机的 `build/`、`install/`、`log/` 或 rosbag 数据库。部分启动文件仍使用 `/home/lsy/vins_ws/src`，在其他路径克隆后需要适配，不能直接承诺上述命令即可完整运行。安装目录中部分资源也是普通副本，不能假定所有源码配置修改都会即时生效。

## 使用入口

以下入口需要相应模块的依赖和配置。硬件 launch 可能连接并使能电机，不能将 CPG 停止、控制输出为零或传感器录制等同于电机已断电。连接实机前需固定机构、确认急停，并按对应调试说明检查。

- [调参工具](src/impedance_tuning/README.md)：关节选择、CPG/阻抗调参、软件零位与周期分析。
- [数据采集](src/exo_data_recorder/README.md)：原始话题预检、录制与 bag 检查。
- [地形调度与离线仿真](src/terrain_adaptive_cpg/README.md)：框架默认行为、合成测试与待采数据。
- `robot_bringup` 的当前总入口组合相机、VINS、高程建图和平面分割；电机控制、融合估计及地形调度使用各包独立入口。

## 来源、许可与发布状态

OpenVINS 的包清单与代码头声明 GPLv3；建图、平面分割相关算法包的包清单声明 MIT。`mapping_launch` 和项目顶层包仍有 TODO 许可字段。上游代码及其已有版权声明保留，自有部分没有在本次整理中被自动指定许可证。

论文副本、供应商手册、机器人模型和数据资源需要逐项核实来源及可公开范围；本仓库当前没有统一授予这些材料的再分发许可。公开前仍需完成许可证、资料来源、本机路径适配与独立环境构建验证；GitHub CI 尚未配置。

原始完整历史保留在本机仓库及离线备份中。GitHub 发布副本从全部历史排除根目录 `docs/`，保留其余文件的修改过程、提交作者、时间和说明；过滤后的提交 SHA 与本机原仓库不同。构建或离线测试结果与实机验证分别记录。
