# ROS 2 Lower-Limb Exoskeleton

[中文](#中文) | [English](#english)

## 中文

基于 ROS 2 Humble 的下肢外骨骼研究工作空间，围绕髋膝四关节控制与多传感器感知，集成 CPG 步态生成、阻抗控制、硬件通信、状态估计、地形建图和实验数据采集。

### 主要功能

- **步态生成**：Hopf 振荡器、傅里叶步态库、参考位置与速度输出，以及连续步态切换。
- **人机交互控制**：基于 ros2_control 的阻抗与虚拟管道控制，支持四关节参数调节和控制分量监测。
- **硬件接入**：STM32 USB 通信、四电机关节映射、双踝编码器反馈及独立足底压力采集。
- **状态估计**：基于 GTSAM 因子图融合视觉惯性里程计、IMU、关节运动和接触事件。
- **地形感知**：OpenVINS 视觉惯性估计、GPU 高程建图与平面分割，以及地形信息到 CPG 的调度接口。
- **实验工具**：Qt 调参界面、周期指标分析、原始数据录制、话题预检和 rosbag 检查。

### 模块组织

| 模块 | 功能包 |
| --- | --- |
| 步态生成与阻抗控制 | `cpg_controller`、`exoskeleton_controllers` |
| 硬件通信与调参 | `exoskeleton_hardware`、`impedance_tuning` |
| 融合状态估计 | `exo_state_estimator` |
| 地形调度框架 | `terrain_adaptive_cpg` |
| 启动、模型与数据采集 | `robot_bringup`、`robot_display`、`vins_launch`、`exo_data_recorder` |
| 视觉惯性估计与地形建图 | `open_vins/`、`elevation_mapping_cupy/` 下的算法及配套包 |

控制链路：

```text
CPG 关节参考 → 阻抗控制器 → ros2_control 硬件接口 → STM32 → 电机
```

控制器输出为电机 `iq` 指令。地形调度框架通过有效性、时效和置信度检查接收地形信息，默认配置保持平地步态参数。

### 开发环境

- Ubuntu 22.04、ROS 2 Humble、colcon。
- ros2_control、RealSense ROS 驱动、grid_map、OpenCV、Eigen、Boost、Ceres、GTSAM（含 GTSAM_UNSTABLE）和 orocos_kdl。
- GPU 建图使用 CUDA/CuPy，并依赖 NumPy、SciPy、Shapely 等 Python 库；调参界面使用 Qt。

具体依赖见各功能包的 `package.xml` 和 `CMakeLists.txt`。安装与硬件匹配的 CUDA/CuPy、RealSense 和 GTSAM 依赖后，在工作空间根目录构建：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
```

运行前，根据本机部署目录与设备连接设置 launch/YAML 中的配置路径、URDF 资源和串口参数。

### 使用说明

- [多关节调参](src/impedance_tuning/README.md)：关节选择、CPG/阻抗参数调节、软件零位与周期分析。
- [原始数据采集](src/exo_data_recorder/README.md)：相机与传感器录制、话题预检及数据检查。
- [地形调度框架](src/terrain_adaptive_cpg/README.md)：调度接口、默认配置和离线合成测试。

`robot_bringup` 提供相机、VINS、高程建图与平面分割的组合入口；控制、估计与调参模块分别提供独立启动入口。

连接实机前需固定机构、核对关节映射并确认急停有效。启动硬件接口可能使能电机；停止 CPG 或将 `iq` 设为零不等于电机断电。

### 第三方组件

本工作空间集成 OpenVINS、elevation_mapping_cupy、convex_plane_decomposition 等组件，并包含 ROS 2 接口和系统集成修改。第三方代码的版权与许可证声明保留在对应源码及功能包中。

## English

A ROS 2 Humble workspace for lower-limb exoskeleton research, combining four-joint hip–knee control with multisensor perception. It brings together CPG gait generation, impedance control, hardware communication, state estimation, terrain mapping, and experimental data collection.

### Features

- **Gait generation:** Hopf oscillators, a Fourier gait library, joint position and velocity references, and continuous gait transitions.
- **Human–robot interaction:** ros2_control impedance and virtual-tube control, with four-joint parameter tuning and control-component monitoring.
- **Hardware integration:** STM32 USB communication, four-motor joint mapping, dual ankle encoder feedback, and a separate foot-pressure acquisition interface.
- **State estimation:** GTSAM factor-graph fusion of visual–inertial odometry, IMU measurements, joint motion, and contact events.
- **Terrain perception:** OpenVINS visual–inertial estimation, GPU elevation mapping, plane segmentation, and a terrain-to-CPG scheduling interface.
- **Experimental tools:** a Qt tuning GUI, cycle metrics, raw-data recording, topic preflight checks, and rosbag inspection.

### Workspace organization

| Module | Packages |
| --- | --- |
| Gait generation and impedance control | `cpg_controller`, `exoskeleton_controllers` |
| Hardware communication and tuning | `exoskeleton_hardware`, `impedance_tuning` |
| Fused state estimation | `exo_state_estimator` |
| Terrain scheduling framework | `terrain_adaptive_cpg` |
| Bringup, visualization, and recording | `robot_bringup`, `robot_display`, `vins_launch`, `exo_data_recorder` |
| Visual–inertial estimation and terrain mapping | Algorithm and support packages under `open_vins/` and `elevation_mapping_cupy/` |

Control pipeline:

```text
CPG joint references → Impedance controller → ros2_control hardware interface → STM32 → Motors
```

The controller sends motor `iq` commands. The terrain scheduler checks validity, freshness, and confidence before accepting terrain inputs; its default configuration retains flat-walking gait parameters.

### Development environment

- Ubuntu 22.04, ROS 2 Humble, and colcon.
- ros2_control, the RealSense ROS driver, grid_map, OpenCV, Eigen, Boost, Ceres, GTSAM with GTSAM_UNSTABLE, and orocos_kdl.
- CUDA/CuPy for GPU mapping, Python libraries including NumPy, SciPy, and Shapely, and Qt for the tuning GUI.

Package dependencies are listed in `package.xml` and `CMakeLists.txt`. Install the CUDA/CuPy, RealSense, and GTSAM dependencies for your system, then build from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
```

Before launching, configure the resource paths, URDF files, and serial ports in launch files and YAML configurations for your deployment.

### Usage

- [Multi-joint tuning](src/impedance_tuning/README.md): joint selection, CPG and impedance parameters, software zero offsets, and cycle analysis.
- [Raw-data recording](src/exo_data_recorder/README.md): camera and sensor recording, topic preflight checks, and data inspection.
- [Terrain scheduling](src/terrain_adaptive_cpg/README.md): interfaces, default settings, and synthetic offline tests.

`robot_bringup` combines the camera, VINS, elevation mapping, and plane segmentation. Control, estimation, and tuning modules provide their own launch entry points.

Secure the mechanism, verify joint mapping, and check the emergency stop before connecting hardware. Starting the hardware interface may enable motors; stopping the CPG or setting `iq` to zero does not disconnect motor power.

### Third-party components

This workspace integrates OpenVINS, elevation_mapping_cupy, convex_plane_decomposition, and related components, alongside ROS 2 interfaces and system integration changes. Third-party copyright and license notices are retained in the corresponding source files and packages.
