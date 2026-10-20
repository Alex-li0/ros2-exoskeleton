# 足底力：标定/录包时独立TTL–USB读取

更新：2026-10-20。读取程序已实现于现有`exoskeleton_hardware`包，没有新增功能包。它是按需启动的独立ROS 2节点，不依赖STM32、controller_manager或电机控制器；普通电机bringup不会自动启动它。实物数据尚未验证。

## 接线与端口

每脚采集板TX→各自TTL–USB适配器RX，分别共地，USB接上位机。两脚TX不可并接同一RX。确认适配器的逻辑电平匹配采集板TX；采集板5 V供电不代表TX为5 V或3.3 V。仅接收时模块配置RX可不接。

程序只读显式指定的端口，不扫描、不写串口数据，也不发送配置、清错、使能或目标命令。模块脚别和波特率应预先配置为左1、右2、115200/8N1，方法见[保留的传感器使用说明](/home/lsy/vins_ws/docs/传感器与通信/足底压力传感器使用与标定说明.md)。可只指定一脚；两脚不能指定为同一设备，收到与端口绑定脚别不符的合法帧会拒收并计数。

查看适配器稳定路径：

```bash
ls -l /dev/serial/by-id/
```

电机硬件接口现有`serial_port=auto`会扫描并打开ttyACM/ttyUSB候选端口。并行录制前应把电机URDF硬件参数`serial_port`固定为已确认的STM32路径，避免其扫描清空足底端口接收缓冲；足底节点始终使用另外的适配器路径。

## 构建与按需启动

```bash
cd /home/lsy/vins_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select exoskeleton_hardware --symlink-install
source install/setup.bash
ros2 launch exoskeleton_hardware foot_pressure_usb.launch.py \
  left_port:=/dev/serial/by-id/LEFT_ADAPTER \
  right_port:=/dev/serial/by-id/RIGHT_ADAPTER
```

将上面两个占位路径替换为实际适配器路径。单脚测试只传对应一个端口参数。两个端口均不填会报错退出，不会猜测设备。脚本依赖`python3-serial`，已在package.xml声明。

也可不用launch，直接启动同一个可执行程序：

```bash
ros2 run exoskeleton_hardware foot_pressure_reader --ros-args \
  -p left_port:=/dev/serial/by-id/LEFT_ADAPTER \
  -p right_port:=/dev/serial/by-id/RIGHT_ADAPTER
```

参数`baud_rate`默认115200，也支持资料列出的9600；`stale_timeout_s`默认0.2秒，是诊断起点，不是标定后的事件阈值。节点每5 ms非阻塞轮询，健康状态每100 ms发布；这不会把模块约20 Hz的新数据变成200 Hz采样。串口异常后每秒尝试重连，成功后必须收到新有效帧才恢复健康状态。

## 话题和录包

```bash
ros2 topic echo /foot_pressure/raw
ros2 topic echo /foot_pressure/health
ros2 bag record -o foot_pressure_calibration \
  /foot_pressure/raw /foot_pressure/health /joint_states /dynamic_joint_states
```

关节话题来自已运行的其他节点，本launch只启动足底读取。需要小腿IMU、CPG或视频时另行加入实际存在的数据源。标定结束Ctrl-C退出读取节点即可；日常运行不启动该节点。

`/foot_pressure/raw`类型为**`exoskeleton_hardware/msg/FootPressureSample`**：

| 字段 | 含义 |
| --- | --- |
| `header.stamp` | PC的ROS接收/解析时间；不是厂家物理采样时刻 |
| `foot_id` | 1左、2右，按帧校验并与端口绑定关系核对 |
| `raw_g[18]` | 点1～18原始uint16值，厂家标称g；未经载荷标定，不转成N/Pa |
| `sample_index` | 每脚从本次读取进程开始累计的合法样本数；重连延续、进程重启归零 |

两脚共用一个原始话题，以foot_id区分。只在收到新合法帧时发布；相同压力值的新帧照常计数，空读/超时不重发缓存。厂家帧没有时间戳、序号，本地计数不能测得厂家漏采或传输丢帧数。多帧积压在一次read中读出时，时间戳接近的是解析时刻，不能据此声称两脚硬件同步。

`/foot_pressure/health`使用标准**`diagnostic_msgs/msg/DiagnosticArray`**，只包含已配置的脚：正常接收为OK，超过超时或尚无样本为WARN，串口未连接为ERROR。记录最近有效样本的主机年龄、累计合法样本数、校验错误、错脚帧、部分帧超时、连接次数和最近串口错误。健康超时使用单调时钟，不受ROS时间跳变影响。

断线时停止该脚原始发布，并报告失效，不能把“没有数据”解释成零压力或离地。另一脚继续独立采集。诊断消息也应一起录包，后续只在新鲜合法观测区间生成标签。

## 厂家帧与实现边界

```text
AA | side(1B) | point_1..18 (各uint16大端) | sum8
side: 01=左脚，02=右脚
sum8 = 前38字节之和 & 0xFF（包含AA和side）
```

固定39字节，滑动查找并校验，支持拆帧/粘帧、坏帧重同步。部分帧在100 ms无新字节后丢弃；该诊断容差不是模块采样周期。

此前STM32双UART足底驱动、USB 0x30桥接、ros2_control足底sensor及旧broadcaster已经撤回。新的读取程序不改变四电机/双踝0x15协议，不提供CPG相位校准或步态事件控制。原始厂家文件已按要求删除，保留提取后的使用说明；代码回退备份仍在`docs/传感器与通信/历史资料/足底压力旧桥接代码备份_2026年10月20日.tar.gz`。

## 本次验证

足底读取9项标准库测试通过（含38种拆帧位置），覆盖厂家例帧、大端边界、粘帧、坏校验、错脚、字节丢失、部分帧超时、ROS消息序列化；双伪串口覆盖独立采集、陈旧/断线状态、无发送字节、稳定路径重连、本地计数延续及新样本发布。原硬件接口29项测试通过。

另在独立本机DDS域中运行真实ROS发布/订阅闭环：双伪串口数据、原始值与时间戳、超时后无缓存重发、单脚断线和另一脚持续接收均通过；安装后的消息类型及launch参数入口已验证。

构建与这些测试不等于实物验收。实际电平、模块脚别/频率、18点位置、空载/加载与连续录包仍需现场确认。
