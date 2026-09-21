# CPG/虚拟管道多关节调参工具

该包在现有 `legs_cpg -> ExoImpedanceController -> hardware` 链路上提供Qt调参界面。
四个关节各有一个可按下的输出按钮。按下哪些按钮，哪些关节就使用各自的 Kp/Kd 和虚拟管道宽度；
未按下的关节强制为零。GUI心跳超过控制器配置的 `tuning_command_timeout` 后，四关节全部平滑归零。

“当前位置设为上位机零位”会先停止CPG并关闭四关节调参输出，等控制器的四路`iq`
命令都降为零后，再由上位机ROS 2硬件接口插件把当时的四个编码器位置保存为软件
偏置。它不向STM32或电机发送校零指令，也不改写下位机参数。执行时机构必须固定且保持
不动，并应处于与URDF零姿态一致的机械姿态。

## 构建与启动

```bash
cd /home/lsy/vins_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to impedance_tuning exoskeleton_controllers exoskeleton_hardware
source install/setup.bash
ros2 launch impedance_tuning impedance_tuning.launch.py
```

启动后CPG处于停止状态，四个关节的调参输出默认关闭。推荐顺序为：选择
关节；填写较低Kp/Kd及管道半宽并点击“应用”；按下需要联调的一个或多个关节按钮；确认支架和急停；
点击“启动CPG轨迹”。再次点击某个关节只会停止该关节，关闭窗口、停止CPG或GUI心跳丢失会
停止全部输出。每次改变增益会重新开始当前周期统计，
完成一个CPG周期后界面显示RMS、峰值、NRMSE、相位滞后、命令iq、饱和比例、控制
周期P99和参考龄期P99。“保存 CSV + 周期报告”输出逐样本CSV和逐周期JSON；“保存阻抗/管道参数
到 YAML”只替换当前控制器配置中的四个参数数组并保留其余注释。运行中的值
立即来自GUI，下次启动从保存后的YAML读取。

`/impedance/debug`沿用既有控制器字段约定：`position=q_ref`、`velocity=q_measured`、
`effort=iq_command`。实测电流来自`/dynamic_joint_states`的`effort`接口。这里的输出单位
仍是电机`iq`，未做物理力矩换算。

普通控制栈不发送`/impedance_tuning/command`时，控制器保持原有四关节行为。调参GUI
一旦发送过命令，该控制器实例就保持调参安全模式；退出调参后若要恢复普通四关节控制，
应停止当前launch并重新启动正常控制栈。

## ROS信息流与单关节隔离

```text
legs_cpg
  ├─ /cpg/joint_ref (各关节参考位置、速度) ──> ExoImpedanceController
  └─ /cpg/debug (CPG相位) ───────────────────> 调参GUI周期分段

调参GUI
  └─ /impedance_tuning/command
       (关节启用位掩码、四组Kp/Kd/管道宽度、清零序号) ──> ExoImpedanceController

未来人体状态/相位模块
  └─ /impedance/auxiliary_command
       (透明度补偿iq、相位助力iq，分别带启用位) ───> ExoImpedanceController

ExoImpedanceController (500 Hz)
  ├─ 读取编码器位置/速度状态接口
  ├─ 管道内: iq_tube = 0
  ├─ 管道外: iq_tube = 平滑激活*(Kp*越界误差 + Kd*速度误差)
  ├─ iq_cmd = iq_tube + iq_transparency + iq_assist + iq_ff
  ├─ 未按下关节: iq目标置零，并经过变化率限制降到零
  ├─ 全部输出归零后: 可向硬件发送一次软件清零脉冲
  └─ effort命令接口 ──> ExoHardwareInterface ──> STM32 ──> CAN电机

编码器/速度/实测iq ──> ExoHardwareInterface
  ├─ /joint_states、/dynamic_joint_states
  └─ 控制器生成 /impedance/debug、/impedance/timing ──> 调参GUI实时与逐周期分析
```

“停止全部输出”指上位机把四个关节的目标`iq`降为零，不表示电机断电、抱闸闭合或机构
进入完全被动状态。关节输出选择发生在阻抗控制器内部，而不是通过隐藏曲线、断开话题或
停止另外三个电机实现。GUI以20 Hz发送调参心跳；心跳超时后控制器对四个关节都采用
零`iq`目标。

## 控制律与预留接口

设 `e=q_ref-q`、管道半宽 `b>=0`、边界外平滑宽度 `s>=0`。当 `b=0`时直接令
`e_w=e,a=1`，与原阻抗律完全等价。当 `b>0`时，越界误差为
`e_w=sign(e)*max(|e|-b,0)`，平滑激活量 `a` 在管道内为0；当 `s=0` 时越界即为1；
否则对 `x=clamp((|e|-b)/s,0,1)` 使用 `a=x²(3-2x)`。最终：

```text
iq_tube = a * (Kp*e_w + Kd*(qdot_ref-qdot))
iq_cmd  = iq_tube + iq_transparency + iq_assist + iq_ff
```

`default_tube_half_width=0` 且 `default_tube_transition_width=0` 时与原阻抗反馈等价。
`/impedance/auxiliary_command` 是版本1数组：
`[1, transparency_enabled, assist_enabled, transparency_iq[4], assist_iq[4]]`。
当前不提供该话题的发布者，因此透明度和助力均为0；日后可由人体状态估计器以实时
`iq`发布，超时自动归零。不在这里预设膝关节自然下落等相位窗口，避免在没有人机数据时填入任意助力曲线。
`/impedance/components` 输出
`[tube_iq[4], transparency_iq[4], assist_iq[4], actual_command_iq[4]]`，最后一组是变化率和幅值限制后的实际命令，用于后续分量验证。
