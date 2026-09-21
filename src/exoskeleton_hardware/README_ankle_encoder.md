# 踝编码器上位机链路

STM32 `CMD_MOTOR_RAW_FEEDBACK (0x15)` 当前使用 LEN=68。前 60 字节是四电机反馈及诊断，
随后为左、右踝各 4 字节：`node_id(u8), flags(u8), raw_count(u16 LE)`。`flags`
bit0 表示已收到有效数据，bit1 表示数据陈旧。

上位机将踝部作为两个只读 ros2_control sensor，不导出 command interface。

- `/joint_states`：`left_ankle`/`right_ankle` 的零偏和方向修正后角度、角速度，
  供 RViz 和后续算法使用。
- `/dynamic_joint_states`：除 `position`/`velocity` 外，保留 `raw_count`、
  `node_id`、`feedback_valid`和 `stale`，供标定、数据质量筛选与步态事件研究使用。

## 零位和方向标定

1. 用机械支撑固定外骨骼，使左右脚掌保持约定的机械零位。
2. 启动硬件反馈后执行：

   ```bash
   source /home/lsy/vins_ws/install/setup.bash
   ros2 run exoskeleton_hardware calibrate_ankle_zero --samples 500
   ```

3. 将输出的 `zero_offset_counts` 写入
   `config/exo_controllers.yaml` 的 `ankle_encoder_map`，然后重启 launch。
4. 分别只做小幅背屈/跥屈，核对 RViz 方向。方向相反时只把对应
   `joint_sign` 在 `1` 和 `-1` 之间切换；不要同时改 URDF axis 和 sign。

`zero_offset_counts` 使用单圈最短距离修正，因此跨越 4095→0 时不会产生角度跳变。

## 可视化与录包前验收

`exo_mapping_check.launch.py` 会启动 RViz，但它仍会通过硬件接口清错并使能四个电机。
执行前必须确认急停、机械支撑、关节活动范围和人员撤离条件；零目标不等于零力矩。

```bash
source /home/lsy/vins_ws/install/setup.bash
ros2 launch exoskeleton_hardware exo_mapping_check.launch.py
ros2 topic echo /joint_states --once
ros2 topic echo /dynamic_joint_states --once
```

录包前检查现在会要求两个踝关节同时出现在上述两个话题，并核对六个踝部动态接口：

```bash
ros2 run robot_bringup check_recording_topics \
  --config /home/lsy/vins_ws/src/robot_bringup/config/state_estimation_record_topics.yaml
ros2 launch robot_bringup record_state_estimation.launch.py output:=state_estimation_bag
```

后续步态事件识别建议同时使用踝角、踝角速度和 `feedback_valid/stale`，但当前不在
在线控制或状态估计中加入未验证的事件阈值。
