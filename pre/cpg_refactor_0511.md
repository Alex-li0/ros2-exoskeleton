# CPG 改造清单

> 基于架构C设计文档，针对 `cpg_controller` 包的总体规划。
> 最后更新：2026-05-11

---

## 设计原则

1. **CPG 始终是轨迹生成器，NMPC 始终是修正器**。区别只在于修正量的大小——平地时 NMPC 修正量小，复杂地形时修正量大。
2. **权重切换是离散的、以步态周期为边界的**。步态是周期性的，每一步就是一个完整的运动周期。从平地到上台阶的过渡发生在步与步之间（摆动腿落地瞬间），不是步态内部的连续变化。不需要 alpha 插值。
3. **在线学习包括频率自适应和振幅自适应**（基于 Hopf 振荡器的 Hebbian 学习机制，见 datasheet/Hopf/cpg.tex）。
4. **Hebbian 权重学习暂时关闭**（`eta_w = 0`），没有上机实验数据，待上机后再激活。
5. **NMPC 检测到地形变化后，在当前步结束时通知 CPG 切换 gait mode**。

---

## 优先级总览

| 优先级 | 含义 | 预计工作量 |
|--------|------|-----------|
| **P0** | 安全基础，必须先做 | 1-2 天 |
| **P1** | CPG 核心架构改造 | 3-5 天 |
| **P2** | 扩展性改造 | 2-3 天 |
| **P3** | 在线自适应学习机制 | 3-5 天 |
| **P4** | 接口对接（与 NMPC/Terrain 通信） | 2-3 天 |
| **P5** | NMPC 侧和数据准备 | 2-4 周 |

---

## P0：启停状态机 + 振幅调制

**目标：** 让 CPG 能安全地启动和停止，而不是永远在极限环上运转。

**改动范围：** `legs_cpg.cpp`, `cpg.yaml`

**核心内容：**
- 新增 CPGState 状态机：`IDLE → STARTING → WALKING → STOPPING → IDLE`
- 新增振幅因子 `A(t) ∈ [0, 1]`，关节输出乘以 A
  - IDLE：A = 0，输出零
  - STARTING：A 从 0 平滑上升到 1（渐入）
  - WALKING：A = 1，全振幅
  - STOPPING：A 从 1 平滑下降到 0（渐出）
- 新增话题：订阅 `/cpg/state_cmd`，发布 `/cpg/state`
- YAML 新增参数：`ramp_time`（渐入渐出时间，默认 0.5s）

**验收标准：** 发布 stop 后输出平滑归零，发布 start 后输出平滑恢复。

---

## P1：多 Gait Mode 权重（Gait Library）

**目标：** CPG 支持多种地形模式的权重配置，每种模式的左右腿有独立的完整权重。

**改动范围：** `cpg.yaml`, `legs_cpg.cpp`

**核心内容：**
- YAML 从当前的扁平结构改为 `gait_library` 嵌套结构
- 支持的 gait mode：
  - `flat_walk`：平地行走（当前已有权重，左右对称）
  - `stair_up`：上楼梯（摆动腿大屈曲，支撑腿小角度）
  - `stair_down`：下楼梯
  - `slope_up`：上坡
  - `slope_down`：下坡
- 每种 mode 下每个关节可独立配置 `num_harmonics`、`w_offset`、`w_x`、`w_y`、`limit_min`、`limit_max`
- C++ 端数据结构改为 `map<string, GaitMode>`，每个 GaitMode 包含 `map<string, JointModel>`

**数据来源：** 每种 gait mode 的权重需要通过 `cpg_optimizer.py` 用对应地形的真实人体步态数据 OLS 拟合得到。

**验收标准：** 能在运行时切换 gait mode，输出的关节轨迹符合对应地形的步态特征。

---

## P2：去掉镜像，左右腿独立权重

**目标：** 去掉当前的 sign 镜像逻辑，让左右腿拥有完全独立的权重。

**改动范围：** `legs_cpg.cpp`

**核心内容：**
- 删除 `sign = (is_right && i % 2 != 0) ? -1.0 : 1.0` 逻辑
- 每个关节直接用自己的权重计算：`θ = w_offset + Σ(w_x[i]·hx[i] + w_y[i]·hy[i])`
- 在 `flat_walk` 模式下，左右腿权重通过 OLS 数据保证自然对称
- 在 `stair_up` 模式下，左右腿权重自然不对称（摆动腿 vs 支撑腿）

**验收标准：** flat_walk 下左右腿轨迹仍然对称；stair_up 下左右腿轨迹明显不同。

---

## P3：谐波阶数自适应

**目标：** 不同 gait mode 的不同关节可以使用不同数量的谐波。

**改动范围：** `legs_cpg.cpp`

**核心内容：**
- 每种 gait mode 的每个关节已有独立的 `num_harmonics`（P1 中定义）
- 运行时取当前 mode 中所有关节的最大谐波数作为实际计算阶数
- 膝关节在上楼梯时可能需要 4-5 阶（波形更复杂），平地只需 2-3 阶
- 踝关节在上楼梯时也需要提升到 3 阶

**验收标准：** 切换到 stair_up 模式后，膝关节输出波形的拟合精度明显提高。

---

## P4：在线频率学习（Hopf 自适应机制）

**目标：** 通过外部物理反馈自动调整步频 ω，使 CPG 自适应环境。

**改动范围：** `legs_cpg.cpp`, `cpg.yaml`

**核心内容（基于 datasheet/Hopf/cpg.tex 的理论推导）：**
- 频率更新方程：`ω̇ = -ε · F(t) · y/r`
  - F(t)：外部反馈信号（关节力矩误差或地面反力）
  - ε：耦合强度，决定学习速度
  - y/r：CPG 当前相位的正弦分量
- 串联一阶低通滤波器（LPF）平滑频率：`τ · ω̇_smooth + ω_smooth = ω_raw`
  - 消除稳态高频振荡（ε 过大时的副作用）
  - τ 是滤波器时间常数，与 ε 需联合整定
- F(t) 的来源：
  - 短期（仿真）：NMPC 输出力矩与 CPG 参考力矩的差
  - 长期（上机）：力传感器的地面反力

**关键参数：**
- `epsilon`：耦合强度（建议初始值 0.1-0.5）
- `tau_lpf`：LPF 时间常数（建议初始值 0.5-1.0s）

**验收标准：** 改变外部激励频率后，CPG 的 ω 能在数个步态周期内收敛到新频率。

---

## P5：在线振幅学习

**目标：** 通过外部物理反馈自动调整振幅 μ，使 CPG 输出幅度匹配实际环境。

**改动范围：** `legs_cpg.cpp`, `cpg.yaml`

**核心内容（基于 datasheet/Hopf/cpg.tex 的振幅自适应推导）：**
- 振幅更新方程：`α̇ = η · F(t) · x/r`
  - η：振幅学习率
  - x：CPG 当前位置（与频率学习的 y 正交）
- 限制最小振幅：`μ = max(μ_min, α)`
- 同样串联 LPF 平滑

**验收标准：** 负载变化时，CPG 的输出幅度能自动调整到合适范围。

---

## P6：接口对接

**目标：** CPG 与 NMPC、Terrain Extractor 之间的 ROS 2 话题连接。

**改动范围：** `legs_cpg.cpp`, `CMakeLists.txt`, `package.xml`

**新增订阅：**
| 话题 | 消息类型 | 发布方 | 用途 |
|------|---------|--------|------|
| `/cpg/state_cmd` | `std_msgs/String` | 上层控制器 | 启停指令 |
| `/terrain/gait_mode` | `std_msgs/String` | NMPC / Terrain | 切换 gait mode |
| `/joint_torques` | `std_msgs/Float64MultiArray` | NMPC | 力矩反馈（供频率/振幅学习） |

**新增发布：**
| 话题 | 消息类型 | 订阅方 | 用途 |
|------|---------|--------|------|
| `/cpg/state` | `std_msgs/String` | 安全监控 | 当前 CPG 状态 |
| `/cpg/joint_ref` | `sensor_msgs/JointState` | NMPC | CPG 参考轨迹 |
| `/cpg/gait_mode` | `std_msgs/String` | NMPC | 当前 gait mode |

**CMakeLists.txt 新增依赖：** `std_msgs`

**验收标准：** 话题全部连通，NMPC 能订阅到 CPG 参考轨迹。

---

## P7：NMPC 侧和数据准备

**目标：** 完成 NMPC 优化器和地形特征提取器，为每种 gait mode 准备 OLS 权重。

**改动范围：** 新建 ROS 2 包，改造 `cpg_optimizer.py`

**子任务：**
- [ ] 改造 `cpg_optimizer.py`：为每种地形模式（flat_walk / stair_up / stair_down / slope_up / slope_down）分别准备参考步态数据，分别 OLS 拟合权重，输出为 gait_library YAML 格式
- [ ] 新建 `terrain_extractor` 包：从 GridMap 提取坡度、高度剖面、落脚点、粗糙度
- [ ] 新建 `gait_optimizer` 包（Python + CasADi）：实现 NMPC 优化器，订阅 CPG 参考轨迹和地形约束，输出优化后的关节轨迹

**验收标准：** 完整的 CPG → NMPC → 地形数据链路在 RViz 中可视化。

---

## 现有代码状态

| 文件 | 现状 | 说明 |
|------|------|------|
| `legs_cpg.cpp` | 基础框架可用 | Hopf 振荡器 + 傅里叶映射 + 子步长积分，但缺少启停、多模式、左右独立 |
| `single_leg_cpg.cpp` | 单腿原型 | 用于早期调试，后续以 legs_cpg 为主 |
| `cpg_optimizer.py` | OLS 拟合工具 | 当前只拟合平地步态，需扩展为多地形 |
| `cpg.yaml` | 单一 gait mode | 需改为 gait_library 结构 |
| `single_cpg_params.yaml` | 单腿参数 | 暂不改动 |
| `datasheet/Hopf/cpg.tex` | 理论文档 | Hopf 自适应学习的完整理论推导，P4/P5 的理论依据 |

---

## 执行顺序

```
P0 启停状态机
  ↓
P1 gait_library + P2 去镜像 + P3 谐波自适应（可并行）
  ↓
P6 接口对接
  ↓
P4 频率学习 + P5 振幅学习（可并行，需要 P6 的力矩反馈接口）
  ↓
P7 NMPC + 数据准备（可与 P4/P5 并行推进）