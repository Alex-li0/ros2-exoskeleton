# CPG 基础路径与多关节在线学习

`legs_cpg` 目前仍是共享 Hopf 振子 + 六关节 Fourier 读出的离线轨迹生成器。
新增的 `adaptive_cpg_shadow` 在独立路径中实现相位、基频和六关节形状学习；默认关闭。
默认 `flat_walk` 仍走原有离线路径；地形占位模板尚不是已验证轨迹。
实机主动关节为左右髋、膝；踝关节参考不等于踝关节执行器控制。

## 修复的行为

- 谐波基底使用固定容量数组，支持参数 `num_harmonics=2..4`（包括 DC 槽），启动时校验权重长度、数值和关节范围。原来的 `vector{5, 0.0}` 只有两个元素，二、三次谐波索引越界。
- 参考位置和速度一起计算：包含 Fourier 基底、切换权重、幅值、偏置和启停包络的时间导数。限幅时速度采用实际边界速度；固定边界上为零。硬限幅拐点仍不可微，不能据此声称加速度连续。
- `publish_ankle=false` 时踝关节位置、速度和显示输出均为零。
- 启停采用五次过渡；启动中停止保留当前包络及其导数，停止期间忽略 `start`。进入 `IDLE` 后可重新启动。零参考不代表阻抗控制器零电流。
- 只在 `WALKING` 接受平滑模式切换；`IDLE` 切换立即更新模式和基频。启停/切换期间新模式请求忽略，不排队，调度层应在稳定后重新提交。停止不打断已开始的权重过渡，也不会被过渡完成事件重新置为行走。
- 模式权重和关节限幅同步插值；运行参数是模板副本，原始 gait library 保留为先验。
- 连续参数目标和超时回退采用平滑过渡，完整计算参数变化带来的速度。正频率/幅值比例在对数域插值以避免频繁改目标造成符号反转；偏置直接插值。过渡并非幅值/速度安全限制，实机仍需经标定的目标范围及控制器限制。
- 连续参数目标拒绝重复、乱序、过期、未来时间戳及重复关节名；有效期按消息时间计算。默认 `parameter_target_timeout_s=0`，仍不接受连续目标。

## 原路径的观测与速率接口

`include/cpg_controller/learning_interface.hpp` 定义两类数据，不直接驱动控制：

| 接口 | 内容 | 当前行为 |
|---|---|---|
| `MeasurementInput` / `MeasurementFrame` | 消息时间、六关节位置、逐关节有效掩码 | 校验并缓存；缺少的关节不补零为有效观测 |
| `LearningRates` | 公共相位修正率 rad/s、基频变化率 rad/s²、每关节 DC 和 1–3 阶 Fourier 权重变化率 | 全零默认，仅定义契约；未连接积分器 |
| `Coefficients` / `Sample` | 参数数值和时间导数 | 已用于参考全导数，后续可承接学习产生的参数变化率 |

设置 `observe_measurements=true` 才订阅 `/cpg/measured_joint_states`，仅用于缓存；
`measurement_timeout_s` 默认 0.1 s。该输入与显示 `/joint_states`、参考 `/cpg/joint_ref` 分离，
若重映射后与本节点这两个输出同名则拒绝启动。接入时只将这个独立输入映射到可信硬件 JSB，
不要全局重映射输入和显示话题。有效消息需有正的、递增且不晚于当前 ROS 时间的时间戳；
已过期样本、重复名称及无任何有效目标关节的消息拒绝接收。

后续学习模块需要消费 `frame().stamp` 一次且仅一次，使用 `fresh()` 检查新鲜度；`legs_cpg` 没有学习消费器；
历史对齐、事件订阅和学习诊断由下文独立的 shadow 核心/节点提供。不能把打开观测入口解释为已经锁定人体相位。
`eta_w` / `eta_omega` 必须为零；非零配置直接报错，替代原先“以自身输出作目标”的无效学习。

事件接入沿用 `exo_state_estimator/GaitEvent`，后续必须按事件发生时间查历史相位；
地形模式仍走既有调度链，不旁路调度器。已有 `CPGParameterTarget.phase_rate_correction`
仍保留未应用。本次学习器输出独立 shadow reference；接入阻抗参考源选择仍需单独验证。

## 已实现的在线学习路径

`include/cpg_controller/adaptive_cpg.hpp` 是节点和离线实验共同使用的 C++ 核心，
不是另一份 Python 算法。六关节共享一个相位；各关节的 DC 与 1–3 阶正余弦系数分别学习。
不同系数表达左右反相、膝屈曲时序和踝波形，不对六关节角度误差直接求平均。

- 设 `e = q_meas - q_hat`，各关节相位导数为 `D = ∂F/∂ψ`，以
  `z = Dᵀ R e / (ε + Dᵀ R D)` 提取公共相位误差；观测平坦、缺失或异常时门控。
- 基频采用有界 PI 锁相更新；实际推进速率由基频、连续纠偏和事件纠偏共同决定，
  经过正速率边界及变化率限制。Hopf 初始化在 `mu=1,r=1` 的稳定极限环上，
  使用其切向流 `dx=-v*y, dy=v*x`，按实际速率积分共享相位，再由归一化基函数读出。
  这样不会把振子半径启动暂态学进 Fourier 系数。
- `CAPTURE` 先同步，`LOCKED` 后开放慢速波形梯度；锁定进入/退出采用迟滞和持续时间，
  避免周期内一次小残差反复使形状学习失效。系数带原始模板正则、信赖域、速率限制和速率平滑。
- 角度与事件均按发生时间查有界历史，包含当时的相位和系数；重复时间戳不重复学习。
  观测超时进入 `LOST`，停止新的梯度/频率更新，已有参数速率平滑衰减到零，短时可能继续产生有限的系数变化。
- 可选 `exo_state_estimator/GaitEvent.INITIAL_CONTACT` 按脚分别校验 ID、时间和置信度；
  同脚事件估计完整周期，事件残差只通过有界推进速率消耗，不直接重置已锁定相位。
  `WAITING/LOST` 首次捕获允许粗搜索及局部细化，此时参考尚不可作为有效控制参考；重捕获可能发生相位拼接。
- 速度含 `W·B'(ψ)·v + Wdot·B(ψ)`。原始重构记录在诊断中；`joint_ref_shadow` 按原模板限幅，
  受限时速度置为实际固定边界速度零。

当前按模板 RMS 与 0.06 rad 下限生成观测尺度，学习增益、系数范围、事件边界均为合成实验初值，
没有人体数据标定。只覆盖持续平地节律的学习；尚未实现自适应模式切换、楼梯转换步、
参考参与度渐入/渐出、输出饱和向学习端反馈，以及阻抗控制器在 `LOST` 后降低辅助的联动。
因此 `LOCKED` 是算法内部状态，并不是实机参考放行条件。

### ROS 输入输出

默认命名空间 `/cpg_adaptive`：

| 话题 | 类型 | 用途 |
|---|---|---|
| `measured_joint_states` | `sensor_msgs/JointState` | 六关节测量，名称匹配和时间戳校验 |
| `gait_events` | `exo_state_estimator/GaitEvent` | 可选事件，默认关闭 |
| `joint_ref_shadow` | `sensor_msgs/JointState` | 六关节位置和全导数，原模板限幅后输出 |
| `learning_debug` | `std_msgs/Float64MultiArray` | 原始重构、学习状态、速率、权重及限幅标志 |

诊断固定 74 项：`[0]` 时间，`[1]` 学习开关，`[2]` 状态（0 WAITING、1 CAPTURE、2 LOCKED、3 LOST），
`[3:10]` 展开相位/基频/实际速率/局部相位误差/残差 RMS/测量年龄/形状学习开关，
`[10:14]` 测量收到与接受计数、事件收到与接受计数，
`[14:20]` 原始六关节重构位置，`[20:26]` 原始速度，`[26:32]` 限幅标志，
`[32:74]` 按六关节顺序排列的 `offset,x1,y1,x2,y2,x3,y3`。
状态需与参考一起使用；`WAITING/CAPTURE/LOST` 时的重构仅供诊断。

独立节点从已有 `cpg.yaml` 导入原始 `flat_walk`，显式启用后才消费测量：

```bash
ros2 launch cpg_controller adaptive_cpg_shadow.launch.py enable_learning:=true
```

事件原点需要与模板统一标定。配置中的 `[0, pi]` 仅用于本次合成事件，
不能当作实际左右脚 IC 的生理零点。新节点不发布 `/cpg/joint_ref`，生产启动文件也未引入它。

### 平地模板变形实验

```bash
/usr/bin/python3 src/cpg_controller/scripts/evaluate_adaptive_learning.py
```

脚本编译或调用同一个 C++ 核心，以 500 Hz 积分、100 Hz 测量运行六组 50 s 场景：
原样、公共相移、频率 +20% 阶跃、六关节形状变形、频率斜坡，以及含噪声/延迟/丢帧/单关节异常的组合。
形状变形包括不同幅值、偏置、局部相移及高阶谐波变化。真实相位、频率与变形参数只用于生成测量和评分，
不送入学习核心；初始系数始终来自未变形的原始模板。

固定模板对照同样使用归一化核心并保持原始参数，代表原始参数轨迹，不是旧节点启动暂态的逐点复刻。
每组对比固定模板、仅节律学习、完整学习和无事件学习，输出
`/tmp/cpg_experiments/adaptive_learning/README.md`、`metrics.json`、输入/输出 CSV 和六关节曲线。
指标区分原始重构、原模板限幅后的参考、基频与实际推进速率，以及 LOCKED 时与全程的速度差分。
没有事件时，系数与相位存在共同旋转的自由度；角度误差更小不代表生理相位更准确。
这些结果属于模型内合成验证。

也可经 ROS 话题实际喂入同一变形测量，演示入口会同时启动 shadow 节点和合成发布器，
显式打开学习与合成事件，50 s 后自动退出：

```bash
ros2 launch cpg_controller adaptive_learning_demo.launch.py case:=combined duration:=50 output:=/tmp/adaptive_ros_trace.csv
```

`case:=shape_deformation` 可单独观察形状变化。`scripts/replay_adaptive_measurements.py`
以 100 Hz 发布测量，可通过 `--output` 记录真实值、原始重构和限幅参考；真实值只留在发布器日志中。
该入口只含两个实验节点，不启动电机、阻抗控制器或生产 CPG。

### 门控与共同学习的对照

`shape_requires_lock=true` 保留当前门控：新鲜、有运动且可观的输入，还需满足
`LOCKED` 和 `|phase_error|<lock_error` 才开放形状更新。
`false` 移除后两个条件，偏置、谐波系数可在有效捕获阶段与公共相位/频率共同学习。
两种策略均保留相同的时间戳、异常值、参数信赖域、参数速率和事件限制。
这比较的是本项目多关节模型的更新政策，不是逐式复现 TRO 的单髋 AO。

```bash
/usr/bin/python3 src/cpg_controller/scripts/compare_learning_strategies.py
ros2 launch cpg_controller adaptive_learning_demo.launch.py shape_requires_lock:=false
```

对照默认输出位于 `/tmp/cpg_experiments/learning_strategy_comparison`。十种场景包括前述六种输入，
以及运行中的平滑相位变化、固定测量偏置、幅值阶跃和启动即存在的形状失配。
实验脚本保留变化开始后 3/5 s、最后 8 s、逐关节误差及恢复时间等原始统计；
当前研究报告另外从原始 CSV 重算起点后 1 s、持续变化过程末 1 s、最后 2 s 与过程峰值，
分别报告关节角度和公共相位误差，角度统一为度。两处统计窗口不同，不能混用。
启动/失锁恢复时含模板相位搜索；原探针行是观测处理后的模型输出，不能据此认为首次观测前没有误差。
偏置拟合同时对比偏移后的测量与未偏移的潜在轨迹，防止把传感器零偏拟合当作意图识别。
共同学习不必然在所有场景胜出；本次使用相同增益做受控比较，没有分别优化两种方法的增益。

## 回归验证

在已 source ROS 2 与工作空间依赖的终端：

```bash
colcon build --packages-select cpg_controller --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select cpg_controller --ctest-args -R 'test_reference_math|test_cpg_node|test_adaptive_cpg|test_adaptive_node' --output-on-failure
colcon test-result --test-result-base build/cpg_controller/test_results --verbose
```

如果本机用户目录安装的 pytest 插件与系统 pytest 冲突，可在构建和测试命令前加
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，无需修改系统 Python。

数学测试覆盖延迟历史、同脚事件周期、多关节纠偏和系数变化全导数，以及第三阶谐波、全导数数值差分、限幅边界速度、过渡拼接和测量有效性。
节点测试使用独立 ROS domain、随机前缀重映射全部控制话题、临时日志目录，
不启动硬件或控制器。覆盖启停、重复切换、停止与切换并发、参数回放/超时、踝输出和非法配置。
这些测试不构成人体佩戴、实时调度或硬件安全验证。

## 实验产物的位置

离线评估、策略对比和误差审计默认写入系统临时目录下的 `cpg_experiments/`，本机通常为 `/tmp/cpg_experiments/`；可用 `--output` 自行指定。根目录运行产物已按用户要求删除，原完整合成CSV和审计重跑数据不再保留；报告工程中的汇总和代表性样本仍可阅读，重新评分须先重跑实验。临时目录可能被清理，真人记录或需长期保存的结果应指定持久位置。
