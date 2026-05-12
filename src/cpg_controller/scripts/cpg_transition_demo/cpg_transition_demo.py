"""
CPG 启停过渡可视化
模拟 IDLE → STARTING → WALKING → STOPPING → IDLE 全过程
对比三个关节（髋/膝/踝）在过渡态与稳态行走的曲线差异
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

# ==================== CPG 参数（来自 cpg.yaml） ====================
mu = 1.0
gamma = 15.0
omega = 5.720866
ramp_time = 0.5
perturbation = 0.1

# 关节谐波模型（左腿）
joints = {
    "Hip": {
        "w_offset": 0.1759,
        "w_x": [0.0, 0.2142],
        "w_y": [0.0, 0.1125],
        "n_harm": 2,
    },
    "Knee": {
        "w_offset": -0.3062,
        "w_x": [0.0, -0.3893, -0.0878],
        "w_y": [0.0, 0.1008, 0.1350],
        "n_harm": 3,
    },
    "Ankle": {
        "w_offset": 0.1473,
        "w_x": [0.0, -0.0181, -0.0075, 0.0141],
        "w_y": [0.0, -0.0120, 0.0319, 0.0210],
        "n_harm": 4,
    },
}


def compute_amplitude(state, elapsed, ramp_time, stop_start_amp):
    """半余弦窗振幅因子"""
    T = max(ramp_time, 0.01)
    if state == "IDLE":
        return 0.0
    elif state == "STARTING":
        t = np.clip(elapsed / T, 0.0, 1.0)
        return 0.5 * (1.0 - np.cos(np.pi * t))
    elif state == "WALKING":
        return 1.0
    elif state == "STOPPING":
        t = np.clip(elapsed / T, 0.0, 1.0)
        return stop_start_amp * 0.5 * (1.0 + np.cos(np.pi * t))
    return 0.0


def compute_harmonics(x, y, n_harm):
    """计算高阶谐波"""
    hx = np.zeros(5)
    hy = np.zeros(5)
    hx[1] = x
    hy[1] = y
    if n_harm >= 2:
        hx[2] = x * x - y * y
        hy[2] = 2.0 * x * y
    if n_harm >= 3:
        hx[3] = x * (x * x - 3.0 * y * y)
        hy[3] = y * (3.0 * x * x - y * y)
    return hx, hy


def simulate_cpg(duration, dt=0.001):
    """模拟完整 CPG 生命周期"""
    steps = int(duration / dt)
    times = np.arange(steps) * dt

    # 状态机时间线
    # 0-1s: IDLE, 1-1.5s: STARTING, 1.5-4s: WALKING, 4-4.5s: STOPPING, 4.5-5.5s: IDLE
    t_start_cmd = 1.0
    t_stop_cmd = 4.0

    # 状态数组
    states = []
    for t in times:
        if t < t_start_cmd:
            states.append("IDLE")
        elif t < t_start_cmd + ramp_time:
            states.append("STARTING")
        elif t < t_stop_cmd:
            states.append("WALKING")
        elif t < t_stop_cmd + ramp_time:
            states.append("STOPPING")
        else:
            states.append("IDLE")

    # 初始化
    x, y = 0.0, 0.0
    amp = 0.0
    stop_start_amp = 1.0

    # 输出记录
    results = {name: np.zeros(steps) for name in joints}
    amplitudes = np.zeros(steps)
    osc_x = np.zeros(steps)

    for i, (t, state) in enumerate(zip(times, states)):
        # IDLE: 冻结振子
        if state == "IDLE" and (i == 0 or states[i - 1] != "IDLE"):
            x, y = 0.0, 0.0

        # STARTING 开始: 扰动振子
        if state == "STARTING" and i > 0 and states[i - 1] == "IDLE":
            x = np.sqrt(mu) * perturbation
            y = 0.0

        # 振荡器积分（非 IDLE 时）
        if state != "IDLE":
            sub_steps = 10
            dt_sim = dt / sub_steps
            for _ in range(sub_steps):
                r2 = x * x + y * y
                dx = gamma * (mu - r2) * x - omega * y
                dy = gamma * (mu - r2) * y + omega * x
                x += dx * dt_sim
                y += dy * dt_sim

        # 振幅因子
        elapsed = 0.0
        if state == "STARTING":
            elapsed = t - t_start_cmd
        elif state == "STOPPING":
            elapsed = t - t_stop_cmd
        amp = compute_amplitude(state, elapsed, ramp_time, stop_start_amp)

        # STOPPING 开始: 记录当前振幅
        if state == "STOPPING" and i > 0 and states[i - 1] == "WALKING":
            stop_start_amp = amplitudes[i - 1]

        amplitudes[i] = amp
        osc_x[i] = x

        # 计算各关节输出
        hx, hy = compute_harmonics(x, y, 4)
        for name, j in joints.items():
            theta = j["w_offset"]
            for k in range(1, j["n_harm"]):
                theta += j["w_x"][k] * hx[k] + j["w_y"][k] * hy[k]
            results[name][i] = amp * theta

    return times, states, results, amplitudes, osc_x


# ==================== 模拟 ====================
times, states, results, amplitudes, osc_x = simulate_cpg(5.5)

# ==================== 绘图 ====================
fig, axes = plt.subplots(5, 1, figsize=(16, 14), sharex=True)
fig.suptitle("CPG Start / Stop Transition Demo", fontsize=18, fontweight="bold", y=0.98)

colors = {"Hip": "#FF6B6B", "Knee": "#4ECDC4", "Ankle": "#45B7D1"}

# --- 1. 状态机时间线 ---
ax = axes[0]
state_map = {"IDLE": 0, "STARTING": 1, "WALKING": 2, "STOPPING": 1}
state_vals = [state_map[s] for s in states]
state_colors_map = {"IDLE": "#2C3E50", "STARTING": "#E67E22", "WALKING": "#27AE60", "STOPPING": "#E74C3C"}

prev_state = states[0]
start_idx = 0
for i in range(1, len(states)):
    if states[i] != prev_state:
        ax.axvspan(times[start_idx], times[i], alpha=0.3, color=state_colors_map[prev_state], label=prev_state)
        start_idx = i
        prev_state = states[i]
ax.axvspan(times[start_idx], times[-1], alpha=0.3, color=state_colors_map[prev_state], label=prev_state)
ax.set_ylabel("State", fontsize=12)
ax.set_yticks([0, 1, 2])
ax.set_yticklabels(["IDLE", "STARTING/\nSTOPPING", "WALKING"])
ax.legend(loc="upper right", ncol=4, fontsize=10)
ax.set_title("State Machine Timeline", fontsize=13)
ax.grid(True, alpha=0.3)

# --- 2. 振幅因子 A(t) ---
ax = axes[1]
ax.plot(times, amplitudes, color="#8E44AD", linewidth=2.5, label="A(t)")
ax.axhline(y=1.0, color="gray", linestyle="--", alpha=0.5)
ax.axhline(y=0.0, color="gray", linestyle="--", alpha=0.5)
ax.set_ylabel("Amplitude A(t)", fontsize=12)
ax.set_title("Amplitude Factor (Half-Cosine Ramp)", fontsize=13)
ax.set_ylim(-0.1, 1.2)
ax.grid(True, alpha=0.3)
ax.legend(fontsize=10)

# --- 3-5. 三个关节曲线 ---
for idx, (name, color) in enumerate(colors.items()):
    ax = axes[idx + 2]
    ax.plot(times, results[name], color=color, linewidth=2, label=f"{name} (transition)")

    # 稳态行走参考线（从 WALKING 段取一个完整周期）
    walk_start = 1.5
    walk_end = 4.0
    walk_mask = (times >= walk_start) & (times <= walk_end)
    walk_data = results[name][walk_mask]
    walk_times = times[walk_mask]

    # 找一个完整周期
    period = 2 * np.pi / omega
    cycle_mask = walk_times <= walk_start + period
    ax.plot(walk_times[cycle_mask], walk_data[cycle_mask],
            color=color, linewidth=3.5, alpha=0.4, linestyle="-", label=f"{name} (steady)")

    # 标注 w_offset 基线
    ax.axhline(y=joints[name]["w_offset"], color=color, linestyle="--", alpha=0.3, linewidth=1)

    ax.set_ylabel("Angle (rad)", fontsize=12)
    ax.set_title(f"{name} Joint Angle", fontsize=13)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10, loc="upper right")

# --- 标注关键时刻 ---
for ax in axes:
    ax.axvline(x=1.0, color="#E67E22", linestyle=":", alpha=0.7, linewidth=1.5)
    ax.axvline(x=1.5, color="#27AE60", linestyle=":", alpha=0.7, linewidth=1.5)
    ax.axvline(x=4.0, color="#E74C3C", linestyle=":", alpha=0.7, linewidth=1.5)
    ax.axvline(x=4.5, color="#2C3E50", linestyle=":", alpha=0.7, linewidth=1.5)

# 添加时刻标注
axes[0].annotate("start cmd", xy=(1.0, 0.5), fontsize=10, color="#E67E22",
                 ha="center", va="bottom", fontweight="bold")
axes[0].annotate("WALKING", xy=(1.5, 0.5), fontsize=10, color="#27AE60",
                 ha="center", va="bottom", fontweight="bold")
axes[0].annotate("stop cmd", xy=(4.0, 0.5), fontsize=10, color="#E74C3C",
                 ha="center", va="bottom", fontweight="bold")
axes[0].annotate("IDLE", xy=(4.5, 0.5), fontsize=10, color="#2C3E50",
                 ha="center", va="bottom", fontweight="bold")

axes[-1].set_xlabel("Time (s)", fontsize=12)
plt.tight_layout(rect=[0, 0, 1, 0.96])

output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cpg_transition_demo.png")
plt.savefig(output_path, dpi=150, bbox_inches="tight")
plt.savefig(output_path.replace(".png", ".pdf"), bbox_inches="tight")
print(f"Saved to {output_path}")
plt.show()
