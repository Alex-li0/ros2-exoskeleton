import numpy as np
import matplotlib.pyplot as plt

# 时间数组，高分辨率以捕捉高频细节
t = np.linspace(0, 20, 2000)

# ==========================================
# 场景一：频率相近 (能学习) -> 差频极小(低频)，和频极大(高频)
# 对应: 高频 + 低频
# ==========================================
Omega_sum_1 = 10.0   # 高频项 (和频)
Omega_diff_1 = 0.5   # 低频项 (差频，因为两个频率很接近)

# 导数层信号 (混合信号)
dot_omega_close = np.cos(Omega_sum_1 * t) + np.cos(Omega_diff_1 * t)

# 积分层信号 (各自除以频率)
omega_close = (1 / Omega_sum_1) * np.sin(Omega_sum_1 * t) + (1 / Omega_diff_1) * np.sin(Omega_diff_1 * t)


# ==========================================
# 场景二：频率悬殊 (学不会) -> 差频也很大(高频)，和频极大(高频)
# 对应: 高频 + 高频
# ==========================================
Omega_sum_2 = 10.0   # 高频项 (和频)
Omega_diff_2 = 8.0   # 高频项 (差频，因为两个频率相差极大)

# 导数层信号 (混合信号)
dot_omega_far = np.cos(Omega_sum_2 * t) + np.cos(Omega_diff_2 * t)

# 积分层信号 (各自除以频率)
omega_far = (1 / Omega_sum_2) * np.sin(Omega_sum_2 * t) + (1 / Omega_diff_2) * np.sin(Omega_diff_2 * t)


# ==========================================
# 可视化与对比
# ==========================================
fig, axs = plt.subplots(2, 2, figsize=(15, 8))
fig.suptitle("Integration of Superimposed Signals: Close vs. Far Frequencies", fontsize=16, fontweight='bold')

# --- 第一列：频率相近 (高频 + 低频) ---
# 导数层 (表象)
axs[0, 0].plot(t, dot_omega_close, color='purple', alpha=0.8)
axs[0, 0].set_title("Close Freqs (Derivative Layer)\n$\dot{\omega} = \cos(10t) + \cos(0.5t)$")
axs[0, 0].set_ylim(-2.5, 2.5)
axs[0, 0].grid(True, linestyle=':')

# 积分层 (本质：宏观学习成功)
axs[1, 0].plot(t, omega_close, color='blue', linewidth=2)
axs[1, 0].set_title("Close Freqs (Integration Layer)\n$\omega = 0.1\sin(10t) + 2.0\sin(0.5t)$")
axs[1, 0].set_ylim(-2.5, 2.5)
axs[1, 0].axhline(0, color='black', linewidth=1)
axs[1, 0].grid(True, linestyle=':')
axs[1, 0].annotate('Macro Movement (Learning)', xy=(10, 2), xytext=(10, 1),
             arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=6),
             horizontalalignment='center', color='blue')

# --- 第二列：频率悬殊 (高频 + 高频) ---
# 导数层 (表象)
axs[0, 1].plot(t, dot_omega_far, color='orange', alpha=0.8)
axs[0, 1].set_title("Far Freqs (Derivative Layer)\n$\dot{\omega} = \cos(10t) + \cos(8t)$")
axs[0, 1].set_ylim(-2.5, 2.5)
axs[0, 1].grid(True, linestyle=':')

# 积分层 (本质：宏观学习失败)
axs[1, 1].plot(t, omega_far, color='red', linewidth=2)
axs[1, 1].set_title("Far Freqs (Integration Layer)\n$\omega = 0.1\sin(10t) + 0.125\sin(8t)$")
axs[1, 1].set_ylim(-2.5, 2.5)
axs[1, 1].axhline(0, color='black', linewidth=1)
axs[1, 1].grid(True, linestyle=':')
axs[1, 1].annotate('Micro Jiggle (Phase Slipping)', xy=(10, 0.2), xytext=(10, 1.5),
             arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=6),
             horizontalalignment='center', color='red')

plt.tight_layout()
plt.subplots_adjust(top=0.88)
plt.savefig('figure4_integral_impact.png', dpi=300, bbox_inches='tight')
plt.show()