import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# ==========================================
# 1. 系统核心参数配置 (抛弃振幅学习，固定目标振幅)
# ==========================================
gamma = 1.0    # 极限环收敛速度
epsilon = 3.0  # 频率对外部信号的耦合强度
mu = 1.0       # 固定的目标振幅平方 (替代了之前的 alpha)

# ==========================================
# 2. 外部复合扰动信号参数
# ==========================================
A1, w1 = 1.0, 4.0   # 目标频率 (靠近初始频率)
A2, w2 = 1.0, 18.0  # 高频干扰 (距离初始频率极远)

# ==========================================
# 3. 定义分段输入函数 (前10秒为0，第10秒引入复合干扰)
# ==========================================
def F(t):
    if t < 10.0:
        return 0.0
    else:
        return A1 * np.sin(w1 * t) + A2 * np.sin(w2 * t)

# 仅包含目标分量的信号（用于对比参考）
def F_target(t):
    if t < 10.0:
        return 0.0
    else:
        return A1 * np.sin(w1 * t)

# ==========================================
# 4. 简化的开环 Hopf CPG 动力学方程 (仅含频率学习)
# ==========================================
def cpg_dynamics_fixed_amp(state, t):
    x, y, omega = state
    r = np.sqrt(x**2 + y**2)
    if r == 0: r = 0.0001 # 避免奇异点
    
    ft = F(t) 
    
    # 移除了闭环误差 e(t)，直接使用外部输入 ft
    # 移除了 alpha 的学习方程，使用固定常数 mu
    dxdt = gamma * (mu - r**2) * x - omega * y + epsilon * ft
    dydt = gamma * (mu - r**2) * y + omega * x
    domegadt = -epsilon * ft * (y / r)
    
    return [dxdt, dydt, domegadt]

# ==========================================
# 5. 初始状态与时间设置
# ==========================================
# 初始状态: x=1.0, y=0.0, 初始频率 omega=5.0
state0 = [1.0, 0.0, 5.0] 
t = np.linspace(0, 40, 5000) 

# 求解微分方程
sol = odeint(cpg_dynamics_fixed_amp, state0, t, hmax=0.01)

x_out = sol[:, 0]
omega_out = sol[:, 2]

input_signal = np.array([F(ti) for ti in t])
target_signal = np.array([F_target(ti) for ti in t])

# ==========================================
# 6. 绘图与可视化
# ==========================================
fig, axs = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

# --- 子图 1: 外部复合输入信号 F(t) ---
axs[0].plot(t, input_signal, 'k-', linewidth=1)
axs[0].axvline(x=10, color='r', linestyle='--', alpha=0.7)
axs[0].set_title('Subplot 1: Composite Input Signal $F(t)$', fontsize=14, fontweight='bold')
axs[0].set_ylabel('Amplitude', fontsize=12)
axs[0].grid(True)

# --- 子图 2: CPG 内部频率学习演化 (核心看点) ---
axs[1].plot(t, omega_out, 'g-', linewidth=3, label='CPG Internal Frequency $\omega(t)$')
axs[1].axhline(y=w1, color='purple', linestyle='--', linewidth=2, label='Target $\omega_1$ (4.0 rad/s)')
axs[1].axhline(y=w2, color='orange', linestyle='--', linewidth=2, label='Noise $\omega_2$ (18.0 rad/s)')
axs[1].axvline(x=10, color='r', linestyle='--', alpha=0.7)
axs[1].set_title('Subplot 2: Pure Frequency Selective Entrainment', fontsize=14, fontweight='bold')
axs[1].set_ylabel('Frequency (rad/s)', fontsize=12)
axs[1].legend(loc='center right', fontsize=11)
axs[1].grid(True)

# --- 子图 3: CPG 位置输出 x(t) ---
axs[2].plot(t, target_signal, 'gray', linestyle='--', linewidth=2, alpha=0.6, label='Isolated Target Component')
axs[2].plot(t, x_out, 'b-', linewidth=2, label='CPG Output $x(t)$')
axs[2].axvline(x=10, color='r', linestyle='--', alpha=0.7)
axs[2].set_title('Subplot 3: Output Trajectory (Stable amplitude without divergence)', fontsize=14, fontweight='bold')
axs[2].set_xlabel('Time (s)', fontsize=12)
axs[2].set_ylabel('Position $x$', fontsize=12)
axs[2].legend(loc='upper right', fontsize=11)
axs[2].grid(True)

plt.tight_layout()
plt.savefig('muti_signal.png', dpi=300, bbox_inches='tight')
plt.show()