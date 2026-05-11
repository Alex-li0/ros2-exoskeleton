import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# --- 系统参数 ---
gamma = 5.0
mu = 1.0
epsilon = 1.5       # 耦合强度
target_freq = 2.5   # 外部信号的频率
initial_omega = 1.0 # CPG 初始的固有频率 (故意设置得很低)

def learning_hopf(state, t):
    x, y, omega = state
    r = np.sqrt(x**2 + y**2)
    
    # 设定扰动信号：前10秒没有扰动，第10秒开始引入外部交互力 F(t)
    F_t = np.sin(target_freq * t) if t >= 10 else 0.0
        
    dxdt = gamma * (mu - r**2) * x - omega * y + epsilon * F_t
    dydt = gamma * (mu - r**2) * y + omega * x
    domegadt = -epsilon * F_t * (y / r) if r > 0.01 else 0
    
    return [dxdt, dydt, domegadt]

# --- 求解轨迹 ---
t = np.linspace(0, 30, 3000)
initial_state = [1.0, 0.0, initial_omega]
sol = odeint(learning_hopf, initial_state, t)

x_out = sol[:, 0]
omega_out = sol[:, 2]

# 生成用于对比的外部信号波形 (为了图表美观，幅值做了一点缩放)
F_signal = np.array([np.sin(target_freq * step) if step >= 10 else 0.0 for step in t])

# ==========================================
# 绘制双拼图 (上方看波形同步，下方看频率学习)
# ==========================================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# --- 子图 1：波形与相位同步 ---
ax1.plot(t, x_out, 'b-', linewidth=2, label='CPG Output: x(t)')
ax1.plot(t, F_signal, 'r--', linewidth=2, alpha=0.7, label='External Force: F(t)')
ax1.axvline(10, color='k', linestyle=':', linewidth=2, label='Perturbation Starts (t=10s)')

ax1.set_title('Phase Entrainment Driven by $\epsilon F(t)$', fontsize=14)
ax1.set_ylabel('Amplitude')
ax1.legend(loc='upper left')
ax1.grid(True, linestyle=':', alpha=0.7)

# 添加文字注释
ax1.text(4, -1.5, 'Desynchronized\n(Free Running)', fontsize=11, color='blue', ha='center')
ax1.text(20, -1.5, 'Phase Locked\n(Synchronized)', fontsize=11, color='red', ha='center')

# --- 子图 2：内部频率的学习过程 ---
ax2.plot(t, omega_out, 'g-', linewidth=2.5, label='Internal Frequency: $\omega(t)$')
ax2.axhline(target_freq, color='r', linestyle='--', linewidth=2, alpha=0.7, label=f'Target Frequency ({target_freq} rad/s)')
ax2.axvline(10, color='k', linestyle=':', linewidth=2)

ax2.set_title('Dynamic Frequency Learning', fontsize=14)
ax2.set_xlabel('Time (s)', fontsize=12)
ax2.set_ylabel('Frequency (rad/s)')
ax2.set_xlim([0, 30])
ax2.legend(loc='lower right')
ax2.grid(True, linestyle=':', alpha=0.7)

plt.tight_layout()
plt.savefig('figure_entrainment.png', dpi=300, bbox_inches='tight')
plt.show()