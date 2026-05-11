import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# 1. 定义带时间触发机制的 CPG 微分方程
def adaptive_hopf(state, t, epsilon, w_ext, t_start):
    x, y, w = state
    r = np.sqrt(x**2 + y**2)
    if r == 0: r = 1e-6
    
    if t >= t_start:
        F = np.sin(w_ext * t)
    else:
        F = 0.0
        
    dxdt = (1.0 - r**2) * x - w * y + epsilon * F
    dydt = (1.0 - r**2) * y + w * x
    dwdt = -epsilon * F * (y / r)
    
    return [dxdt, dydt, dwdt]

# 2. 仿真参数与时间轴
t_start_perturbation = 30.0  
t = np.linspace(0, 100, 5000)
epsilon_val = 0.8
w_cpg_val = 1.0
state0 = [0.1, 0.0, w_cpg_val] 

# 生成外部干扰信号数组
def generate_F_array(t_array, w_ext, t_start):
    return np.array([np.sin(w_ext * time) if time >= t_start else 0.0 for time in t_array])

# 计算导数 dw/dt 的辅助函数
def calculate_dwdt(sol, t_array, w_ext, t_start, eps):
    x = sol[:, 0]
    y = sol[:, 1]
    r = np.sqrt(x**2 + y**2)
    r[r == 0] = 1e-6
    F = generate_F_array(t_array, w_ext, t_start)
    dwdt = -eps * F * (y / r)
    return dwdt

# ================= 场景 A：频率相近 =================
w_ext_close = 0.8
ratio_close = w_ext_close / w_cpg_val  
sol_close = odeint(adaptive_hopf, state0, t, args=(epsilon_val, w_ext_close, t_start_perturbation))
x_close = sol_close[:, 0]
w_close = sol_close[:, 2]
F_close = generate_F_array(t, w_ext_close, t_start_perturbation)
dwdt_close = calculate_dwdt(sol_close, t, w_ext_close, t_start_perturbation, epsilon_val)

# ================= 场景 B：频率悬殊 =================
w_ext_far = 0.3
ratio_far = w_ext_far / w_cpg_val      
sol_far = odeint(adaptive_hopf, state0, t, args=(epsilon_val, w_ext_far, t_start_perturbation))
x_far = sol_far[:, 0]
w_far = sol_far[:, 2]
F_far = generate_F_array(t, w_ext_far, t_start_perturbation)
dwdt_far = calculate_dwdt(sol_far, t, w_ext_far, t_start_perturbation, epsilon_val)

# 3. 高级可视化布局 (删除主标题，完全利用 figsize 空间)
fig, axs = plt.subplots(2, 3, figsize=(24, 14))

# 通用绘图设置函数 (字号暴增版本)
def format_subplot(ax, title, ylabel, xlabel=""):
    # 子标题放大至 20
    ax.set_title(title, fontsize=24, pad=18, fontweight='bold')  
    # 坐标轴标签放大至 18
    ax.set_ylabel(ylabel, fontsize=18, fontweight='bold')        
    if xlabel: 
        ax.set_xlabel(xlabel, fontsize=18, fontweight='bold')    
    
    # 坐标轴刻度数字放大至 16
    ax.tick_params(axis='both', which='major', labelsize=20)
    
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.axvspan(0, t_start_perturbation, color='lightgreen', alpha=0.15)
    ax.axvspan(t_start_perturbation, 100, color='salmon', alpha=0.1)
    ax.axvline(t_start_perturbation, color='red', linestyle='--', linewidth=2)

# ----------------- 第一行：频率相近 (Close Frequencies) -----------------
# 1. 物理输出 x 
title_A1 = (f"1. Physical Output $x$\n"
            f"[Params: $\epsilon={epsilon_val}$, $\omega_{{cpg}}={w_cpg_val}$, $\omega_{{ext}}={ratio_close:.1f}\omega_{{cpg}}$]")
format_subplot(axs[0, 0], title_A1, "Amplitude")
axs[0, 0].plot(t, F_close, color='orange', linestyle='--', linewidth=2.5, alpha=0.8, label='Interference $F(t)$')
axs[0, 0].plot(t, x_close, color='#1f77b4', linewidth=3.5, label='CPG Output $x(t)$')
axs[0, 0].legend(loc='lower left', fontsize=16) # 图例字号爆改 16

# 2. 积分层 w
format_subplot(axs[0, 1], "2. Integration Layer $\omega$", "Frequency $\omega$")
axs[0, 1].plot(t, w_close, color='blue', linewidth=3.5, label=r'Internal $\omega$')
axs[0, 1].axhline(w_cpg_val, color='black', linestyle=':', linewidth=2, label=f'cpgial ({w_cpg_val})')
axs[0, 1].axhline(w_ext_close, color='green', linestyle='--', linewidth=2, label=f'Target ({w_ext_close})')
axs[0, 1].legend(loc='lower left', fontsize=16)

# 3. 导数层 dw/dt
format_subplot(axs[0, 2], "3. Derivative Layer $\dot{\omega}$", "Rate of Change")
axs[0, 2].plot(t, dwdt_close, color='purple', linewidth=3, label=r'$\dot{\omega}$')
axs[0, 2].axhline(0, color='black', linewidth=1.5)
axs[0, 2].legend(loc='lower left', fontsize=16)

# ----------------- 第二行：频率悬殊 (Far Frequencies) -----------------
# 1. 物理输出 x 
title_B1 = (f"1. Physical Output $x$ (Clash)\n"
            f"[Params: $\epsilon={epsilon_val}$, $\omega_{{cpg}}={w_cpg_val}$, $\omega_{{ext}}={ratio_far:.1f}\omega_{{cpg}}$]")
format_subplot(axs[1, 0], title_B1, "Amplitude", "Time (s)")
axs[1, 0].plot(t, F_far, color='orange', linestyle='--', linewidth=2.5, alpha=0.8, label='Interference $F(t)$')
axs[1, 0].plot(t, x_far, color='#d62728', linewidth=3.5, label='CPG Output $x(t)$')
axs[1, 0].legend(loc='lower left', fontsize=16)

# 2. 积分层 w
format_subplot(axs[1, 1], "2. Integration Layer $\omega$ (Paralysis)", "Frequency $\omega$", "Time (s)")
axs[1, 1].plot(t, w_far, color='red', linewidth=3.5, label=r'Internal $\omega$')
axs[1, 1].axhline(w_cpg_val, color='black', linestyle=':', linewidth=2, label=f'cpgial ({w_cpg_val})')
axs[1, 1].axhline(w_ext_far, color='green', linestyle='--', linewidth=2, label=f'Target ({w_ext_far})')
axs[1, 1].legend(loc='lower left', fontsize=16)

# 3. 导数层 dw/dt
format_subplot(axs[1, 2], "3. Derivative Layer $\dot{\omega}$ (High-Freq Noise)", "Rate of Change", "Time (s)")
axs[1, 2].plot(t, dwdt_far, color='purple', linewidth=3, alpha=0.8, label=r'$\dot{\omega}$')
axs[1, 2].axhline(0, color='black', linewidth=1.5)
axs[1, 2].legend(loc='lower left', fontsize=16)

# 优化整体布局：放大 hspace 和 wspace 防止大字号重叠
plt.tight_layout()
plt.subplots_adjust(hspace=0.4, wspace=0.25)

save_path = "cpg_full_pipeline_max_font.png"
plt.savefig(save_path, dpi=300, bbox_inches='tight')

plt.show()