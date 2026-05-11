import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint, cumtrapz

# 1. 辅助绘图函数
def format_subplot(ax, title, ylabel):
    ax.set_title(title, fontsize=18, pad=12, fontweight='bold')
    ax.set_ylabel(ylabel, fontsize=16, fontweight='bold')
    ax.tick_params(axis='both', which='major', labelsize=14)
    ax.grid(True, linestyle=':', alpha=0.7)

# 2. CPG 方程：保持你原版的 Hopf 逻辑，支持无滤波 (tau=0)
def adaptive_hopf_lpf(state, t, gamma, mu, epsilon, tau):
    x, y, w_raw, w_smooth = state
    r = np.sqrt(x**2 + y**2)
    if r == 0: r = 1e-6
    
    # 15秒前无干扰，15秒后施加目标频率为 2.5 的强干扰
    if t < 15:
        F_t = 0.0
    else:
        F_t = np.sin(2.5 * t)
        
    active_w = w_raw if tau == 0 else w_smooth
        
    dxdt = gamma * (mu - r**2) * x - active_w * y + epsilon * F_t
    dydt = gamma * (mu - r**2) * y + active_w * x
    
    domegadt_raw = -epsilon * F_t * (y / r)
    domegadt_smooth = 0 if tau == 0 else (w_raw - w_smooth) / tau
    
    return [dxdt, dydt, domegadt_raw, domegadt_smooth]

# 3. 核心参数与时间轴
gamma, mu = 10.0, 1.0
epsilon_val = 3.5  
t = np.linspace(0, 45, 6000) 

initial_state = [1.0, 0.0, 1.5, 1.5]
target_freqs = np.array([1.5 if time < 15 else 2.5 for time in t])

# 4. 可视化布局 (2行 x 1列)
fig, axs = plt.subplots(2, 1, figsize=(15, 12))

# 极简高亮：左上角等式
props = dict(boxstyle='round,pad=0.4', facecolor='white', edgecolor='#d62728', linewidth=2.5, alpha=0.9)
axs[0].text(0.02, 0.92, r"$\epsilon = 3.5$", transform=axs[0].transAxes, 
            fontsize=22, fontweight='bold', color='#d62728', va='top', bbox=props)

# 定义三种情况
conditions = {
    'No Filter (Raw)': (0.0, 'red'), 
    r'$\tau=0.4$ (Medium)': (0.4, 'blue'), 
    r'$\tau=1.5$ (Over-filtered)': (1.5, 'cyan')
}

# ================= 子图 (a): 内部角速度 \omega 的对比 (保持不变) =================
format_subplot(axs[0], "(a) Internal Frequency $\omega$: Trade-off between Chatter and Lag", r'$\omega$ (rad/s)')

axs[0].plot(t, target_freqs, color='black', linestyle='--', linewidth=3, label='Target Freq')
axs[0].axvline(15, color='gray', linestyle=':', linewidth=2)

for label, (tau_val, color) in conditions.items():
    sol = odeint(adaptive_hopf_lpf, initial_state, t, args=(gamma, mu, epsilon_val, tau_val))
    w_plot = sol[:, 2] if tau_val == 0 else sol[:, 3]
    alpha_val = 0.5 if tau_val == 0 else 1.0 
    linewidth_val = 1.5 if tau_val == 0 else 3.0
    
    axs[0].plot(t, w_plot, label=label, color=color, linewidth=linewidth_val, alpha=alpha_val)

axs[0].annotate('Perturbation ON', xy=(15.2, 1.6), xytext=(5, 1.8),
             arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=8),
             fontsize=14, fontweight='bold')
axs[0].legend(loc='lower right', fontsize=14)

# ================= 子图 (b): 物理输出 -> 关节角度积分 (新逻辑) =================
format_subplot(axs[1], r"(b) Joint Angle $\theta = \int \omega dt$ (Zoomed & Vertically Offset)", 'Angle $\theta$ (rad)')
axs[1].set_xlabel('Time (s)', fontsize=16, fontweight='bold')
axs[1].axvline(15, color='gray', linestyle=':', linewidth=2)

# 定义三种情况在 Y 轴的平移量，防止三条斜线完全重叠
axs[1].set_yticks([-18, 0, 18])
axs[1].set_yticklabels(['Over-filtered\n(Offset -18)', 'Medium Filter\n(Base 0)', 'No Filter\n(Offset +18)'], 
                       fontsize=12, fontweight='bold')

offsets = {0.0: 18.0, 0.4: 0.0, 1.5: -18.0}

for label, (tau_val, color) in conditions.items():
    sol = odeint(adaptive_hopf_lpf, initial_state, t, args=(gamma, mu, epsilon_val, tau_val))
    w_plot = sol[:, 2] if tau_val == 0 else sol[:, 3]
    
    # 【核心改动】：利用 cumtrapz 对提取出的角速度 w_plot 直接进行数值积分
    joint_angle = cumtrapz(w_plot, t, initial=0)
    
    # 加上平移量拉开距离
    joint_angle_offset = joint_angle + offsets[tau_val]
    
    linewidth_val = 2.0 if tau_val == 0 else 3.0
    axs[1].plot(t, joint_angle_offset, color=color, linewidth=linewidth_val, alpha=0.9)

# 局部放大 12s~25s 的黄金观察期，否则 45 秒内积分值高达 90 会彻底掩盖毛刺
axs[1].set_xlim(12, 25)
axs[1].set_ylim(0, 70) 

axs[1].annotate('Ripples on Integral\n(Causes Joint Jitter)', xy=(18, 55), xytext=(20, 62),
             arrowprops=dict(facecolor='red', shrink=0.05, width=1.5, headwidth=8),
             fontsize=13, fontweight='bold', color='red')

axs[1].annotate('Slope (Frequency)\nrises too slowly', xy=(17.5, 12), xytext=(19.5, 5),
             arrowprops=dict(facecolor='cyan', shrink=0.05, width=1.5, headwidth=8),
             fontsize=13, fontweight='bold', color='darkcyan')

plt.tight_layout()
plt.subplots_adjust(top=0.92, hspace=0.3)

save_path = "figure_hopf_omega_vs_joint.png"
plt.savefig(save_path, dpi=300, bbox_inches='tight')
plt.show()