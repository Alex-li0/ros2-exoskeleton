import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# ==========================================
# 1. 系统核心参数配置
# ==========================================
gamma = 1.0    # 极限环收敛速度
epsilon = 4.0  # 频率/相位耦合强度 (适度调高加速锁定)
eta = 2.5      # 振幅学习率

# ==========================================
# 2. 仿真阶段与信号设定
# ==========================================
w_init = 5.0   # CPG 初始固有频率
w_ext = 3.0    # 外部干扰信号频率 (与初始不同)
A1 = 1.0       # 阶段 2：干扰信号振幅
A2 = 2.5       # 阶段 3：干扰信号突变后的振幅

def get_target_signal(t):
    """根据所处时间段返回外部目标信号"""
    if t < 15.0:
        return 0.0 # 阶段 1：无干扰
    elif t < 30.0:
        return A1 * np.sin(w_ext * t) # 阶段 2：引入不同频率的干扰
    else:
        return A2 * np.sin(w_ext * t) # 阶段 3：改变干扰幅值

# ==========================================
# 3. 混合式 CPG 动力学方程 (带激活开关)
# ==========================================
def cpg_full_learning(state, t):
    x, y, omega, alpha = state
    r = np.sqrt(x**2 + y**2)
    if r == 0: 
        r = 0.0001 # 避免奇异点
    
    target_f = get_target_signal(t)
    
    # 核心技巧：激活开关
    # 如果处于第一阶段（无外部输入），切断误差反馈，保持开环自由振荡
    if t < 15.0:
        error = 0.0
    else:
        # 否则，开启闭环误差跟随
        error = target_f - x 
    
    # 动力学方程组
    dxdt = gamma * (alpha - r**2) * x - omega * y + epsilon * error
    dydt = gamma * (alpha - r**2) * y + omega * x
    domegadt = -epsilon * error * (y / r)
    dalphadt = eta * error * (x / r)
    
    return [dxdt, dydt, domegadt, dalphadt]

# ==========================================
# 4. 初始状态与时间设置
# ==========================================
# 初始状态: x=1.0, y=0.0, 初始频率 w=5.0, 初始振幅参数 alpha=1.0 (对应振幅1.0)
state0 = [1.0, 0.0, w_init, 1.0] 
t = np.linspace(0, 45, 4500) # 仿真总时长 45 秒

# 求解常微分方程
sol = odeint(cpg_full_learning, state0, t, hmax=0.01)

x_out = sol[:, 0]
omega_out = sol[:, 2]
alpha_out = sol[:, 3]

# 生成绘图所需的目标信号参考数组
input_signal = np.array([get_target_signal(ti) for ti in t])

# ==========================================
# 5. 绘图与可视化 (4 个子图全景展示)
# ==========================================
fig, axs = plt.subplots(4, 1, figsize=(14, 16), sharex=True)

# --- 子图 1: 外部目标输入信号 + CPG初始输出 ---
axs[0].plot(t, input_signal, 'k-', linewidth=1.5, label='External Interference Signal')
# 【修改点】：在第一张图叠加上 0-15s 没有干扰时的 CPG 自身输出
mask_phase1 = t < 15.0
axs[0].plot(t[mask_phase1], x_out[mask_phase1], 'b--', linewidth=2, label='CPG Free Oscillation (0-15s)')

axs[0].axvline(x=15, color='r', linestyle='--', alpha=0.7)
axs[0].axvline(x=30, color='r', linestyle='--', alpha=0.7)
axs[0].set_title('Subplot 1: External Signal & CPG Initial State', fontsize=13, fontweight='bold')
axs[0].set_ylabel('Amplitude', fontsize=12)
axs[0].legend(loc='upper left', fontsize=10)
axs[0].grid(True)

# --- 子图 2: 频率学习演化 ---
axs[1].plot(t, omega_out, 'g-', linewidth=3, label=r'Internal Frequency $\omega(t)$')
axs[1].axhline(y=w_init, color='gray', linestyle='--', linewidth=2, label=r'Initial $\omega_0$ = 5.0')
axs[1].axhline(y=w_ext, color='purple', linestyle='--', linewidth=2, label=r'Target $\omega_{ext}$ = 3.0')
axs[1].axvline(x=15, color='r', linestyle='--', alpha=0.7)
axs[1].axvline(x=30, color='r', linestyle='--', alpha=0.7)
axs[1].set_title('Subplot 2: Frequency Entrainment', fontsize=13, fontweight='bold')
axs[1].set_ylabel('Frequency (rad/s)', fontsize=12)
axs[1].legend(loc='upper right', fontsize=10)
axs[1].grid(True)

# --- 子图 3: 振幅参数学习演化 ---
axs[2].plot(t, alpha_out, 'm-', linewidth=3, label=r'Amplitude Parameter $\alpha(t)$')
axs[2].axhline(y=1.0**2, color='gray', linestyle='--', linewidth=2, label=r'Phase 1 Target $\alpha$ = 1.0')
axs[2].axhline(y=A1**2, color='c', linestyle='--', linewidth=2, label=rf'Phase 2 Target $\alpha$ = {A1**2}')
axs[2].axhline(y=A2**2, color='orange', linestyle='--', linewidth=2, label=rf'Phase 3 Target $\alpha$ = {A2**2}')
axs[2].axvline(x=15, color='r', linestyle='--', alpha=0.7)
axs[2].axvline(x=30, color='r', linestyle='--', alpha=0.7)
axs[2].set_title(r'Subplot 3: Amplitude Parameter Adaptation ($\alpha \rightarrow A^2$)', fontsize=13, fontweight='bold')
axs[2].set_ylabel(r'Parameter $\alpha$', fontsize=12)
axs[2].legend(loc='lower right', fontsize=10)
axs[2].grid(True)

# --- 子图 4: 最终输出轨迹 ---
axs[3].plot(t, input_signal, 'k--', linewidth=2, alpha=0.3, label='External Signal Reference')
axs[3].plot(t, x_out, 'b-', linewidth=2, label='CPG Output $x(t)$')
axs[3].axvline(x=15, color='r', linestyle='--', alpha=0.7)
axs[3].axvline(x=30, color='r', linestyle='--', alpha=0.7)
axs[3].set_title('Subplot 4: Final Output Trajectory', fontsize=13, fontweight='bold')
axs[3].set_xlabel('Time (s)', fontsize=12)
axs[3].set_ylabel('Position $x$', fontsize=12)
axs[3].legend(loc='upper right', fontsize=10)
axs[3].grid(True)

plt.tight_layout()
plt.savefig('amplitude_impact.png', dpi=300, bbox_inches='tight')
plt.show()