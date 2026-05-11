import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# --- 系统参数 ---
gamma = 1.0  # 恢复系数
mu = 1.0     # 控制振幅的参数 (极限环半径的平方)
omega = 2.0  # 固有频率

# --- 定义系统微分方程 ---
def basic_hopf(state, t):
    x, y = state
    r2 = x**2 + y**2
    dxdt = gamma * (mu - r2) * x - omega * y
    dydt = gamma * (mu - r2) * y + omega * x
    return [dxdt, dydt]

# --- 求解轨迹 ---
t = np.linspace(0, 15, 1500)
# 从内部出发的轨迹 (初始幅值小)
sol_inside = odeint(basic_hopf, [0.1, 0.1], t)
# 从外部出发的轨迹 (初始幅值大)
sol_outside = odeint(basic_hopf, [1.8, 1.8], t)

# ==========================================
# 创建带有两个子图的画布 (1行2列)
# ==========================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# --- 子图 1：相平面 (Phase Plane) ---
x_val = np.linspace(-2, 2, 20)
y_val = np.linspace(-2, 2, 20)
X, Y = np.meshgrid(x_val, y_val)

r2_mesh = X**2 + Y**2
U = gamma * (mu - r2_mesh) * X - omega * Y
V = gamma * (mu - r2_mesh) * Y + omega * X

# 归一化箭头长度
N = np.sqrt(U**2 + V**2)
U_norm, V_norm = U/N, V/N

ax1.quiver(X, Y, U_norm, V_norm, N, cmap='viridis', alpha=0.6)

# 绘制轨迹和极限环
ax1.plot(sol_inside[:, 0], sol_inside[:, 1], 'b--', linewidth=1.5, label='Trajectory (from inside)')
ax1.plot(sol_outside[:, 0], sol_outside[:, 1], 'g--', linewidth=1.5, label='Trajectory (from outside)')

theta = np.linspace(0, 2*np.pi, 100)
r = np.sqrt(mu)
ax1.plot(r*np.cos(theta), r*np.sin(theta), 'r', linewidth=2.5, label=f'Limit Cycle ($r=\sqrt{{\mu}}$)')

ax1.set_title('(a) Phase Plane Vector Field', fontsize=14)
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.axis('equal')
ax1.set_xlim([-2.2, 2.2])
ax1.set_ylim([-2.2, 2.2])
ax1.legend(loc='upper right')
ax1.grid(True, linestyle=':', alpha=0.7)

# --- 子图 2：笛卡尔时域波形 (Time Domain) ---
# 我们以“从外部出发”的轨迹为例，展示它逐渐收敛到稳态振幅的过程
ax2.plot(t, sol_outside[:, 0], 'r-', linewidth=2, label='x(t) (Position)')
ax2.plot(t, sol_outside[:, 1], 'b-', linewidth=2, label='y(t) (Velocity)')

# 标出目标振幅参考线
ax2.axhline(np.sqrt(mu), color='k', linestyle='--', alpha=0.5, label=r'Steady Amplitude ($\sqrt{\mu}$)')
ax2.axhline(-np.sqrt(mu), color='k', linestyle='--', alpha=0.5)

ax2.set_title('(b) Time Domain Signals (Cartesian)', fontsize=14)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Amplitude')
ax2.set_xlim([0, 15])
ax2.legend(loc='upper right')
ax2.grid(True, linestyle=':', alpha=0.7)

# --- 调整布局并保存 ---
plt.tight_layout()
plt.savefig('figure1_limit_cycle.png', dpi=300, bbox_inches='tight')
plt.show()