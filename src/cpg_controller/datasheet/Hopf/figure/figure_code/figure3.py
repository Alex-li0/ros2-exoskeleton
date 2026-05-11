import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def adaptive_hopf_multi_epsilon(state, t, gamma, mu, epsilon, target_freq):
    x, y, omega = state
    r = np.sqrt(x**2 + y**2)
    
    # 设定扰动信号，仅在前 30 秒有效
    F_t = np.sin(target_freq * t) if 0 <= t <= 30 else 0.0
        
    dxdt = gamma * (mu - r**2) * x - omega * y + epsilon * F_t
    dydt = gamma * (mu - r**2) * y + omega * x
    domegadt = -epsilon * F_t * (y / r) if r > 0.01 else 0
    
    return [dxdt, dydt, domegadt]

# 系统参数
gamma, mu, target_freq = 10.0, 1.0, 2.0 
epsilons = {'Too Small (0.2)': 0.2, 'Optimal (0.8)': 0.8, 'Too Large (4.0)': 4.0}
colors = {'Too Small (0.2)': 'blue', 'Optimal (0.8)': 'green', 'Too Large (4.0)': 'red'}

t = np.linspace(0, 40, 4000)
initial_state = [1.0, 0.0, 5.0]

plt.figure(figsize=(12, 5))
plt.axhline(y=target_freq, color='k', linestyle='--', label='Target Freq (2.0)', linewidth=2)
plt.axvspan(0, 30, color='lightgray', alpha=0.3, label='Perturbation Active (0-30s)')

for label, eps in epsilons.items():
    sol = odeint(adaptive_hopf_multi_epsilon, initial_state, t, args=(gamma, mu, eps, target_freq))
    plt.plot(t, sol[:, 2], label=f'$\epsilon$={eps} ({label.split("(")[0].strip()})', color=colors[label], linewidth=2)

plt.title('Impact of $\epsilon$ on Learning Speed and Chattering')
plt.xlabel('Time (s)')
plt.ylabel('Internal CPG Frequency $\omega$ (rad/s)')
plt.legend()
plt.grid(True)

# --- 新增的代码：保存高清图片 ---
plt.savefig('figure2_epsilon_impact.png', dpi=300, bbox_inches='tight')

plt.show()