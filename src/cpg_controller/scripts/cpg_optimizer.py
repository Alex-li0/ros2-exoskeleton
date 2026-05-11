#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import os

def generate_reference_gait(num_points=100, dt=0.01):
    t = np.linspace(0, num_points * dt, num_points, endpoint=False)
    # 真实步态
    ref_hip = 0.4 * np.sin(2 * np.pi * t) + 0.1 
    ref_knee = -0.5 * np.sin(2 * np.pi * t - np.pi/2) - 0.2 * np.sin(4 * np.pi * t) - 0.5
    ref_ankle = 0.15 * np.sin(2 * np.pi * t + np.pi/4) - 0.05
    return t, ref_hip, ref_knee, ref_ankle

if __name__ == "__main__":
    num_points = 100
    dt_ctrl = 0.01   # 控制周期 10ms
    dt_sim = 0.001   # 仿真子步长 1ms (消除毛刺的核心)
    sub_steps = int(dt_ctrl / dt_sim)
    
    t, ref_hip, ref_knee, ref_ankle = generate_reference_gait(num_points, dt_ctrl)
    
    # 1. 生成极度纯净的主振荡器波形
    x, y = 0.1, 0.1
    mu, omega, gamma = 1.0, 6.2831853, 10.0 
    
    # 充分预热，确保完全收敛到极限环
    for _ in range(2000):
        r2 = x**2 + y**2
        x += dt_sim * (gamma * (mu - r2) * x - omega * y)
        y += dt_sim * (gamma * (mu - r2) * y + omega * x)

    X_list, Y_list = [], []
    for _ in range(num_points):
        # 引入子步长积分
        for _ in range(sub_steps):
            r2 = x**2 + y**2
            dx = gamma * (mu - r2) * x - omega * y
            dy = gamma * (mu - r2) * y + omega * x
            x += dt_sim * dx
            y += dt_sim * dy
        X_list.append(x)
        Y_list.append(y)
        
    X = np.array(X_list)
    Y = np.array(Y_list)
    
    # 2. 构造特征矩阵 (包含二倍角特征)
    X2_Y2 = X**2 - Y**2
    XY2 = 2 * X * Y
    Ones = np.ones_like(X)
    
    # 3. 工程师的暴力美学：最小二乘法直接求解最优权重 (毫秒级)
    A_hip_ankle = np.vstack([X, Y, Ones]).T
    w_hip, _, _, _ = np.linalg.lstsq(A_hip_ankle, ref_hip, rcond=None)
    w_ankle, _, _, _ = np.linalg.lstsq(A_hip_ankle, ref_ankle, rcond=None)
    
    A_knee = np.vstack([X, Y, X2_Y2, XY2, Ones]).T
    w_knee, _, _, _ = np.linalg.lstsq(A_knee, ref_knee, rcond=None)

    print("\n=========================================")
    print("OLS 精确求解完成")
    print("=========================================")
    print(f"w_hx: {w_hip[0]:.4f},  w_hy: {w_hip[1]:.4f},  h_offset: {w_hip[2]:.4f}")
    print(f"w_kx1: {w_knee[0]:.4f}, w_ky1: {w_knee[1]:.4f}")
    print(f"w_kx2: {w_knee[2]:.4f}, w_ky2: {w_knee[3]:.4f}, k_offset: {w_knee[4]:.4f}")
    print(f"w_ax: {w_ankle[0]:.4f},  w_ay: {w_ankle[1]:.4f},  a_offset: {w_ankle[2]:.4f}")
    
    # =======================================================
    # 新增功能：自动生成 YAML 文件
    # =======================================================
    yaml_content = f"""single_leg_cpg_node:
  ros__parameters:
    # ==========================================
    # 1. CPG 底层数学参数 (Heartbeat 主振荡器)
    # ==========================================
    mu: 1.0              # 极限环半径的平方 (决定振幅基准，通常保持 1.0)
    omega: 6.2831853     # 基础角频率 (2 * pi * 1Hz = 6.28)
    gamma: 10.0          # 收敛速度 (数字越大，回到极限环越快)

    # ==========================================
    # 2. 髋关节 (Hip) 映射参数
    # ==========================================
    w_hx: {w_hip[0]:.4f}            # cos(wt) 的权重
    w_hy: {w_hip[1]:.4f}            # sin(wt) 的权重
    h_offset: {w_hip[2]:.4f}        # 关节运动中点偏移

    # ==========================================
    # 3. 膝关节 (Knee) 映射参数 (包含双峰特征)
    # ==========================================
    w_kx1: {w_knee[0]:.4f}          # cos(wt) 的权重 (sin 滞后 90 度即为 -cos)
    w_ky1: {w_knee[1]:.4f}          # sin(wt) 的权重
    w_kx2: {w_knee[2]:.4f}          # cos(2wt) 的权重
    w_ky2: {w_knee[3]:.4f}          # sin(2wt) 的权重 (二倍角倍频，塑造双峰)
    k_offset: {w_knee[4]:.4f}       # 关节运动中点偏移

    # ==========================================
    # 4. 踝关节 (Ankle) 映射参数
    # ==========================================
    w_ax: {w_ankle[0]:.4f}          # cos(wt) 的权重
    w_ay: {w_ankle[1]:.4f}          # sin(wt) 的权重
    a_offset: {w_ankle[2]:.4f}      # 关节运动中点偏移
"""

    # 自动寻址：定位到当前脚本的上一级目录，进入 config 文件夹

    config_dir = os.path.join('/home/lsy/vins_ws/src/cpg_controller/config')
    
    # 如果 config 文件夹不存在，自动创建它
    if not os.path.exists(config_dir):
        os.makedirs(config_dir)
        
    # 保存路径为 config/cpg_params.yaml
    yaml_file_path = os.path.join(config_dir, 'cpg_params.yaml')
    
    with open(yaml_file_path, 'w', encoding='utf-8') as f:
        f.write(yaml_content)
        
    print(f"=========================================")
    print(f"✅ YAML 配置文件已自动生成并保存至:")
    print(f"📂 {yaml_file_path}")
    print(f"=========================================\n")

    # 4. 验证求解结果 (绘图)
    opt_hip = A_hip_ankle @ w_hip
    opt_knee = A_knee @ w_knee
    opt_ankle = A_hip_ankle @ w_ankle

    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1); plt.plot(t, ref_hip, 'r--', lw=3, label='Human Reference'); plt.plot(t, opt_hip, 'b-', label='CPG OLS Fit'); plt.title('Hip')
    plt.legend()
    plt.subplot(3, 1, 2); plt.plot(t, ref_knee, 'r--', lw=3); plt.plot(t, opt_knee, 'b-'); plt.title('Knee (Perfect Match)')
    plt.subplot(3, 1, 3); plt.plot(t, ref_ankle, 'r--', lw=3); plt.plot(t, opt_ankle, 'b-'); plt.title('Ankle')
    plt.tight_layout(); plt.show()