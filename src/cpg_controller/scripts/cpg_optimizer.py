#!/usr/bin/env python3
"""
CPG 参数优化器 —— 从真实人体步态 .mat 数据拟合 Hopf 振荡器映射权重
=====================================================================

工作流程 (两种数据源联合使用):
  1. 从 '绝对时间完整值/' 提取真实步频 omega (FFT 分析)
  2. 从 '步态周期平均值/' 拟合谐波权重 (w_offset, w_x, w_y)
  3. 自动生成与 legs_cpg.cpp 节点匹配的 YAML 配置

两种数据一一对应 (同 speed, 同 trial)。

使用方法:
  python3 cpg_optimizer.py                     # 默认 S=1, Trial=1
  python3 cpg_optimizer.py --speed 1x5         # 1.5m/s
  python3 cpg_optimizer.py --speed 0x5 --trial 3
  python3 cpg_optimizer.py --no-plot           # 不生成图像
"""

import argparse
import os
import sys
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ============================================================
# 1. 数据加载
# ============================================================

def load_gait_data(mat_path):
    """从 .mat 文件读取左腿三关节矢状面角度 (度)。"""
    mat = sio.loadmat(mat_path)
    hip_deg   = mat['LHipAngles'][:, 0]
    knee_deg  = mat['LKneeAngles'][:, 0]
    ankle_deg = mat['LAnkleAngles'][:, 0]
    return hip_deg, knee_deg, ankle_deg


# ============================================================
# 2. 从绝对时间数据提取步频
# ============================================================

def extract_walking_segment(hip_deg, threshold_deg=1.0, margin=10):
    """提取行走段，跳过首尾静止期。返回 (start_idx, end_idx) 闭区间。"""
    active = np.abs(hip_deg) > threshold_deg
    active_idx = np.where(active)[0]
    if len(active_idx) == 0:
        return 0, len(hip_deg) - 1
    start_idx = max(0, active_idx[0] - margin)
    end_idx = min(len(hip_deg) - 1, active_idx[-1] + margin)
    return start_idx, end_idx


def estimate_gait_frequency(hip_deg_walk, fs=100.0):
    """FFT 估计行走段步态主频。返回 (f_hz, omega_rad_s, T_sec)。"""
    N = len(hip_deg_walk)
    signal = hip_deg_walk - np.mean(hip_deg_walk)
    fft_vals = np.abs(np.fft.fft(signal))[:N // 2]
    freqs = np.fft.fftfreq(N, d=1.0 / fs)[:N // 2]
    peak_idx = np.argmax(fft_vals[1:]) + 1
    f_gait = freqs[peak_idx]
    T_cycle = 1.0 / f_gait
    omega = 2.0 * np.pi * f_gait
    return f_gait, omega, T_cycle


# ============================================================
# 3. CPG 振荡器采样
# ============================================================

def sample_cpg_one_cycle(num_points, omega, mu=1.0, gamma=15.0,
                         dt_ctrl=0.01, sub_steps=10):
    """
    用给定 omega 在极限环上均匀采样 num_points 个点 (一个周期)。
    预热足够多周期确保收敛。
    """
    dt_sim = dt_ctrl / sub_steps
    T_cycle = 2.0 * np.pi / omega
    x, y = 0.1, 0.1

    # 预热 20 个完整周期
    warmup_cycles = 20
    warmup_steps = int(warmup_cycles * T_cycle / dt_ctrl)
    for _ in range(warmup_steps):
        r2 = x * x + y * y
        for _ in range(sub_steps):
            dx = gamma * (mu - r2) * x - omega * y
            dy = gamma * (mu - r2) * y + omega * x
            x += dt_sim * dx
            y += dt_sim * dy

    # 在极限环上等间隔采样一个完整周期
    # 控制频率和 omega 匹配: 每步 = T_cycle/num_points 秒
    dt_sample = T_cycle / num_points
    dt_sample_sub = dt_sample / sub_steps
    X_list, Y_list = [], []
    for _ in range(num_points):
        for _ in range(sub_steps):
            r2 = x * x + y * y
            dx = gamma * (mu - r2) * x - omega * y
            dy = gamma * (mu - r2) * y + omega * x
            x += dt_sample_sub * dx
            y += dt_sample_sub * dy
        X_list.append(x)
        Y_list.append(y)

    return np.array(X_list), np.array(Y_list)


# ============================================================
# 4. OLS 谐波拟合
# ============================================================

def fit_harmonics_ols(X, Y, ref_signal, num_harmonics):
    """
    OLS 拟合谐波权重:
      theta = w_offset
            + w_x[1]*X + w_y[1]*Y                              (基频)
            + w_x[2]*(X²-Y²) + w_y[2]*(2XY)                   (2倍频)
            + w_x[3]*X(X²-3Y²) + w_y[3]*Y(3X²-Y²)            (3倍频)

    与 legs_cpg.cpp 的谐波生成完全对应。

    返回: w_offset, w_x (list), w_y (list), residual_rms
    """
    features = [np.ones_like(X)]
    if num_harmonics >= 2:
        features.append(X)                         # cos(wt)
        features.append(Y)                         # sin(wt)
    if num_harmonics >= 3:
        features.append(X**2 - Y**2)               # cos(2wt)
        features.append(2.0 * X * Y)               # sin(2wt)
    if num_harmonics >= 4:
        features.append(X * (X**2 - 3 * Y**2))     # cos(3wt)
        features.append(Y * (3 * X**2 - Y**2))     # sin(3wt)

    A = np.vstack(features).T
    w, _, _, _ = np.linalg.lstsq(A, ref_signal, rcond=None)

    w_offset = w[0]
    w_x, w_y = [0.0], [0.0]
    idx = 1
    if num_harmonics >= 2:
        w_x.append(w[idx]); w_y.append(w[idx + 1]); idx += 2
    if num_harmonics >= 3:
        w_x.append(w[idx]); w_y.append(w[idx + 1]); idx += 2
    if num_harmonics >= 4:
        w_x.append(w[idx]); w_y.append(w[idx + 1]); idx += 2

    residual = np.sqrt(np.mean((ref_signal - A @ w)**2))
    return w_offset, w_x, w_y, residual


def reconstruct_signal(X, Y, w_offset, w_x, w_y, num_harmonics):
    """从权重重建信号。"""
    theta = np.full_like(X, w_offset)
    if num_harmonics >= 2:
        theta += w_x[1] * X + w_y[1] * Y
    if num_harmonics >= 3:
        theta += w_x[2] * (X**2 - Y**2) + w_y[2] * (2.0 * X * Y)
    if num_harmonics >= 4:
        theta += w_x[3] * X * (X**2 - 3 * Y**2) + w_y[3] * Y * (3 * X**2 - Y**2)
    return theta


# ============================================================
# 5. YAML 配置生成
# ============================================================

def generate_yaml(joint_params, joint_limits, omega, mu=1.0, gamma=15.0, output_path=None):
    """生成与 legs_cpg.cpp 匹配的 YAML。"""
    lines = [
        "adaptive_cpg_node:",
        "  ros__parameters:",
        f"    mu: {mu}",
        f"    omega: {omega:.6f}",
        f"    gamma: {gamma}",
        "    eta_w: 0.00      # 纯离线回放，学习率设为0",
        "    eta_omega: 0.00",
        "",
        "    # === 启停状态机参数 ===",
        "    ramp_time: 0.5              # 渐入/渐出时间 (秒)",
        "    initial_perturbation: 0.1   # 启动时振子扰动量",
        "",
        "    # === 踝关节开关 ===",
        "    # true: 仿真时输出 CPG 踝关节角度 (可视化)",
        "    # false: 实机时踝关节锁定 w_offset (被动关节不助力)",
        "    publish_ankle: true",
        "",
        "    joints:",
    ]

    joint_order = ['left_hip3', 'left_knee', 'left_ankle',
                   'right_hip3', 'right_knee', 'right_ankle']

    for name in joint_order:
        src_name = name.replace('right_', 'left_')
        p = joint_params[src_name]
        lo, hi = joint_limits[src_name]
        wx_str = ', '.join(f"{v:.4f}" for v in p['w_x'])
        wy_str = ', '.join(f"{v:.4f}" for v in p['w_y'])
        lines.extend([
            f"      {name}:",
            f"        num_harmonics: {p['n_h']}",
            f"        limit_min: {lo}",
            f"        limit_max: {hi}",
            f"        w_offset: {p['w_offset']:.4f}",
            f"        w_x: [{wx_str}]",
            f"        w_y: [{wy_str}]",
            "",
        ])

    yaml_content = '\n'.join(lines)
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)
        print(f"\n✅ YAML 配置已保存: {output_path}")
    return yaml_content


# ============================================================
# 6. 可视化
# ============================================================

def plot_results(t_norm, joint_data, joint_fitted, joint_names,
                 omega, T_cycle, speed_label, save_path):
    """绘制拟合对比图。"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    titles = ['Hip (left_hip3)', 'Knee (left_knee)', 'Ankle (left_ankle)']

    for i, (name, title) in enumerate(zip(joint_names, titles)):
        ref = np.rad2deg(joint_data[name])
        fit = np.rad2deg(joint_fitted[name])
        rmse = np.sqrt(np.mean((ref - fit)**2))
        axes[i].plot(t_norm, ref, 'r-', lw=2.5, label='Human Reference')
        axes[i].plot(t_norm, fit, 'b--', lw=2, label=f'CPG Fit (RMSE={rmse:.2f}°)')
        axes[i].set_ylabel('Angle (deg)')
        axes[i].set_title(title)
        axes[i].legend(loc='upper right')
        axes[i].grid(True, alpha=0.3)

    axes[-1].set_xlabel('Gait Cycle Phase (0→1)')
    plt.suptitle(
        f'CPG Harmonic Fit — Speed {speed_label}  '
        f'(ω={omega:.2f} rad/s, T={T_cycle:.3f}s)',
        fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"📊 拟合对比图已保存: {save_path}")


# ============================================================
# 7. 主函数
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='从人体步态 .mat 数据拟合 CPG 谐波权重参数')
    parser.add_argument('--speed', type=str, default='1',
                        choices=['0x5', '1', '1x5'],
                        help='步态速度 (默认: 1, 即 1.0 m/s)')
    parser.add_argument('--trial', type=int, default=1,
                        choices=[1, 2, 3, 4, 5],
                        help='试次编号 (默认: 1)')
    parser.add_argument('--mu', type=float, default=1.0,
                        help='极限环半径平方 (默认: 1.0)')
    parser.add_argument('--gamma', type=float, default=15.0,
                        help='收敛速度 (默认: 15.0)')
    parser.add_argument('--no-plot', action='store_true',
                        help='不生成绘图')
    parser.add_argument('--output', type=str, default=None,
                        help='YAML 输出路径 (默认: config/cpg.yaml)')
    parser.add_argument('--fs', type=float, default=100.0,
                        help='绝对时间数据采样率 Hz (默认: 100)')

    args = parser.parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    speed_map = {'0x5': '0.5', '1': '1.0', '1x5': '1.5'}
    speed_label = speed_map.get(args.speed, args.speed)

    # ----------------------------------------------------------
    # 定位两个数据文件
    # ----------------------------------------------------------
    base_dir = os.path.join(script_dir, '关节角度')
    fname = f'P01_Ideal_LG_0_S_{args.speed}_Trial_{args.trial}_Angles.mat'

    cycle_path = os.path.join(base_dir, '步态周期平均值', fname)
    abs_path   = os.path.join(base_dir, '绝对时间完整值', fname)

    print("=" * 60)
    print(f"CPG 参数优化器 — 速度 {speed_label} m/s, Trial {args.trial}")
    print("=" * 60)

    if not os.path.exists(cycle_path):
        print(f"❌ 周期数据不存在: {cycle_path}"); sys.exit(1)
    if not os.path.exists(abs_path):
        print(f"❌ 绝对时间数据不存在: {abs_path}"); sys.exit(1)

    # ==========================================================
    # 步骤 1: 从绝对时间数据提取真实步频
    # ==========================================================
    print("\n--- 步骤 1: 提取真实步频 (FFT) ---")
    abs_hip, _, _ = load_gait_data(abs_path)
    N_abs = len(abs_hip)
    print(f"  绝对时间数据: {N_abs} 点, {N_abs / args.fs:.2f}s")

    start_idx, end_idx = extract_walking_segment(abs_hip)
    hip_walk = abs_hip[start_idx:end_idx + 1]
    N_walk = end_idx - start_idx + 1

    f_gait, omega, T_cycle = estimate_gait_frequency(hip_walk, fs=args.fs)
    print(f"  行走段: [{start_idx}, {end_idx}], {N_walk} 点 ({N_walk/args.fs:.2f}s)")
    print(f"  步态频率: {f_gait:.4f} Hz")
    print(f"  步态周期: {T_cycle:.3f} s")
    print(f"  角频率 ω: {omega:.4f} rad/s")

    # ==========================================================
    # 步骤 2: 从周期平均数据拟合谐波权重
    # ==========================================================
    print(f"\n--- 步骤 2: 拟合谐波权重 ---")
    hip_deg, knee_deg, ankle_deg = load_gait_data(cycle_path)
    N_cycle = len(hip_deg)
    hip_rad   = np.deg2rad(hip_deg)
    knee_rad  = -np.deg2rad(knee_deg)   # 取反! URDF 膝关节弯曲为负值
    ankle_rad = np.deg2rad(ankle_deg)
    print(f"  周期数据: {N_cycle} 点/周期")

    # 用真实 omega 在极限环上采样 N_cycle 个等间隔点
    X, Y = sample_cpg_one_cycle(N_cycle, omega, mu=args.mu, gamma=args.gamma)

    # 验证极限环半径
    r_check = np.sqrt(X[0]**2 + Y[0]**2)
    print(f"  极限环半径: r = {r_check:.4f} (期望: {np.sqrt(args.mu):.4f})")

    # 从实际数据计算各关节安全限位 (数据范围 + 10% 裕量)
    limit_margin = 0.10
    joint_limits = {}
    for name, rad_data in [('left_hip3', hip_rad), ('left_knee', knee_rad), ('left_ankle', ankle_rad)]:
        d_min = float(rad_data.min())
        d_max = float(rad_data.max())
        span = d_max - d_min
        lo = round(d_min - limit_margin * span, 2)
        hi = round(d_max + limit_margin * span, 2)
        joint_limits[name] = (lo, hi)
        print(f"  {name} 数据范围: [{np.rad2deg(d_min):.2f}°, {np.rad2deg(d_max):.2f}°]"
              f" = [{d_min:.4f}, {d_max:.4f}] rad → limit: [{lo}, {hi}]")

    joint_n_h = {
        'left_hip3':  2,   # 常数 + 基频
        'left_knee':  3,   # 常数 + 基频 + 二倍频
        'left_ankle': 4,   # 常数 + 基频 + 二倍频 + 三倍频 (被动关节，需更高精度拟合)
    }
    joint_names = ['left_hip3', 'left_knee', 'left_ankle']
    joint_data = {
        'left_hip3': hip_rad,
        'left_knee': knee_rad,
        'left_ankle': ankle_rad,
    }

    joint_params = {}
    joint_fitted = {}

    print("\n  OLS 拟合结果:")
    for name in joint_names:
        n_h = joint_n_h[name]
        w_offset, w_x, w_y, res = fit_harmonics_ols(X, Y, joint_data[name], n_h)
        joint_params[name] = {
            'n_h': n_h, 'w_offset': w_offset,
            'w_x': w_x, 'w_y': w_y, 'residual': res,
        }
        joint_fitted[name] = reconstruct_signal(X, Y, w_offset, w_x, w_y, n_h)

        print(f"\n  [{name}] (n_h={n_h})")
        print(f"    w_offset = {w_offset:+.4f} rad ({np.rad2deg(w_offset):+.2f}°)")
        for i in range(1, n_h):
            print(f"    w_x[{i}] = {w_x[i]:+.4f}   w_y[{i}] = {w_y[i]:+.4f}")
        print(f"    RMSE = {np.rad2deg(res):.3f}°")

    # ==========================================================
    # 步骤 3: 生成 YAML
    # ==========================================================
    # 文件名包含 speed 和 trial
    yaml_basename = f'cpgS{args.speed}T{args.trial}.yaml'
    yaml_path = args.output or os.path.join(
        '/home/lsy/vins_ws/src/cpg_controller/config', yaml_basename)
    yaml_content = generate_yaml(joint_params, joint_limits, omega=omega, mu=args.mu,
                                  gamma=args.gamma, output_path=yaml_path)

    print("\n  生成的 YAML:")
    for line in yaml_content.split('\n'):
        print(f"    {line}")

    # ==========================================================
    # 步骤 4: 绘图
    # ==========================================================
    if not args.no_plot:
        t_norm = np.linspace(0, 1, N_cycle, endpoint=False)
        plot_path = os.path.join(script_dir, f'gait_fit_S{args.speed}_T{args.trial}.png')
        plot_results(t_norm, joint_data, joint_fitted, joint_names,
                     omega, T_cycle, speed_label, plot_path)


if __name__ == '__main__':
    main()