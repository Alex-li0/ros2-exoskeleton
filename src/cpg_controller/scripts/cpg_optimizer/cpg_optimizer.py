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
    """从 .mat 文件读取左右腿三关节矢状面角度 (度)。返回 (left_data, right_data)。"""
    mat = sio.loadmat(mat_path)
    left_data = {
        'hip':   mat['LHipAngles'][:, 0],
        'knee':  mat['LKneeAngles'][:, 0],
        'ankle': mat['LAnkleAngles'][:, 0],
    }
    right_data = {
        'hip':   mat['RHipAngles'][:, 0],
        'knee':  mat['RKneeAngles'][:, 0],
        'ankle': mat['RAnkleAngles'][:, 0],
    }
    return left_data, right_data


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

def fit_harmonics_ols(X, Y, ref_signal, num_harmonics, flip_odd=False):
    """
    OLS 拟合谐波权重:
      theta = w_offset
            + w_x[1]*X + w_y[1]*Y                              (基频)
            + w_x[2]*(X²-Y²) + w_y[2]*(2XY)                   (2倍频)
            + w_x[3]*X(X²-3Y²) + w_y[3]*Y(3X²-Y²)            (3倍频)

    与 legs_cpg.cpp 的谐波生成完全对应。
    flip_odd=True 时，奇数谐波特征取反（用于右腿，使左右腿相位差 180°）。

    返回: w_offset, w_x (list), w_y (list), residual_rms
    """
    sign = -1.0 if flip_odd else 1.0
    features = [np.ones_like(X)]
    if num_harmonics >= 2:
        features.append(sign * X)                    # cos(wt)
        features.append(sign * Y)                    # sin(wt)
    if num_harmonics >= 3:
        features.append(X**2 - Y**2)                 # cos(2wt)  — 偶数谐波不取反
        features.append(2.0 * X * Y)                 # sin(2wt)
    if num_harmonics >= 4:
        features.append(sign * X * (X**2 - 3 * Y**2))   # cos(3wt)
        features.append(sign * Y * (3 * X**2 - Y**2))   # sin(3wt)

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


def fit_harmonics_wls(X, Y, ref_signal, num_harmonics, flip_odd=False, weight_mode='gradient'):
    """
    加权最小二乘 (WLS) 拟合谐波权重。
    对关键相位点（峰值、谷值、梯度大的区域）给予更高权重。

    weight_mode: 'gradient' | 'peak' | 'uniform'
      - gradient: 根据信号梯度加权（变化快的区域权重高）
      - peak: 根据信号峰值加权（峰值/谷值区域权重高）
      - uniform: 等权重（退化为OLS）

    返回: w_offset, w_x (list), w_y (list), residual_rms
    """
    N = len(ref_signal)

    # 计算权重
    if weight_mode == 'gradient':
        # 根据梯度加权：变化快的区域权重高
        gradient = np.abs(np.gradient(ref_signal))
        gradient = gradient / gradient.max()  # 归一化到 [0, 1]
        weights = 1.0 + 2.0 * gradient  # 权重范围 [1, 3]
    elif weight_mode == 'peak':
        # 根据峰值加权：峰值/谷值区域权重高
        signal_range = ref_signal.max() - ref_signal.min()
        mid = (ref_signal.max() + ref_signal.min()) / 2
        dist_from_mid = np.abs(ref_signal - mid) / signal_range
        weights = 1.0 + 2.0 * dist_from_mid  # 权重范围 [1, 3]
    else:  # uniform
        weights = np.ones(N)

    # 构建特征矩阵
    sign = -1.0 if flip_odd else 1.0
    features = [np.ones_like(X)]
    if num_harmonics >= 2:
        features.append(sign * X)
        features.append(sign * Y)
    if num_harmonics >= 3:
        features.append(X**2 - Y**2)
        features.append(2.0 * X * Y)
    if num_harmonics >= 4:
        features.append(sign * X * (X**2 - 3 * Y**2))
        features.append(sign * Y * (3 * X**2 - Y**2))

    A = np.vstack(features).T

    # 加权最小二乘: (A^T W A) w = A^T W b
    W = np.diag(weights)
    AtWA = A.T @ W @ A
    AtWb = A.T @ W @ ref_signal
    w = np.linalg.solve(AtWA, AtWb)

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


def reconstruct_signal(X, Y, w_offset, w_x, w_y, num_harmonics, flip_odd=False):
    """从权重重建信号。flip_odd 需与拟合时一致。"""
    sign = -1.0 if flip_odd else 1.0
    theta = np.full_like(X, w_offset)
    if num_harmonics >= 2:
        theta += w_x[1] * (sign * X) + w_y[1] * (sign * Y)
    if num_harmonics >= 3:
        theta += w_x[2] * (X**2 - Y**2) + w_y[2] * (2.0 * X * Y)
    if num_harmonics >= 4:
        theta += w_x[3] * (sign * X * (X**2 - 3 * Y**2)) + w_y[3] * (sign * Y * (3 * X**2 - Y**2))
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
        p = joint_params[name]
        lo, hi = joint_limits[name]
        w_off = p['w_offset']

        # 髋关节 w_offset 归零，限位同步平移
        is_hip = 'hip3' in name
        if is_hip:
            lo = round(lo - w_off, 4)
            hi = round(hi - w_off, 4)
            w_off = 0.0

        wx_str = ', '.join(f"{v:.4f}" for v in p['w_x'])
        wy_str = ', '.join(f"{v:.4f}" for v in p['w_y'])
        lines.extend([
            f"      {name}:",
            f"        num_harmonics: {p['n_h']}",
            f"        limit_min: {lo}",
            f"        limit_max: {hi}",
            f"        w_offset: {w_off:.4f}",
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

def plot_results(t_norm, joint_data, joint_fitted, omega, T_cycle, speed_label, save_path,
                 joint_data_raw=None, correct_offset=False):
    """绘制拟合对比图（左右腿独立显示）。"""
    if correct_offset and joint_data_raw:
        # 校正模式：显示校正前后对比
        fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
        joint_types = ['hip3', 'knee', 'ankle']
        joint_labels = ['Hip', 'Knee', 'Ankle']

        for row, (jtype, jlabel) in enumerate(zip(joint_types, joint_labels)):
            # 左列：原始数据
            name = f'left_{jtype}'
            ref_raw = np.rad2deg(joint_data_raw[name])
            ref_corr = np.rad2deg(joint_data[name])
            fit = np.rad2deg(joint_fitted[name])
            rmse = np.sqrt(np.mean((ref_corr - fit)**2))

            axes[row][0].plot(t_norm, ref_raw, 'gray', lw=1.5, alpha=0.5, label='Raw')
            axes[row][0].plot(t_norm, ref_corr, 'r-', lw=2.5, label='Corrected')
            axes[row][0].plot(t_norm, fit, 'b--', lw=2, label=f'Fit (RMSE={rmse:.2f}°)')
            axes[row][0].set_ylabel('Angle (deg)')
            axes[row][0].set_title(f'Left {jlabel}')
            axes[row][0].legend(loc='upper right', fontsize=8)
            axes[row][0].grid(True, alpha=0.3)

            # 右列：原始数据
            name = f'right_{jtype}'
            ref_raw = np.rad2deg(joint_data_raw[name])
            ref_corr = np.rad2deg(joint_data[name])
            fit = np.rad2deg(joint_fitted[name])
            rmse = np.sqrt(np.mean((ref_corr - fit)**2))

            axes[row][1].plot(t_norm, ref_raw, 'gray', lw=1.5, alpha=0.5, label='Raw')
            axes[row][1].plot(t_norm, ref_corr, 'r-', lw=2.5, label='Corrected')
            axes[row][1].plot(t_norm, fit, 'b--', lw=2, label=f'Fit (RMSE={rmse:.2f}°)')
            axes[row][1].set_title(f'Right {jlabel}')
            axes[row][1].legend(loc='upper right', fontsize=8)
            axes[row][1].grid(True, alpha=0.3)

        axes[-1][0].set_xlabel('Gait Cycle Phase (0→1)')
        axes[-1][1].set_xlabel('Gait Cycle Phase (0→1)')
        plt.suptitle(
            f'CPG Harmonic Fit with Zero Offset Correction — Speed {speed_label}  '
            f'(ω={omega:.2f} rad/s, T={T_cycle:.3f}s)',
            fontsize=14, fontweight='bold')
    else:
        # 普通模式：只显示拟合结果
        fig, axes = plt.subplots(3, 2, figsize=(16, 10), sharex=True)
        joint_types = ['hip3', 'knee', 'ankle']
        joint_labels = ['Hip', 'Knee', 'Ankle']

        for col, side in enumerate(['left', 'right']):
            for row, (jtype, jlabel) in enumerate(zip(joint_types, joint_labels)):
                name = f'{side}_{jtype}'
                ref = np.rad2deg(joint_data[name])
                fit = np.rad2deg(joint_fitted[name])
                rmse = np.sqrt(np.mean((ref - fit)**2))

                axes[row][col].plot(t_norm, ref, 'r-', lw=2.5, label='Human Reference')
                axes[row][col].plot(t_norm, fit, 'b--', lw=2, label=f'CPG Fit (RMSE={rmse:.2f}°)')
                axes[row][col].set_ylabel('Angle (deg)')
                axes[row][col].set_title(f'{side.capitalize()} {jlabel}')
                axes[row][col].legend(loc='upper right')
                axes[row][col].grid(True, alpha=0.3)

        axes[-1][0].set_xlabel('Gait Cycle Phase (0→1)')
        axes[-1][1].set_xlabel('Gait Cycle Phase (0→1)')
        plt.suptitle(
            f'CPG Harmonic Fit (L/R Independent) — Speed {speed_label}  '
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
    parser.add_argument('--correct-offset', action='store_true',
                        help='使用站立段数据校正零位偏移')
    parser.add_argument('--fit-method', type=str, default='ols',
                        choices=['ols', 'wls-gradient', 'wls-peak'],
                        help='拟合方法: ols(普通最小二乘), wls-gradient(梯度加权), wls-peak(峰值加权) (默认: ols)')

    args = parser.parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    speed_map = {'0x5': '0.5', '1': '1.0', '1x5': '1.5'}
    speed_label = speed_map.get(args.speed, args.speed)

    # ----------------------------------------------------------
    # 定位两个数据文件
    # ----------------------------------------------------------
    base_dir = os.path.join(script_dir, '..', '关节角度')
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
    abs_left, abs_right = load_gait_data(abs_path)
    abs_hip = abs_left['hip']  # 用左腿髋关节提取步频
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
    # 步骤 2: 计算零位偏移（可选）
    # ==========================================================
    offsets = {}  # 零位偏移量（度）
    if args.correct_offset:
        print(f"\n--- 步骤 2: 计算零位偏移 (使用站立段数据) ---")
        standing_samples = int(args.fs * 1)  # 1秒

        def find_standing_offset(joint_data, fs, duration=1.0):
            """找到第一个非零样本，从那里开始取站立数据计算offset"""
            nonzero_idx = np.nonzero(joint_data)[0]
            if len(nonzero_idx) == 0:
                print(f"    ⚠️  警告: 数据全为零，返回offset=0")
                return 0.0
            start_idx = nonzero_idx[0]
            end_idx = min(start_idx + int(fs * duration), len(joint_data))
            return joint_data[start_idx:end_idx].mean()

        offsets = {
            'left_hip3': find_standing_offset(abs_left['hip'], args.fs),
            'left_knee': find_standing_offset(abs_left['knee'], args.fs),
            'left_ankle': find_standing_offset(abs_left['ankle'], args.fs),
            'right_hip3': find_standing_offset(abs_right['hip'], args.fs),
            'right_knee': find_standing_offset(abs_right['knee'], args.fs),
            'right_ankle': find_standing_offset(abs_right['ankle'], args.fs),
        }

        print(f"  左腿站立角度:")
        print(f"    Hip:   {offsets['left_hip3']:.2f}°")
        print(f"    Knee:  {offsets['left_knee']:.2f}°")
        print(f"    Ankle: {offsets['left_ankle']:.2f}°")
        print(f"  右腿站立角度:")
        print(f"    Hip:   {offsets['right_hip3']:.2f}°")
        print(f"    Knee:  {offsets['right_knee']:.2f}°")
        print(f"    Ankle: {offsets['right_ankle']:.2f}°")
    else:
        print(f"\n--- 步骤 2: 跳过零位校正 (使用 --correct-offset 启用) ---")

    # ==========================================================
    # 步骤 3: 从周期平均数据拟合谐波权重（左右腿独立）
    # ==========================================================
    print(f"\n--- 步骤 3: 拟合谐波权重 (左右腿独立) ---")
    cycle_left, cycle_right = load_gait_data(cycle_path)
    N_cycle = len(cycle_left['hip'])
    print(f"  周期数据: {N_cycle} 点/周期")

    # 用真实 omega 在极限环上采样 N_cycle 个等间隔点
    X, Y = sample_cpg_one_cycle(N_cycle, omega, mu=args.mu, gamma=args.gamma)

    # 验证极限环半径
    r_check = np.sqrt(X[0]**2 + Y[0]**2)
    print(f"  极限环半径: r = {r_check:.4f} (期望: {np.sqrt(args.mu):.4f})")

    # 谐波阶数配置（左右腿相同）
    # 高速时波形更复杂，需要更高阶谐波
    speed_val = float(speed_label)
    hip_harmonics = 4 if speed_val >= 1.0 else 3   # 髋关节：高速4阶，低速3阶
    knee_harmonics = 4 if speed_val >= 1.0 else 3  # 膝关节：高速4阶，低速3阶
    joint_n_h = {
        'hip3':  hip_harmonics,   # 常数 + 基频 + 二倍频 + (三倍频)
        'knee':  knee_harmonics,  # 常数 + 基频 + 二倍频 + (三倍频)
        'ankle': 4,               # 常数 + 基频 + 二倍频 + 三倍频 (固定)
    }

    # 对左右腿分别进行 OLS 拟合
    joint_params = {}  # key: 'left_hip3', 'right_knee', etc.
    joint_data = {}    # 校正后的参考数据（弧度）
    joint_data_raw = {}  # 校正前的原始数据（弧度）
    joint_fitted = {}
    joint_limits = {}
    limit_margin = 0.10

    for side, side_data in [('left', cycle_left), ('right', cycle_right)]:
        # 原始数据（弧度）
        hip_raw = np.deg2rad(side_data['hip'])
        knee_raw = -np.deg2rad(side_data['knee'])
        ankle_raw = np.deg2rad(side_data['ankle'])

        # 零位校正后的数据（弧度）
        if args.correct_offset:
            hip_rad = np.deg2rad(side_data['hip'] - offsets[f'{side}_hip3'])
            knee_rad = -np.deg2rad(side_data['knee'] - offsets[f'{side}_knee'])
            ankle_rad = np.deg2rad(side_data['ankle'] - offsets[f'{side}_ankle'])
        else:
            hip_rad = hip_raw
            knee_rad = knee_raw
            ankle_rad = ankle_raw

        # 保存数据供绘图使用
        joint_data_raw[f'{side}_hip3'] = hip_raw
        joint_data_raw[f'{side}_knee'] = knee_raw
        joint_data_raw[f'{side}_ankle'] = ankle_raw

        joint_data[f'{side}_hip3'] = hip_rad
        joint_data[f'{side}_knee'] = knee_rad
        joint_data[f'{side}_ankle'] = ankle_rad

        side_joints = {
            f'{side}_hip3':  hip_rad,
            f'{side}_knee':  knee_rad,
            f'{side}_ankle': ankle_rad,
        }

        is_right = (side == 'right')
        print(f"\n  === {side.upper()} 腿 ===")
        for name, rad_data in side_joints.items():
            joint_type = name.split('_')[1]  # 'hip3', 'knee', 'ankle'
            n_h = joint_n_h[joint_type]

            # 选择拟合方法
            if args.fit_method == 'wls-gradient':
                w_offset, w_x, w_y, res = fit_harmonics_wls(X, Y, rad_data, n_h, flip_odd=is_right, weight_mode='gradient')
            elif args.fit_method == 'wls-peak':
                w_offset, w_x, w_y, res = fit_harmonics_wls(X, Y, rad_data, n_h, flip_odd=is_right, weight_mode='peak')
            else:  # ols
                w_offset, w_x, w_y, res = fit_harmonics_ols(X, Y, rad_data, n_h, flip_odd=is_right)

            joint_params[name] = {
                'n_h': n_h, 'w_offset': w_offset,
                'w_x': w_x, 'w_y': w_y, 'residual': res,
            }
            joint_fitted[name] = reconstruct_signal(X, Y, w_offset, w_x, w_y, n_h, flip_odd=is_right)

            # 安全限位
            d_min = float(rad_data.min())
            d_max = float(rad_data.max())
            span = d_max - d_min
            lo = round(d_min - limit_margin * span, 2)
            hi = round(d_max + limit_margin * span, 2)
            joint_limits[name] = (lo, hi)

            print(f"\n  [{name}] (n_h={n_h})")
            print(f"    w_offset = {w_offset:+.4f} rad ({np.rad2deg(w_offset):+.2f}°)")
            for i in range(1, n_h):
                print(f"    w_x[{i}] = {w_x[i]:+.4f}   w_y[{i}] = {w_y[i]:+.4f}")
            print(f"    RMSE = {np.rad2deg(res):.3f}°")
            print(f"    limit: [{lo}, {hi}] rad [{np.rad2deg(lo):.1f}°, {np.rad2deg(hi):.1f}°]")

    # ==========================================================
    # 步骤 4: 生成 YAML
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
    # 步骤 5: 绘图
    # ==========================================================
    if not args.no_plot:
        t_norm = np.linspace(0, 1, N_cycle, endpoint=False)
        plot_path = os.path.join(script_dir, f'gait_fit_S{args.speed}_T{args.trial}.png')
        plot_results(t_norm, joint_data, joint_fitted,
                     omega, T_cycle, speed_label, plot_path,
                     joint_data_raw=joint_data_raw if args.correct_offset else None,
                     correct_offset=args.correct_offset)


if __name__ == '__main__':
    main()