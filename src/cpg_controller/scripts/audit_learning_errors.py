#!/usr/bin/env python3
"""Recompute angle errors directly from actual input CSV, without the scorer.

This audit does not run or change the learner. No time-shift search, amplitude
normalization, phase alignment or smoothing precedes the reported angle RMSE.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

JOINTS = ['left_hip3', 'left_knee', 'left_ankle', 'right_hip3', 'right_knee', 'right_ankle']
LABELS = ['L hip', 'L knee', 'L ankle', 'R hip', 'R knee', 'R ankle']
MODES = ['frozen', 'sync', 'full', 'joint', 'no_events', 'joint_no_events']


def phase_truth(t, case, omega):
    phase = omega * t
    if case in ['phase_shift', 'combined']:
        phase += .7
    if case in ['frequency_step', 'combined']:
        phase += .2 * omega * np.maximum(t - 12, 0)
    if case == 'frequency_ramp':
        u = np.clip(t - 12, 0, 12)
        phase += .2 * omega * (u*u/24 + np.maximum(t - 24, 0))
    if case == 'runtime_phase_offset':
        u = np.clip((t - 12)/.6, 0, 1)
        phase += .7 * u**3 * (10 - 15*u + 6*u*u)
    return phase


def matrix(rows):
    return np.column_stack([rows['q' + str(j)] for j in range(6)])


def window_stats(t, error, phase_error, mask, rom):
    e = error[mask]
    p = phase_error[mask]
    times = t[mask]
    per_joint = np.sqrt(np.mean(e*e, axis=0))
    rms = float(np.sqrt(np.mean(e*e)))
    flat = np.unravel_index(np.argmax(np.abs(e)), e.shape)
    k = int(np.argmax(np.abs(p)))
    return {
        'sample_count': int(mask.sum()), 'six_joint_angle_rmse_rad': rms,
        'six_joint_angle_rmse_deg': float(np.rad2deg(rms)),
        'joint_angle_rmse_deg': dict(zip(JOINTS, np.rad2deg(per_joint).tolist())),
        'joint_angle_nrmse_original_template_rom_pct': dict(zip(JOINTS, (100*per_joint/rom).tolist())),
        'joint_peak_abs_error_deg': dict(zip(JOINTS, np.rad2deg(np.max(np.abs(e), axis=0)).tolist())),
        'phase_rmse_deg': float(np.rad2deg(np.sqrt(np.mean(p*p)))),
        'phase_peak_abs_deg': float(np.rad2deg(np.max(np.abs(p)))),
        'max_instant_six_joint_rms_deg': float(np.rad2deg(np.max(np.sqrt(np.mean(e*e, axis=1))))),
        'worst_joint': JOINTS[flat[1]], 'worst_joint_time_s': float(times[flat[0]]),
        'worst_joint_abs_error_deg': float(np.rad2deg(abs(e[flat]))),
        'phase_peak_time_s': float(times[k]),
        'joint_error_at_phase_peak_deg': dict(zip(JOINTS, np.rad2deg(e[k]).tolist()))}


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path(tempfile.gettempdir())/'cpg_experiments/learning_strategy_comparison')
    parser.add_argument('--output', type=Path, default=Path(tempfile.gettempdir())/'cpg_experiments/learning_error_audit')
    args = parser.parse_args()
    if not (args.data/'metrics.json').is_file():
        parser.error('Missing experiment traces; run compare_learning_strategies.py first or pass --data')
    args.output.mkdir(parents=True, exist_ok=True)
    original = json.loads((args.data/'metrics.json').read_text())
    cfg_path = root/'config/cpg.yaml'
    gait = yaml.safe_load(cfg_path.read_text())['adaptive_cpg_node']['ros__parameters']['gait_library']['flat_walk']
    omega = gait['omega']
    expected = []
    for name in JOINTS:
        model = gait['joints'][name]
        row = [model['w_offset']]
        for n in range(1, 4):
            row += [model['w_x'][n] if n < len(model['w_x']) else 0,
                    model['w_y'][n] if n < len(model['w_y']) else 0]
        expected.append(row)
    prior = np.loadtxt(args.data/'original_flat_walk.txt', skiprows=1)
    assert np.max(np.abs(prior - expected)) < 1e-11
    assert hashlib.sha256(cfg_path.read_bytes()).hexdigest() == original['source_template_sha256']
    phase_grid = np.linspace(0, 2*np.pi, 10001, endpoint=False)
    cycle = np.broadcast_to(prior[:, 0], (len(phase_grid), 6)).copy()
    for n in range(1, 4):
        cycle += np.cos(n*phase_grid[:, None])*prior[:, 2*n-1] + np.sin(n*phase_grid[:, None])*prior[:, 2*n]
    template_rom = np.ptp(cycle, axis=0)
    audit = {'prior_is_original_flat_walk': True,
             'nrmse_denominator_original_template_rom_rad': dict(zip(JOINTS, template_rom.tolist())),
             'angle_definition': 'sqrt(sum over time and six joints of squared radian position residual / (N*6))',
             'phase_definition': 'wrapped shared gait-phase difference, not joint angle difference',
             'no_posthoc_time_or_phase_alignment': True,
             'combined_note': 'Direct input audit scores historical published reconstruction against noisy measurement at source stamp. Original metrics score current reconstruction against clean truth, including times with no input.',
             'cases': {}}
    plotted = {}
    largest_difference = 0
    for case, saved_modes in original['cases'].items():
        raw = np.genfromtxt(args.data/case/'input.csv', delimiter=',', names=True)
        measurements = raw[raw['stamp'] > 0]
        mq = matrix(measurements)
        measured_rom = np.ptp(mq, axis=0)
        rom = template_rom
        phi0 = omega * (measurements['stamp'] - 1)
        baseline = np.broadcast_to(prior[:, 0], mq.shape).copy()
        for n in range(1, 4):
            baseline += np.cos(n*phi0[:, None])*prior[:, 2*n-1] + np.sin(n*phi0[:, None])*prior[:, 2*n]
        evidence = {
            'input_columns': list(raw.dtype.names), 'source_samples': len(measurements),
            'ideal_event_count': int(np.sum(raw['event_foot'] >= 0)),
            'delay_min_max_s': [float(np.min(measurements['now']-measurements['stamp'])),
                                float(np.max(measurements['now']-measurements['stamp']))],
            'input_minus_original_fixed_clock_rms_rad': dict(zip(JOINTS, np.sqrt(np.mean((mq-baseline)**2, axis=0)).tolist())),
            'joint_input_rom_rad': dict(zip(JOINTS, measured_rom.tolist())), 'modes': {}}
        start = original['change_start_s'][case]
        for mode in MODES:
            trace = np.genfromtxt(args.data/case/(mode+'.csv'), delimiter=',', names=True)
            # Align only by known source timestamps, not by signal matching.
            dt = np.median(np.diff(trace['now']))
            index = np.rint((measurements['stamp']-trace['now'][0])/dt).astype(int)
            assert index.min() >= 0 and index.max() < len(trace)
            assert np.max(np.abs(trace['now'][index]-measurements['stamp'])) < 1e-8
            t = measurements['stamp']-1
            q = matrix(trace)[index]
            error = q-mq
            ph = trace['phase'][index]-phase_truth(t, case, omega)
            ph = np.arctan2(np.sin(ph), np.cos(ph))
            stats = {}
            for name, mask in [('first_1s_after_change', (t>=start)&(t<start+1)),
                               ('first_5s_after_change', (t>=start)&(t<start+5)),
                               ('final_8s', t>=42)]:
                stats[name] = window_stats(t, error, ph, mask, rom)
            if case != 'combined':
                for name, metric in [('first_5s_after_change', 'change_window_5s_rmse_rad'),
                                     ('final_8s', 'final_rmse_rad')]:
                    difference = abs(stats[name]['six_joint_angle_rmse_rad']-saved_modes[mode][metric])
                    largest_difference = max(largest_difference, difference)
                    assert difference < 1e-9, (case, mode, name, difference)
            evidence['modes'][mode] = stats
            if case in ['frequency_step', 'runtime_phase_offset'] and mode == 'full':
                plotted[case] = (t, q, mq, error, ph, stats)
                window = (t>=11)&(t<=18)
                values = np.column_stack([t[window], np.rad2deg(ph[window]),
                                          np.rad2deg(error[window]), np.rad2deg(q[window]),
                                          np.rad2deg(mq[window])])
                header = 'time_s,shared_phase_error_deg,' + ','.join(
                    [prefix+name+'_deg' for prefix in ['error_', 'prediction_', 'measurement_'] for name in JOINTS])
                np.savetxt(args.output/(case+'_pointwise.csv'), values, delimiter=',', header=header, comments='', fmt='%.10g')
        audit['cases'][case] = evidence
    audit['max_difference_from_report_nine_noiseless_cases_rad'] = largest_difference
    (args.output/'audit.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2)+'\n')
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(4, 2, figsize=(13, 11), sharex=True)
    for col, case in enumerate(['frequency_step', 'runtime_phase_offset']):
        t, q, target, error, phase, stats = plotted[case]
        view = (t>=11.5)&(t<=17)
        worst = JOINTS.index(stats['first_5s_after_change']['worst_joint'])
        axes[0, col].plot(t[view], np.rad2deg(target[view, worst]), 'k', label=LABELS[worst]+' measured input')
        axes[0, col].plot(t[view], np.rad2deg(q[view, worst]), color='#2077a5', label='CPG raw reconstruction')
        axes[0, col].set_ylabel('Joint angle [deg]')
        axes[0, col].set_title('Frequency +20%' if col == 0 else 'Shared phase +0.7 rad over 0.6 s')
        axes[1, col].plot(t[view], np.rad2deg(phase[view]), color='#973f42', label='Shared gait-phase error')
        axes[1, col].set_ylabel('Gait-phase error [deg]')
        for j in range(6):
            axes[2, col].plot(t[view], np.rad2deg(error[view, j]), label=LABELS[j])
        axes[2, col].set_ylabel('Individual joint error [deg]')
        instant = np.rad2deg(np.sqrt(np.mean(error**2, axis=1)))
        rolling = np.sqrt(np.convolve(instant**2, np.ones(100)/100, 'valid'))
        axes[3, col].plot(t[view], instant[view], color='#2077a5', label='Instantaneous six-joint RMS')
        rv = (t[99:]>=11.5)&(t[99:]<=17)
        axes[3, col].plot(t[99:][rv], rolling[rv], color='#d08c2a', label='Causal 1 s six-joint RMS')
        axes[3, col].hlines(stats['first_5s_after_change']['six_joint_angle_rmse_deg'],12,17,
                           color='#973f42', linestyle='--', label='Six-joint RMSE over [12,17) s')
        axes[3, col].set_ylabel('Aggregate angle error [deg]')
        axes[3, col].set_xlabel('Time [s]')
        for row in range(4):
            axes[row, col].axvline(12, color='.6', lw=.8, ls=':')
            axes[row, col].grid(alpha=.2)
            axes[row, col].legend(fontsize=7, ncol=3 if row == 2 else 1, loc='best')
    fig.suptitle('Different quantities: shared gait-phase error vs joint-angle error\nStaged learner; actual CSV measurements, no post-hoc signal alignment')
    fig.tight_layout(rect=(0,0,1,.95))
    fig.savefig(args.output/'transient_error_units.png', dpi=165)
    fig.savefig(args.output/'transient_error_units.pdf')
    plt.close(fig)
    lines = ['# CPG 误差与测量输入核查', '',
             '未重新运行或修改学习器；直接读取实际 input.csv 和输出 CSV，按已知源时间戳重算，未做事后相位/时移匹配。',
             f'9个无噪声场景、每场景6种方法的前5s/末8s角度RMSE与原报告一致，最大差值 {largest_difference:.3g} rad。', '',
             '## 原报告混在一起展示的四件事', '',
             '步态相位误差与关节角误差不是同一个物理量；相位峰值也不是相位RMSE。',
             '六关节跨时间的RMSE既平均了短暂峰值，也平均了不同关节，不能代表最坏单关节。',
             '原指标用未限幅的模型重构，真实控制参考还需考虑限幅；本试验不含电机或阻抗闭环。', '',
             '| 门控方案、12–17s | 相位峰值 | 相位RMSE | 六关节角RMSE | 六关节瞬时RMS峰值 | 单关节最大绝对误差 |',
             '|---|---:|---:|---:|---:|---:|']
    for case in ['frequency_step','runtime_phase_offset']:
        s = plotted[case][-1]['first_5s_after_change']
        lines.append('| '+case+' | '+' | '.join(f'{s[k]:.3f}°' for k in [
            'phase_peak_abs_deg','phase_rmse_deg','six_joint_angle_rmse_deg',
            'max_instant_six_joint_rms_deg','worst_joint_abs_error_deg'])+' |')
    lines += ['', '## 学习器实际输入', '',
              '原始 flat_walk 仅用于初始系数与基频；已逐项核对 original_flat_walk.txt 与原 cpg.yaml 一致。',
              '六关节变形角度以 input.csv 的 q0…q5 和源时间 stamp 输入 observe()。stamp=0 的500Hz占位行不会作为零测量输入。',
              '另有合成真值过0/π生成的理想离散事件，作为 event() 输入。未直接输入连续真值相位、真值频率或变形系数，但这仍是额外的理想锚点。',
              'combined 含30ms延迟、噪声、单关节异常和断流。原报告相对干净真值评分；本核查另相对带噪实测角度评分，二者应分开解释。', '',
              '## 为什么稳态结果很小', '',
              '多数输入无噪声，并且变形前后都在学习器能够表达的三阶Fourier模型内；再加上理想事件，稳态接近零可以出现。',
              '这只证明模型内跟随与代码通路可行，不足以证明真实人体测量、未知波形或闭环控制具有同样精度。',
              '后续报告应同时列出逐关节RMSE、最坏关节峰值、相同窗口的相位RMSE和NRMSE，避免只用小的总体RMSE判断暂态。',
              '本核查的NRMSE统一以原始模板完整周期的逐关节活动范围为分母，固定于所有场景，避免异常样本扩大分母而显得误差更小。', '',
              '逐点记录见 frequency_step_pointwise.csv 和 runtime_phase_offset_pointwise.csv；audit.json保留所有场景、各方法的核查结果。']
    (args.output/'README.md').write_text('\n'.join(lines)+'\n')
    print('Verified report values; max absolute difference:', largest_difference)
    for case in plotted:
        print(case, audit['cases'][case]['modes']['full']['first_5s_after_change'])
    print('Outputs:', args.output)


if __name__ == '__main__':
    main()
