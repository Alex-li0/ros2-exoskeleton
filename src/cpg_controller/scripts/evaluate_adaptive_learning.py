#!/usr/bin/env python3
"""Causal six-joint experiment against the compiled shared adaptive CPG core.

The learner receives only original priors, delayed joint measurements and optional
synthetic events. Truth phase/frequency and deformation parameters stay in this
scoring process. No ROS control topic, hardware interface or motor is opened.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

JOINTS = ['left_hip3', 'left_knee', 'left_ankle',
          'right_hip3', 'right_knee', 'right_ankle']
LABELS = ['Left hip', 'Left knee', 'Left ankle', 'Right hip', 'Right knee', 'Right ankle']
CASES = ['unchanged', 'phase_shift', 'frequency_step', 'shape_deformation', 'frequency_ramp', 'combined']
MODES = ['frozen', 'sync', 'full', 'no_events']
COLORS = {'frozen': '#8c8c8c', 'sync': '#dd8b24', 'full': '#1d80b4', 'no_events': '#9467bd'}


def template(path):
    config = yaml.safe_load(path.read_text())['adaptive_cpg_node']['ros__parameters']['gait_library']['flat_walk']
    weights = np.zeros((6, 7))
    for j, name in enumerate(JOINTS):
        model = config['joints'][name]
        weights[j, 0] = model['w_offset']
        for n in range(1, min(4, model['num_harmonics'])):
            weights[j, 2*n-1] = model['w_x'][n]
            weights[j, 2*n] = model['w_y'][n]
    return config['omega'], weights


def truth(times, case, omega, prior):
    t = np.asarray(times)
    phase = omega * t
    frequency = np.full(t.shape, omega)
    if case in ['phase_shift', 'combined']:
        phase += .7
    if case in ['frequency_step', 'combined']:
        phase += .2 * omega * np.maximum(t-12, 0)
        frequency += np.where(t >= 12, .2*omega, 0)
    if case == 'frequency_ramp':
        elapsed = np.clip(t-12, 0, 12)
        phase += .2 * omega * (elapsed**2/24 + np.maximum(t-24, 0))
        frequency += .2 * omega * elapsed/12
    shaped = case in ['shape_deformation', 'combined']
    u = np.clip((t-8)/4, 0, 1)
    blend = u**3 * (10-15*u+6*u**2) if shaped else np.zeros_like(t)
    amplitude = np.array([1.18, .84, 1.20, .88, 1.14, .82])
    offset = np.array([.025, -.035, .008, -.020, .030, -.008])
    shift = np.array([.08, -.06, .10, -.07, .05, -.08])
    q = np.zeros((t.size, 6))
    weights = np.broadcast_to(prior, (t.size, 6, 7)).copy()
    for j in range(6):
        weights[:, j, 0] += blend*offset[j]
        for n in range(1, 4):
            angle = n*shift[j]*blend
            scale = 1+blend*(amplitude[j]-1)
            # Different higher harmonic changes cannot be absorbed by phase/scale alone.
            if n == 2:
                scale *= 1+blend*(.15 if j % 2 == 0 else -.2)
            if n == 3:
                scale *= 1+blend*.15
            a, b = prior[j, 2*n-1], prior[j, 2*n]
            weights[:, j, 2*n-1] = scale*(a*np.cos(angle)+b*np.sin(angle))
            weights[:, j, 2*n] = scale*(-a*np.sin(angle)+b*np.cos(angle))
            if shaped and n == 3:
                weights[:, j, 2*n-1] += blend*(.008 if j < 2 else -.005)
        q[:, j] = weights[:, j, 0]
        for n in range(1, 4):
            q[:, j] += weights[:, j, 2*n-1]*np.cos(n*phase)+weights[:, j, 2*n]*np.sin(n*phase)
    return phase, frequency, q, weights


def create_input(folder, case, duration, omega, prior, seed):
    dt = .002
    t = np.arange(round(duration/dt)+1)*dt
    phase, frequency, clean, _ = truth(t, case, omega, prior)
    rng = np.random.default_rng(seed)
    measured = clean.copy()
    if case == 'combined':
        measured += rng.normal(size=measured.shape)*np.array([.002, .003, .001, .002, .003, .001])
        measured[np.abs(t-32)<.001, 0] += .7
    delay = .03 if case == 'combined' else 0.
    measurement_queue, event_queue = deque(), deque()
    event_ids = [0, 0]
    path = folder / 'input.csv'
    with path.open('w') as handle:
        handle.write('now,stamp,'+','.join('q'+str(j) for j in range(6))+
                     ',event_stamp,event_foot,event_id,event_phase\n')
        for k, now in enumerate(t):
            dropout = case == 'combined' and 24 <= now < 24.5
            if k % 5 == 0 and not dropout:
                measurement_queue.append((now+delay, now+1, measured[k]))
            # Crossing time is interpolated, events only delivered at/after occurrence.
            # Zero and pi are synthetic coordinates; no human IC calibration is claimed.
            if k and not dropout:
                before, after = int(np.floor(phase[k-1]/np.pi)), int(np.floor(phase[k]/np.pi))
                if after > before:
                    p = after*np.pi
                    crossing = t[k-1]+(p-phase[k-1])/(phase[k]-phase[k-1])*dt
                    foot = after % 2
                    event_ids[foot] += 1
                    event_queue.append((crossing+delay, crossing+1, foot,
                                        event_ids[foot], foot*np.pi))
            stamp, q = 0., np.zeros(6)
            if measurement_queue and measurement_queue[0][0] <= now+1e-10:
                _, stamp, q = measurement_queue.popleft()
            es, foot, eid, ep = 0., -1, 0, 0.
            if event_queue and event_queue[0][0] <= now+1e-10:
                _, es, foot, eid, ep = event_queue.popleft()
            values = [now+1, stamp, *q, es, foot, eid, ep]
            handle.write(','.join(format(float(v), '.12g') for v in values)+'\n')
    return path


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def summary(trace, case, omega, prior, limits):
    time = trace['now']-1
    phase, frequency, clean, truth_weights = truth(time, case, omega, prior)
    q = np.column_stack([trace['q'+str(j)] for j in range(6)])
    velocity = np.column_stack([trace['qd'+str(j)] for j in range(6)])
    error = q-clean
    phase_error = wrap(trace['phase']-phase)
    window = time >= time[-1]-8
    initial = (time >= 0) & (time < 4)
    clipped = np.clip(q, limits[:, 0], limits[:, 1])
    truth_clipped = np.clip(clean, limits[:, 0], limits[:, 1])
    result = {'reference_clipped_fraction': float(np.mean(q != clipped)),
              'truth_outside_original_limits_fraction': float(np.mean(clean != truth_clipped)),
              'final_clamped_vs_raw_truth_rmse_rad': float(np.sqrt(np.mean((clipped[window]-clean[window])**2))),
              'final_clamped_vs_clamped_truth_rmse_rad': float(np.sqrt(np.mean((clipped[window]-truth_clipped[window])**2)))}
    for name, mask in [('initial', initial), ('final', window)]:
        per_joint = np.sqrt(np.mean(error[mask]**2, axis=0))
        result[name+'_rmse_rad'] = float(np.sqrt(np.mean(error[mask]**2)))
        result[name+'_joint_rmse_rad'] = dict(zip(JOINTS, per_joint.tolist()))
        result[name+'_phase_rmse_deg'] = float(np.rad2deg(np.sqrt(np.mean(phase_error[mask]**2))))
        result[name+'_base_frequency_bias_pct'] = float(np.mean((trace['omega_base'][mask]-frequency[mask])/frequency[mask])*100)
        result[name+'_rate_frequency_bias_pct'] = float(np.mean((trace['phase_rate'][mask]-frequency[mask])/frequency[mask])*100)
    result['full_phase_p95_deg'] = float(np.rad2deg(np.percentile(np.abs(phase_error), 95)))
    result['max_reference_speed_rad_s'] = float(np.max(np.abs(velocity)))
    # Exclude isolated state/input-update boundaries for the bulk FD statistic;
    # also report max to expose discontinuities rather than hiding them in P99.
    numerical = (q[2:]-q[:-2])/(time[2:]-time[:-2])[:, None]
    discrepancy = np.abs(numerical-velocity[1:-1])
    result['velocity_fd_p99_rad_s'] = float(np.percentile(discrepancy, 99))
    result['velocity_fd_max_rad_s'] = float(discrepancy.max())
    locked = (trace['state'][:-2] == 2) & (trace['state'][1:-1] == 2) & (trace['state'][2:] == 2)
    result['locked_velocity_fd_max_rad_s'] = float(discrepancy[locked].max()) if np.any(locked) else None
    result['locked_velocity_fd_p99_rad_s'] = float(np.percentile(discrepancy[locked], 99)) if np.any(locked) else None
    result['shape_active_fraction'] = float(np.mean(trace['shape_active']))
    states = trace['state'].astype(int)
    result['states_seen'] = sorted(np.unique(states).tolist())
    result['state_transitions'] = [[float(time[k]), int(states[k])] for k in range(len(states))
                                   if k == 0 or states[k] != states[k-1]]
    expected_step = np.diff(time) * (trace['phase_rate'][1:]+trace['phase_rate'][:-1])/2
    phase_splice = np.diff(trace['phase'])-expected_step
    result['capture_phase_splices'] = [
        {'time_s': float(time[k+1]), 'excess_phase_change_deg': float(np.rad2deg(phase_splice[k]))}
        for k in np.flatnonzero(np.abs(phase_splice) > np.deg2rad(.5))]
    coefficient_change = np.column_stack([trace[name]-trace[name][0] for name in trace.dtype.names
                                          if name.startswith(('a', 'b', 'c'))])
    result['max_coefficient_change_rad'] = float(np.max(np.abs(coefficient_change)))
    final_weights = np.array([[trace['b'+str(j)][-1]] +
                              [trace[prefix+str(j)+str(n)][-1]
                               for n in range(1, 4) for prefix in ['a', 'c']]
                             for j in range(6)])
    learned_beta = np.arctan2(final_weights[:, 1], final_weights[:, 2])
    truth_beta = np.arctan2(truth_weights[-1, :, 1], truth_weights[-1, :, 2])
    relative_error = wrap((learned_beta-learned_beta[0])-(truth_beta-truth_beta[0]))
    result['final_relative_fundamental_phase_error_deg'] = dict(zip(
        JOINTS, np.rad2deg(relative_error).tolist()))
    result['final_relative_fundamental_phase_rmse_deg'] = float(np.rad2deg(
        np.sqrt(np.mean(relative_error[1:]**2))))
    phase_good = np.abs(phase_error) < np.deg2rad(10)
    rate_good = np.abs((trace['omega_base']-frequency)/frequency) < .03
    good = phase_good & rate_good
    first = None
    # First two-second contiguous interval within phase/frequency tolerances.
    width = round(2/np.median(np.diff(time)))
    if good.size >= width:
        rolling = np.convolve(good.astype(int), np.ones(width, int), 'valid')
        candidates = np.flatnonzero(rolling == width)
        if candidates.size:
            first = float(time[candidates[0]])
    result['first_2s_lock_time_s'] = first
    return result


def plot_case(folder, case, traces, omega, prior, limits):
    full = traces['full']
    t = full['now']-1
    phase, frequency, clean, _ = truth(t, case, omega, prior)
    fig, axes = plt.subplots(3, 2, figsize=(13, 8), sharex=True)
    view = (t >= max(0, t[-1]-5))
    for j, ax in zip([0, 3, 1, 4, 2, 5], axes.flat):
        ax.plot(t[view], clean[view, j], 'k', lw=1.7, label='Synthetic clean measurement')
        for mode in ['frozen', 'sync', 'full']:
            ax.plot(t[view], traces[mode]['q'+str(j)][view], color=COLORS[mode], lw=1.1,
                    alpha=.9, label={'frozen': 'Original fixed CPG', 'sync': 'Phase + frequency',
                                     'full': 'Phase + frequency + shape'}[mode])
        clipped = np.clip(full['q'+str(j)], limits[j, 0], limits[j, 1])
        if np.any(clipped[view] != full['q'+str(j)][view]):
            ax.plot(t[view], clipped[view], color='#b34a55', ls=':', lw=1.3,
                    label='Full, original limits applied')
        ax.axhline(limits[j, 0], color='#9d9d9d', ls='--', lw=.6)
        ax.axhline(limits[j, 1], color='#9d9d9d', ls='--', lw=.6)
        ax.set_title(LABELS[j]); ax.set_ylabel('Angle [rad]'); ax.grid(alpha=.2)
    axes[2, 0].set_xlabel('Time [s]'); axes[2, 1].set_xlabel('Time [s]')
    axes[0, 0].legend(fontsize=7)
    fig.suptitle(case+' — final six-joint tracking (shared phase)')
    fig.tight_layout(); fig.savefig(folder/'six_joint_tracking.png', dpi=160); plt.close(fig)
    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    axes[0].plot(t, frequency/(2*np.pi), 'k', lw=2, label='Truth stride frequency')
    for mode, trace in traces.items():
        axes[0].plot(t, trace['omega_base']/(2*np.pi), color=COLORS[mode], label=mode)
        axes[1].plot(t, np.rad2deg(wrap(trace['phase']-phase)), color=COLORS[mode], label=mode)
        q = np.column_stack([trace['q'+str(j)] for j in range(6)])
        rmse = np.sqrt(np.mean((q-clean)**2, axis=1))
        width = 100
        smoothed = np.convolve(rmse, np.ones(width)/width, mode='valid')
        axes[2].plot(t[width-1:], smoothed, color=COLORS[mode], label=mode)
    zoom = axes[1].inset_axes([.60, .10, .38, .40])
    for mode in ['sync', 'full', 'no_events']:
        zoom.plot(t, np.rad2deg(wrap(traces[mode]['phase']-phase)), color=COLORS[mode], lw=.7)
    zoom.set_ylim(-15, 15)
    zoom.set_title('Learners: phase error zoom', fontsize=8)
    zoom.tick_params(labelsize=7)
    zoom.grid(alpha=.2)
    axes[3].plot(t, full['shape_active'], color=COLORS['full'], label='Shape update enabled')
    axes[3].plot(t, full['state'], color='#466242', alpha=.6, label='Learner state')
    for ax, label in zip(axes, ['Stride frequency [Hz]', 'Circular phase error [deg]',
                               'Six-joint RMS error [rad]', 'Gate / state']):
        ax.set_ylabel(label); ax.grid(alpha=.2); ax.legend(ncol=4, fontsize=8)
    axes[-1].set_xlabel('Time [s]')
    fig.suptitle(case+' — rhythm, error and learning gates')
    fig.tight_layout(); fig.savefig(folder/'learning_overview.png', dpi=160); plt.close(fig)
    return {mode: summary(trace, case, omega, prior, limits) for mode, trace in traces.items()}


def main():
    script_dir = Path(__file__).resolve().parent
    source_root = script_dir.parent
    source_layout = (source_root/'include/cpg_controller/adaptive_cpg.hpp').is_file() and (
        script_dir/'adaptive_learning_probe.cpp').is_file()
    prefix = share = None
    if not source_layout:
        try:
            from ament_index_python.packages import get_package_prefix, get_package_share_directory
            prefix = Path(get_package_prefix('cpg_controller'))
            share = Path(get_package_share_directory('cpg_controller'))
        except (ImportError, LookupError):
            pass
    default_template = source_root/'config/cpg.yaml' if source_layout else (
        share/'config/cpg.yaml' if share else None)
    default_output = Path(tempfile.gettempdir())/'cpg_experiments/adaptive_learning'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--template', type=Path, default=default_template)
    parser.add_argument('--output', type=Path, default=default_output)
    parser.add_argument('--probe', type=Path)
    parser.add_argument('--duration', type=float, default=50.)
    parser.add_argument('--cases', nargs='+', choices=CASES, default=CASES)
    parser.add_argument('--phase-gain', type=float)
    parser.add_argument('--frequency-gain', type=float)
    parser.add_argument('--shape-gain', type=float)
    args = parser.parse_args()
    if args.duration < 36:
        parser.error('duration must be >=36s to include changes and a final settling window')
    if any(x is not None for x in [args.phase_gain, args.frequency_gain, args.shape_gain]) and not all(
            x is not None for x in [args.phase_gain, args.frequency_gain, args.shape_gain]):
        parser.error('Pass all three gain overrides together')
    if args.template is None:
        parser.error('Cannot discover installed configuration; pass --template explicitly')
    args.output.mkdir(parents=True, exist_ok=True)
    probe = args.probe
    if probe is None and source_layout:
        probe = args.output/'adaptive_learning_probe'
        subprocess.run(['g++', '-O2', '-std=c++17', '-I'+str(source_root/'include'),
                        str(script_dir/'adaptive_learning_probe.cpp'), '-o', str(probe)], check=True)
    if probe is None:
        probe = script_dir/'adaptive_learning_probe'
    if not probe.is_file():
        parser.error('Cannot discover compiled C++ probe; pass --probe explicitly')
    probe = probe.resolve()
    header = (source_root/'include/cpg_controller/adaptive_cpg.hpp') if source_layout else (
        prefix/'include/cpg_controller/adaptive_cpg.hpp' if prefix else None)
    probe_source = script_dir/'adaptive_learning_probe.cpp'
    def optional_hash(path):
        return hashlib.sha256(path.read_bytes()).hexdigest() if path and path.is_file() else None
    omega, prior = template(args.template)
    original = yaml.safe_load(args.template.read_text())['adaptive_cpg_node']['ros__parameters']['gait_library']['flat_walk']['joints']
    limits = np.array([[original[name]['limit_min'], original[name]['limit_max']] for name in JOINTS])
    template_path = args.output/'original_flat_walk.txt'
    np.savetxt(template_path, prior, header=format(omega, '.12g'), comments='', fmt='%.12g')
    metrics = {'source_template': str(args.template.resolve()),
               'source_sha256': hashlib.sha256(args.template.read_bytes()).hexdigest(),
               'core_sha256': optional_hash(header),
               'probe_source_sha256': optional_hash(probe_source),
               'probe_binary_sha256': optional_hash(probe),
               'omega_prior_rad_s': omega, 'integration_dt_s': .002, 'measurement_dt_s': .01,
               'trace_dt_s': .01, 'seed': 20261023, 'duration_s': args.duration,
               'gain_overrides': [args.phase_gain, args.frequency_gain, args.shape_gain], 'cases': {}}
    for case in args.cases:
        folder = args.output/case
        folder.mkdir(exist_ok=True)
        input_path = create_input(folder, case, args.duration, omega, prior, 20261023+CASES.index(case))
        traces = {}
        for mode in MODES:
            output = folder/(mode+'.csv')
            command = [str(probe), str(template_path), str(input_path), str(output), mode]
            if args.phase_gain is not None:
                command += [str(args.phase_gain), str(args.frequency_gain), str(args.shape_gain)]
            subprocess.run(command, check=True)
            traces[mode] = np.genfromtxt(output, delimiter=',', names=True)
        metrics['cases'][case] = plot_case(folder, case, traces, omega, prior, limits)
        final = metrics['cases'][case]
        print(case, 'final RMSE:', ', '.join(mode+'='+format(final[mode]['final_rmse_rad'], '.5f')
                                            for mode in MODES), flush=True)
    (args.output/'metrics.json').write_text(json.dumps(metrics, indent=2, ensure_ascii=False)+'\n')
    lines = ['# 多关节 Hopf 在线学习：平地模板变形实验', '',
             '本报告由同一 C++ 学习核心离线运行生成。原始 flat_walk 参数只作为先验；',
             '真实相位、真实频率及变形参数仅用于本脚本评分，不传给学习器。', '',
             '所有六关节共享连续的真实步态相位。合成事件在该相位的 0/π 处生成，',
             '它们是仿真标签，不是已标定的人体初始接触相位。默认积分 500 Hz、测量 100 Hz。',
             '参考轨迹使用归一化相位的 Fourier 读出，无电机、阻抗控制器和实际人体闭环。', '',
             '比较：frozen 为原始频率和形状自由运行；sync 仅学相位/频率；full 增加波形学习；',
             'no_events 移除事件锚定。各组使用同一测量数据与原始先验，未使用变形后的系数初始化。', '',
             '| 场景 | 固定模板末段 RMSE | 节律学习末段 RMSE | 完整学习末段 RMSE | 完整学习相位 RMSE | 基频偏差 |',
             '|---|---:|---:|---:|---:|---:|']
    for case, result in metrics['cases'].items():
        lines.append('| '+case+' | '+' | '.join([
            format(result['frozen']['final_rmse_rad'], '.5f')+' rad',
            format(result['sync']['final_rmse_rad'], '.5f')+' rad',
            format(result['full']['final_rmse_rad'], '.5f')+' rad',
            format(result['full']['final_phase_rmse_deg'], '.2f')+'°',
            format(result['full']['final_base_frequency_bias_pct'], '.2f')+'%'])+' |')
    lines += ['', '去除事件锚定的 no_events 组在部分场景中角度误差更小，不能因此宣称生理相位更准。',
              '相位整体平移与各阶正余弦系数共同旋转能够保持相同角度读出；',
              '必须结合 metrics.json 中独立的真实相位误差评估事件作用，不能只比较重构误差。',
              '当前数据是单个确定种子的合成实验，不能据此声称事件校正普遍改善所有指标。', '',
              '末段为最后 8 s；指标相对于无噪声真实角度计算，含逐关节 RMSE、',
              '圆周相位误差、基础频率与实际推进速率的独立偏差、连续 2 s 锁定时间、',
              '形状门控比例、状态转换、系数变化、参考速度差分的 P99 和最大偏差。',
              '差分另提供三个相邻样本均为 LOCKED 的统计；全程最大值仍保留以暴露重捕获跳变。',
              '`capture_phase_splices` 记录相邻输出相位变化超出速率积分 0.5° 的拼接。',
              '初始/丢失后粗捕获可改变相位，此时参考有效标志为假，不能直接用于控制。',
              '另统计各关节基频谐波相位相对左髋的误差，以检查共享节律下的关节间协调。',
              '锁定时间用真实相位<10°且基频误差<3%持续2s计算，区别于算法自身 LOCKED 状态。', '',
              '变形：幅值比例 [1.18,0.84,1.20,0.88,1.14,0.82]，不同关节偏置 ±0.008–0.035 rad，',
              '局部相移 ±0.05–0.10 rad，二/三次谐波变化。变频阶跃 +20%，斜坡在 12–24 s 完成。',
              '组合场景另含 30 ms 因果延迟、独立小噪声、24–24.5 s 观测/事件缺失及单个髋角异常值。', '',
              '局限：输入仍由低阶 Fourier 模型生成，属于模型内合成可行性验证。',
              '较小重构误差不直接证明生理相位正确、地形切换安全、足端净空或人机闭环稳定。',
              '同时按原始 flat_walk 上下限计算 shadow 输出裁剪比例及误差：',
              '`final_clamped_vs_raw_truth_rmse_rad` 对未裁剪目标评分，显示约束造成的不可达误差；',
              '`final_clamped_vs_clamped_truth_rmse_rad` 对同样裁剪后的目标评分。',
              '主表报告学习核心的原始重构误差，不能直接当作约束后的控制参考跟踪能力。', '',
              '每个场景目录包含 input.csv、四组 C++ 核心原始输出、六关节末段跟踪图和全程学习图。']
    (args.output/'README.md').write_text('\n'.join(lines)+'\n')
    print('Results:', args.output.resolve())


if __name__ == '__main__':
    main()
