#!/usr/bin/env python3
"""Fair staged-vs-joint update-policy experiment with the exact shared C++ core.

Only measurement angles and occurrence-stamped events enter that core. Target
phase, frequency and coefficients exist only here for generation and scoring.
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

from evaluate_adaptive_learning import JOINTS, LABELS, template, truth as original_truth, wrap

CASES = ['unchanged', 'phase_shift', 'frequency_step', 'shape_deformation', 'frequency_ramp',
         'combined', 'runtime_phase_offset', 'dc_bias_step', 'amplitude_step', 'startup_shape_deformation']
MODES = ['frozen', 'sync', 'full', 'joint', 'no_events', 'joint_no_events']
COLORS = {'frozen': '#9b9b9b', 'sync': '#d08c2a', 'full': '#2077a5', 'joint': '#b84f57',
          'no_events': '#669f78', 'joint_no_events': '#8a6ea8'}
LABEL = {'frozen': 'Fixed template', 'sync': 'Rhythm only', 'full': 'Staged shape learning',
         'joint': 'Simultaneous learning', 'no_events': 'Staged, no events',
         'joint_no_events': 'Simultaneous, no events'}
CHANGE = {'unchanged': 0., 'phase_shift': 0., 'frequency_step': 12., 'shape_deformation': 8.,
          'frequency_ramp': 12., 'combined': 8., 'runtime_phase_offset': 12.,
          'dc_bias_step': 12., 'amplitude_step': 12., 'startup_shape_deformation': 0.}
CHANGE_END = dict(CHANGE, shape_deformation=12., frequency_ramp=24., combined=12., runtime_phase_offset=12.6)
OFFSET = np.array([.025, -.035, .008, -.020, .030, -.008])
AMP = np.array([1.18, .84, 1.20, .88, 1.14, .82])


def experiment_truth(times, case, omega, prior):
    t = np.asarray(times)
    if case not in ['runtime_phase_offset', 'dc_bias_step', 'amplitude_step', 'startup_shape_deformation']:
        return original_truth(t, case, omega, prior)
    phase = omega*t
    frequency = np.full(t.shape, omega)
    weights = np.broadcast_to(prior, (t.size, 6, 7)).copy()
    after = (t >= 12).astype(float)
    if case == 'startup_shape_deformation':
        target_weights=original_truth(np.array([12.]),'shape_deformation',omega,prior)[3][0]
        weights=np.broadcast_to(target_weights,(t.size,6,7)).copy()
    elif case == 'runtime_phase_offset':
        # A finite-duration phase advance has its own instantaneous frequency.
        u = np.clip((t-12)/.6, 0, 1)
        s = u**3*(10-15*u+6*u*u)
        ds = 30*u*u*(1-u)*(1-u)/.6
        phase += .7*s
        frequency += .7*ds
    elif case == 'dc_bias_step':
        weights[:, :, 0] += after[:, None]*OFFSET
    else:
        weights[:, :, 1:] *= 1+after[:, None, None]*(AMP[None, :, None]-1)
    q = weights[:, :, 0].copy()
    for n in range(1, 4):
        q += weights[:, :, 2*n-1]*np.cos(n*phase[:, None])+weights[:, :, 2*n]*np.sin(n*phase[:, None])
    return phase, frequency, q, weights


def input_csv(folder, case, duration, omega, prior, seed):
    dt = .002
    t = np.arange(round(duration/dt)+1)*dt
    phase, _, q, _ = experiment_truth(t, case, omega, prior)
    rng = np.random.default_rng(seed)
    if case == 'combined':
        q += rng.normal(size=q.shape)*np.array([.002, .003, .001, .002, .003, .001])
        q[np.abs(t-32)<.001, 0] += .7
    delay = .03 if case == 'combined' else 0
    mq, eq = deque(), deque()
    ids = [0, 0]
    filename = folder/'input.csv'
    with filename.open('w') as handle:
        handle.write('now,stamp,'+','.join('q'+str(j) for j in range(6))+
                     ',event_stamp,event_foot,event_id,event_phase\n')
        for k, now in enumerate(t):
            lost = case == 'combined' and 24 <= now < 24.5
            if k % 5 == 0 and not lost:
                mq.append((now+delay, now+1, q[k]))
            if k and not lost:
                before, after = int(np.floor(phase[k-1]/np.pi)), int(np.floor(phase[k]/np.pi))
                if after > before:
                    cross = t[k-1]+(after*np.pi-phase[k-1])/(phase[k]-phase[k-1])*dt
                    foot = after % 2
                    ids[foot] += 1
                    eq.append((cross+delay, cross+1, foot, ids[foot], foot*np.pi))
            stamp, measured = 0., np.zeros(6)
            if mq and mq[0][0] <= now+1e-10:
                _, stamp, measured = mq.popleft()
            event_stamp, foot, eid, event_phase = 0., -1, 0, 0.
            if eq and eq[0][0] <= now+1e-10:
                _, event_stamp, foot, eid, event_phase = eq.popleft()
            values = [now+1, stamp, *measured, event_stamp, foot, eid, event_phase]
            handle.write(','.join(format(float(v), '.12g') for v in values)+'\n')
    return filename


def rows_weights(trace):
    return np.stack([np.column_stack([trace['b'+str(j)]]+
                     [trace[p+str(j)+str(n)] for n in range(1, 4) for p in ['a', 'c']])
                     for j in range(6)], axis=1)


def arrays(trace, case, omega, prior):
    t = trace['now']-1
    phase, freq, target, target_w = experiment_truth(t, case, omega, prior)
    q = np.column_stack([trace['q'+str(j)] for j in range(6)])
    qd = np.column_stack([trace['qd'+str(j)] for j in range(6)])
    return t, phase, freq, target, target_w, q, qd


def metrics(trace, case, omega, prior, limits):
    t, phase, freq, target, target_w, q, qd = arrays(trace, case, omega, prior)
    err = q-target
    pe = wrap(trace['phase']-phase)
    change = CHANGE[case]
    result = {}
    masks = {'overall': np.ones(t.shape, bool), 'final': t >= t[-1]-8,
             'change_window_3s': (t >= change) & (t < change+3),
             'change_window_5s': (t >= change) & (t < change+5),
             'pre_change': (t >= max(0, change-4)) & (t < max(change, 4))}
    for name, mask in masks.items():
        if not np.any(mask):
            continue
        result[name+'_rmse_rad'] = float(np.sqrt(np.mean(err[mask]**2)))
        result[name+'_joint_rmse_rad'] = dict(zip(JOINTS, np.sqrt(np.mean(err[mask]**2, axis=0)).tolist()))
        result[name+'_phase_rmse_deg'] = float(np.rad2deg(np.sqrt(np.mean(pe[mask]**2))))
        result[name+'_base_frequency_bias_pct'] = float(np.mean((trace['omega_base'][mask]-freq[mask])/freq[mask])*100)
        result[name+'_rate_frequency_bias_pct'] = float(np.mean((trace['phase_rate'][mask]-freq[mask])/freq[mask])*100)
        result[name+'_rate_frequency_rmse_pct'] = float(np.sqrt(np.mean(((trace['phase_rate'][mask]-freq[mask])/freq[mask])**2))*100)
    change_mask = masks['change_window_5s']
    result['peak_change_phase_deg'] = float(np.rad2deg(np.max(np.abs(pe[change_mask]))))
    result['peak_change_six_joint_rmse_rad'] = float(np.max(np.sqrt(np.mean(err[change_mask]**2, axis=1))))
    clipped = np.clip(q, limits[:, 0], limits[:, 1])
    final = masks['final']
    result['reference_clipped_fraction'] = float(np.mean(q != clipped))
    result['final_clamped_vs_raw_truth_rmse_rad'] = float(np.sqrt(np.mean((clipped[final]-target[final])**2)))
    if case == 'dc_bias_step':
        latent = original_truth(t, 'unchanged', omega, prior)[2]
        result['final_raw_vs_latent_rmse_rad'] = float(np.sqrt(np.mean((q[final]-latent[final])**2)))
    result['final_clamped_vs_clamped_truth_rmse_rad'] = float(np.sqrt(np.mean((clipped[final]-np.clip(target[final],limits[:, 0],limits[:, 1]))**2)))
    states = trace['state'].astype(int)
    result['locked_fraction'] = float(np.mean(states == 2))
    result['shape_active_fraction'] = float(np.mean(trace['shape_active']))
    active=np.flatnonzero(trace['shape_active']>.5)
    result['first_shape_active_s']=float(t[active[0]]) if active.size else None
    result['state_transitions'] = [[float(t[k]), int(states[k])] for k in range(t.size) if k == 0 or states[k] != states[k-1]]
    numerical = (q[2:]-q[:-2])/(t[2:]-t[:-2])[:, None]
    fd = np.abs(numerical-qd[1:-1])
    result['velocity_fd_max_rad_s'] = float(fd.max())
    result['velocity_fd_p99_rad_s'] = float(np.percentile(fd, 99))
    locked = (states[:-2] == 2) & (states[1:-1] == 2) & (states[2:] == 2)
    result['locked_velocity_fd_max_rad_s'] = float(fd[locked].max()) if np.any(locked) else None
    result['locked_velocity_fd_p99_rad_s'] = float(np.percentile(fd[locked],99)) if np.any(locked) else None
    expected = np.diff(t)*(trace['phase_rate'][1:]+trace['phase_rate'][:-1])/2
    splice = np.diff(trace['phase'])-expected
    result['capture_phase_splices'] = [[float(t[k+1]),float(np.rad2deg(splice[k]))]
                                      for k in np.flatnonzero(np.abs(splice)>np.deg2rad(.5))]
    learned = rows_weights(trace)
    rel = wrap((np.arctan2(learned[-1,:,1],learned[-1,:,2])-np.arctan2(learned[-1,0,1],learned[-1,0,2]))-
               (np.arctan2(target_w[-1,:,1],target_w[-1,:,2])-np.arctan2(target_w[-1,0,1],target_w[-1,0,2])))
    result['final_relative_fundamental_phase_rmse_deg'] = float(np.rad2deg(np.sqrt(np.mean(rel[1:]**2))))
    result['final_relative_fundamental_phase_error_deg'] = dict(zip(JOINTS,np.rad2deg(rel).tolist()))
    result['max_coefficient_change_rad'] = float(np.max(np.abs(learned-prior)))
    # True external rhythm recovery: not the internal LOCKED flag.
    valid = (np.abs(pe) < np.deg2rad(10)) & (np.abs((trace['omega_base']-freq)/freq) < .03)
    result['rhythm_recovery_after_change_s'] = continuous_recovery(t, valid, CHANGE_END[case])
    # Fixed one-second RMS reconstruction threshold: 0.01 rad for every method/case.
    rms = np.sqrt(np.mean(err**2,axis=1))
    width = round(1.0/np.median(np.diff(t)))
    smooth = np.sqrt(np.convolve(rms*rms,np.ones(width)/width,'valid'))
    good = np.zeros(t.shape,bool); good[width-1:] = smooth < .01
    result['angle_recovery_after_change_s'] = continuous_recovery(t,good,CHANGE_END[case])
    result['joint_recovery_after_change_s'] = continuous_recovery(t,good & valid,CHANGE_END[case])
    if case == 'combined':
        result['dropout_angle_recovery_s'] = continuous_recovery(t,good,24.5)
        result['dropout_rhythm_recovery_s'] = continuous_recovery(t,valid,24.5)
        result['post_outlier_5s_rmse_rad'] = float(np.sqrt(np.mean(err[(t>=32)&(t<37)]**2)))
    return result


def continuous_recovery(t, valid, change):
    width = round(2/np.median(np.diff(t)))
    if len(t) < width:
        return None
    rolling = np.convolve(valid.astype(int),np.ones(width,dtype=int),'valid')
    starts = np.flatnonzero((rolling == width) & (t[:len(rolling)] >= change))
    return float(t[starts[0]]-change) if starts.size else None


def save(fig,path):
    fig.tight_layout(rect=(0,0,1,.88) if getattr(fig,'_report_legend',False) else (0,0,1,1))
    fig.savefig(path.with_suffix('.pdf'), bbox_inches='tight')
    fig.savefig(path.with_suffix('.png'), dpi=150,bbox_inches='tight')
    plt.close(fig)


def top_legend(fig, ax, columns=2):
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, ncol=columns, loc='upper center', fontsize=10,
               frameon=False, bbox_to_anchor=(.52,1.0))
    fig._report_legend=True


def style():
    plt.rcParams.update({'font.size':12,'axes.titlesize':13,'axes.labelsize':12,'legend.fontsize':10,
                         'xtick.labelsize':11,'ytick.labelsize':11,
                         'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'white'})


def rolling_rms(q,target,width=100):
    return np.sqrt(np.convolve(np.mean((q-target)**2,axis=1),np.ones(width)/width,'valid'))


def figures(out, traces, omega, prior, limits, all_metrics):
    figdir=out/'report_figures';figdir.mkdir(exist_ok=True)
    style()
    # 1. Final and transient, no logarithmic zero problem.
    names=['frequency_step','shape_deformation','combined','runtime_phase_offset','dc_bias_step','amplitude_step']
    short=['Frequency\nstep','Waveform\nchange','Combined','Phase\nadvance','DC offset\nstep','Amplitude\nstep']
    fig,ax=plt.subplots(2,1,figsize=(8.6,5.1),sharex=True)
    x=np.arange(len(names));width=.19
    for k,mode in enumerate(['sync','full','joint']):
        ax[0].bar(x+(k-1)*width,[all_metrics[c][mode]['final_rmse_rad'] for c in names],width,
                  color=COLORS[mode],label=LABEL[mode])
        ax[1].bar(x+(k-1)*width,[all_metrics[c][mode]['change_window_5s_rmse_rad'] for c in names],width,
                  color=COLORS[mode],label=LABEL[mode])
    ax[0].set_ylabel('Final 8 s RMSE [rad]');top_legend(fig,ax[0],3)
    ax[1].set_ylabel('First 5 s after change\nRMSE [rad]');ax[1].set_xticks(x,short)
    for a in ax:a.grid(axis='y',alpha=.2)
    save(fig,figdir/'summary')
    # 2. All six joints under shape deformation, left/right columns.
    case='shape_deformation'; tr=traces[case]['full'];t,_,_,target,_,_,_=arrays(tr,case,omega,prior)
    fig,axes=plt.subplots(3,2,figsize=(8.6,5.7),sharex=True)
    mask=t>=45
    for j,a in zip([0,3,1,4,2,5],axes.flat):
        a.plot(t[mask],target[mask,j],color='black',lw=1.6,label='Measurement target')
        for mode in ['sync','full','joint']:
            a.plot(t[mask],traces[case][mode]['q'+str(j)][mask],color=COLORS[mode],lw=1.15,label=LABEL[mode])
        a.set_title(LABELS[j]);a.set_ylabel('Angle [rad]');a.grid(alpha=.2)
    top_legend(fig,axes[0,0],2);axes[-1,0].set_xlabel('Time [s]');axes[-1,1].set_xlabel('Time [s]')
    save(fig,figdir/'shape_six_joint')
    # 3. Two rhythm changes with actual phase/frequency truth.
    fig,axes=plt.subplots(3,2,figsize=(8.6,5.7),sharex='col')
    for col,case in enumerate(['frequency_step','runtime_phase_offset']):
        tr=traces[case]['full'];t,phase,freq,target,_,_,_=arrays(tr,case,omega,prior)
        mask=(t>=10)&(t<=23)
        axes[0,col].plot(t[mask],freq[mask]/(2*np.pi),'k',lw=1.6,label='True instantaneous frequency')
        for mode in ['sync','full','joint']:
            tr=traces[case][mode]
            axes[0,col].plot(t[mask],tr['omega_base'][mask]/(2*np.pi),color=COLORS[mode],label=LABEL[mode])
            axes[1,col].plot(t[mask],np.rad2deg(wrap(tr['phase'][mask]-phase[mask])),color=COLORS[mode])
            q=np.column_stack([tr['q'+str(j)] for j in range(6)])
            rms=rolling_rms(q,target)
            mt=(t[99:]>=10)&(t[99:]<=23)
            axes[2,col].plot(t[99:][mt],rms[mt],color=COLORS[mode])
        axes[0,col].set_title('Frequency +20%' if col==0 else 'Phase +0.7 rad over 0.6 s')
        for row in range(3):axes[row,col].axvline(12,color='#777',ls='--',lw=.7);axes[row,col].grid(alpha=.2)
        axes[2,col].set_xlabel('Time [s]')
    for row,label in enumerate(['Base frequency\n[Hz]','Phase error\n[deg]','1 s RMS error\n[rad]']):axes[row,0].set_ylabel(label)
    top_legend(fig,axes[0,0],2);save(fig,figdir/'rhythm_transients')
    # 4. DC and amplitude: targets at source time, angles and coefficients.
    fig,axes=plt.subplots(3,2,figsize=(8.6,5.9))
    for col,case in enumerate(['dc_bias_step','amplitude_step']):
        tr=traces[case]['full'];t,_,_,target,target_w,_,_=arrays(tr,case,omega,prior)
        zoom=(t>=10)&(t<=17)
        axes[0,col].plot(t[zoom],target[zoom,0],'k',lw=1.4,label='Left hip target')
        for mode in ['sync','full','joint']:
            tr=traces[case][mode]
            axes[0,col].plot(t[zoom],tr['q0'][zoom],color=COLORS[mode],label=LABEL[mode])
            q=np.column_stack([tr['q'+str(j)] for j in range(6)])
            axes[1,col].plot(t[99:],rolling_rms(q,target),color=COLORS[mode])
        if col==0:
            axes[2,col].plot(t,target_w[:,0,0],'k',lw=1.5,label='True left hip DC')
            for mode in ['full','joint']:axes[2,col].plot(t,traces[case][mode]['b0'],color=COLORS[mode],label=LABEL[mode])
        else:
            atrue=np.sqrt(target_w[:,0,1]**2+target_w[:,0,2]**2)
            axes[2,col].plot(t,atrue,'k',lw=1.5,label='True left hip fundamental amplitude')
            for mode in ['full','joint']:
                tr=traces[case][mode];axes[2,col].plot(t,np.hypot(tr['a01'],tr['c01']),color=COLORS[mode],label=LABEL[mode])
        for row in range(3):axes[row,col].axvline(12,color='#777',ls='--',lw=.7);axes[row,col].grid(alpha=.2)
        axes[0,col].set_xlim(10,17)
        axes[0,col].set_xlabel('Time [s], transient detail')
        axes[0,col].set_title('Measurement DC offset step' if col==0 else 'Six-joint amplitude step')
        axes[2,col].set_xlabel('Time [s]')
    axes[0,0].set_ylabel('Hip angle [rad]');axes[1,0].set_ylabel('1 s RMS error\n[rad]');axes[2,0].set_ylabel('Coefficient\n[rad]')
    top_legend(fig,axes[0,0],2);save(fig,figdir/'offset_amplitude')
    # 5. Combined full duration, dropout, rates, active learning masks.
    case='combined';tr=traces[case]['full'];t,phase,freq,target,_,_,_=arrays(tr,case,omega,prior)
    fig,axes=plt.subplots(4,1,figsize=(8.6,5.7),sharex=True)
    axes[0].plot(t,freq/(2*np.pi),'k',lw=1.5,label='True stride frequency')
    for mode in ['sync','full','joint']:
        tr=traces[case][mode]
        axes[0].plot(t,tr['omega_base']/(2*np.pi),color=COLORS[mode],label=LABEL[mode])
        axes[1].plot(t,np.rad2deg(wrap(tr['phase']-phase)),color=COLORS[mode])
        q=np.column_stack([tr['q'+str(j)] for j in range(6)])
        axes[2].plot(t[99:],rolling_rms(q,target),color=COLORS[mode])
    for mode in ['full','joint']:axes[3].plot(t,traces[case][mode]['shape_active'],color=COLORS[mode],label=LABEL[mode],alpha=.8)
    for a in axes:
        a.axvspan(24,24.5,color='#777',alpha=.15);a.grid(alpha=.2)
    for a,label in zip(axes,['Frequency [Hz]','Phase error\n[deg]','1 s RMS error\n[rad]','Shape gate']):a.set_ylabel(label)
    top_legend(fig,axes[0],2);axes[-1].set_xlabel('Time [s]');save(fig,figdir/'combined')
    # A compact startup panel isolates the initial capture gate from later changes.
    if 'startup_shape_deformation' in traces:
        case='startup_shape_deformation';tr=traces[case]['full']
        t,_,_,target,_,_,_=arrays(tr,case,omega,prior)
        fig,axes=plt.subplots(1,2,figsize=(8.6,2.7))
        for mode in ['full','joint']:
            tr=traces[case][mode]
            q=np.column_stack([tr['q'+str(j)] for j in range(6)])
            mask=(t[99:]<=8)
            axes[0].plot(t[99:][mask],rolling_rms(q,target)[mask],color=COLORS[mode],label=LABEL[mode])
            mask=t<=8
            axes[1].plot(t[mask],tr['shape_active'][mask],color=COLORS[mode],label=LABEL[mode],alpha=.85)
        axes[0].set_ylabel('1 s RMS error [rad]');axes[1].set_ylabel('Shape update active')
        axes[1].set_yticks([0,1]);axes[1].set_ylim(-.05,1.15)
        for a in axes:a.set_xlabel('Time [s]');a.grid(alpha=.2)
        top_legend(fig,axes[0],2);save(fig,figdir/'startup_transient')
    # 6. Relative phase and no-event ablation; shared phase is not same joint waveform.
    names=['shape_deformation','combined','frequency_ramp']
    fig,axes=plt.subplots(1,2,figsize=(8.6,3.7))
    x=np.arange(len(names));width=.17
    for k,mode in enumerate(['full','joint','no_events','joint_no_events']):
        axes[0].bar(x+(k-1.5)*width,[all_metrics[c][mode]['final_phase_rmse_deg'] for c in names],width,color=COLORS[mode],label=LABEL[mode])
        axes[1].bar(x+(k-1.5)*width,[all_metrics[c][mode]['final_relative_fundamental_phase_rmse_deg'] for c in names],width,color=COLORS[mode])
    for a in axes:a.set_xticks(x,['Waveform','Combined','Freq. ramp']);a.grid(axis='y',alpha=.2)
    axes[0].set_ylabel('True phase RMSE [deg]');axes[1].set_ylabel('Relative joint phase\nRMSE [deg]');top_legend(fig,axes[0],2)
    save(fig,figdir/'coordination')


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--template',type=Path,default=root/'config/cpg.yaml')
    parser.add_argument('--output',type=Path,default=Path(tempfile.gettempdir())/'cpg_experiments/learning_strategy_comparison')
    parser.add_argument('--duration',type=float,default=50.)
    parser.add_argument('--cases',nargs='+',choices=CASES,default=CASES)
    parser.add_argument('--probe',type=Path)
    parser.add_argument('--skip-figures',action='store_true')
    args=parser.parse_args()
    if args.duration<40:parser.error('duration>=40 required for post-change settling')
    args.output.mkdir(parents=True,exist_ok=True)
    probe=args.probe or args.output/'adaptive_learning_probe'
    if args.probe is None:
        subprocess.run(['g++','-O2','-std=c++17','-I'+str(root/'include'),str(root/'scripts/adaptive_learning_probe.cpp'),'-o',str(probe)],check=True)
    omega,prior=template(args.template)
    joints=yaml.safe_load(args.template.read_text())['adaptive_cpg_node']['ros__parameters']['gait_library']['flat_walk']['joints']
    limits=np.array([[joints[j]['limit_min'],joints[j]['limit_max']] for j in JOINTS])
    prior_file=args.output/'original_flat_walk.txt'
    np.savetxt(prior_file,prior,header=format(omega,'.12g'),comments='',fmt='%.12g')
    output={'schema_version':1,'duration_s':args.duration,'measurement_dt_s':.01,'integration_dt_s':.002,'trace_dt_s':.01,
            'source_template':str(args.template.resolve()),'source_template_sha256':hashlib.sha256(args.template.read_bytes()).hexdigest(),
            'core_sha256':hashlib.sha256((root/'include/cpg_controller/adaptive_cpg.hpp').read_bytes()).hexdigest(),
            'probe_sha256':hashlib.sha256(probe.read_bytes()).hexdigest(),
            'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'gains':{'phase':6.,'frequency':3.,'shape':4.},'modes':MODES,
            'comparison':'Only shape_requires_lock differs: full requires LOCKED and abs(filtered phase error)<lock_error; joint omits both, while freshness/motion/observability remain common. Priors, gains, samples, events, bounds, filters and coarse capture are identical.',
            'change_start_s':CHANGE,'change_end_s':CHANGE_END,'metric_definitions':{
                'transient':'RMSE over [change_start, change_start+5) or +3 s, not only post-end samples',
                'final':'last8s of each50s record',
                'rhythm_recovery':'from change_end to first 2s interval with true circular phase<10deg and abs base frequency error<3%',
                'angle_recovery':'from change_end to first2s interval with causal1s sixjoint RMS angle error<.01rad',
                'joint_recovery':'angle and rhythm conditions both continuously met for2s; null if no qualifying window',
                'initial_phase_case':'constant initial0.7rad phase offset uses shared coarse capture; not evidence for either shape gate',
                'frequency':'actual phase_rate separately from basefrequency; phasewarp truthfreq includes phase derivative'},'cases':{}}
    traces={}
    for case in args.cases:
        folder=args.output/case;folder.mkdir(exist_ok=True)
        inp=input_csv(folder,case,args.duration,omega,prior,20261023+CASES.index(case))
        traces[case]={};output['cases'][case]={}
        for mode in MODES:
            csv=folder/(mode+'.csv')
            subprocess.run([str(probe),str(prior_file),str(inp),str(csv),mode],check=True)
            tr=np.genfromtxt(csv,delimiter=',',names=True)
            traces[case][mode]=tr
            output['cases'][case][mode]=metrics(tr,case,omega,prior,limits)
        print(case,', '.join(m+'='+format(output['cases'][case][m]['final_rmse_rad'],'.6f') for m in MODES),flush=True)
    # Three deterministic paired seeds are a robustness check, not a human
    # sample size or a statistical significance test. Keep the plotted main seed.
    if 'combined' in args.cases:
        base_seed = 20261023+CASES.index('combined')
        repeats = [{'seed':base_seed,'metrics':output['cases']['combined']}]
        for seed in [base_seed+1,base_seed+2]:
            folder=args.output/'seed_repeats'/str(seed)
            folder.mkdir(parents=True,exist_ok=True)
            inp=input_csv(folder,'combined',args.duration,omega,prior,seed)
            result={}
            for mode in ['full','joint']:
                csv=folder/(mode+'.csv')
                subprocess.run([str(probe),str(prior_file),str(inp),str(csv),mode],check=True)
                result[mode]=metrics(np.genfromtxt(csv,delimiter=',',names=True),'combined',omega,prior,limits)
            repeats.append({'seed':seed,'metrics':result})
        output['combined_seed_repeats']=repeats
        output['combined_seed_summary']={mode:{key:{
            'values':[r['metrics'][mode][key] for r in repeats],
            'mean':float(np.mean([r['metrics'][mode][key] for r in repeats])),
            'sample_std':float(np.std([r['metrics'][mode][key] for r in repeats],ddof=1))}
            for key in ['final_rmse_rad','change_window_5s_rmse_rad','final_phase_rmse_deg']}
            for mode in ['full','joint']}
    if not args.skip_figures and set(CASES).issubset(args.cases):figures(args.output,traces,omega,prior,limits,output['cases'])
    (args.output/'metrics.json').write_text(json.dumps(output,indent=2,ensure_ascii=False)+'\n')
    lines=['# 分阶段与共同学习：六关节对照结果','','共同学习只改变波形通道门控，使用同一项目模型；不宣称逐式复现 TRO。',
           '所有方法使用原始 flat_walk 先验，同一组六关节输入与合成事件；无连续真值相位/频率输入，离散事件由已知合成相位的0/π交叉生成。',
           '相位、频率在两种方法中始终同时修正，比较锁定/小相位误差门控与有效观测下的共同更新。','',
           '|场景|仅节律末段RMSE|分阶段末段RMSE|共同学习末段RMSE|分阶段变化后5sRMSE|共同学习变化后5sRMSE|',
           '|---|---:|---:|---:|---:|---:|']
    for case,result in output['cases'].items():
        lines.append('|'+case+'|'+'|'.join(f"{result[m][key]:.6f}" for m,key in [
            ('sync','final_rmse_rad'),('full','final_rmse_rad'),('joint','final_rmse_rad'),
            ('full','change_window_5s_rmse_rad'),('joint','change_window_5s_rmse_rad')])+'|')
    lines += ['', '单位为 rad，末段是最后8s；变化后窗口从表中各场景变化时刻开始。',
              '相位变化在12–12.6s平滑增加0.7rad，瞬时真实频率包含该变化的导数。',
              'DC偏置阶跃是观测角度偏移；算法能跟随偏移不等于能够识别传感器零偏与真实人体姿态。',
              '幅值阶跃在12s直接改变各通道谐波幅值。波形变形在8–12s平滑改变幅值、偏置、相对相位、高阶成分；startup_shape_deformation从t0即采用该变形波形，真实相位仍为omega*t。',
              '形状/节律恢复分别使用1s滚动六关节RMSE<0.01rad、相位<10度且基频误差<3%，保持2s，从变化结束计到合格窗口起点；同时满足两者另存joint_recovery。',
              '全程速度差分保留捕获拼接；LOCKED内另统计。原模板限幅另评分，不能用原始拟合误差代替实际参考误差。',
              '共享相位并不让六关节同相，另以关节基频相对相位误差检查协调。',
              '主表每组一条共享固定种子轨迹；组合场景另做3个配对噪声种子的稳健性检查，不是人体统计试验或显著性检验。模型内合成结果不证明地形转换或闭环稳定性。']
    (args.output/'findings.md').write_text('\n'.join(lines)+'\n')
    print('Artifacts:',args.output.resolve())


if __name__=='__main__':main()
