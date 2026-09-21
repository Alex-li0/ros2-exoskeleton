"""Pure numerical helpers for one-CPG-cycle impedance metrics."""

import numpy as np


def _finite(values):
    return np.asarray(values, dtype=float)


def estimate_phase_lag_ms(t, reference, measured):
    """Return positive lag when the measured trajectory trails the reference."""
    t = _finite(t)
    reference = _finite(reference)
    measured = _finite(measured)
    valid = np.isfinite(t) & np.isfinite(reference) & np.isfinite(measured)
    t, reference, measured = t[valid], reference[valid], measured[valid]
    if len(t) < 20 or np.ptp(reference) < 1e-6:
        return float('nan')
    dt = float(np.median(np.diff(t)))
    max_lag = max(1, min(len(t) // 4, int(round(0.25 * (t[-1] - t[0]) / dt))))
    best_lag = 0
    best_corr = -np.inf
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            ref_seg = reference[:len(reference) - lag or None]
            meas_seg = measured[lag:]
        else:
            ref_seg = reference[-lag:]
            meas_seg = measured[:len(measured) + lag]
        if len(ref_seg) < 10:
            continue
        ref_seg = ref_seg - np.mean(ref_seg)
        meas_seg = meas_seg - np.mean(meas_seg)
        denom = np.linalg.norm(ref_seg) * np.linalg.norm(meas_seg)
        corr = float(np.dot(ref_seg, meas_seg) / denom) if denom > 1e-12 else -np.inf
        if corr > best_corr:
            best_corr = corr
            best_lag = lag
    return 1000.0 * best_lag * dt


def analyze_cycle(samples, torque_limit):
    """Compute metrics for a list of sample dictionaries from one CPG cycle."""
    if len(samples) < 20:
        return None
    t = _finite([s['t'] for s in samples])
    q_ref = _finite([s['q_ref'] for s in samples])
    q_meas = _finite([s['q_meas'] for s in samples])
    iq_cmd = _finite([s['iq_cmd'] for s in samples])
    iq_meas = _finite([s.get('iq_meas', np.nan) for s in samples])
    loop_dt = _finite([s.get('loop_dt', np.nan) for s in samples])
    ref_age = _finite([s.get('ref_age', np.nan) for s in samples])
    valid = np.isfinite(q_ref) & np.isfinite(q_meas)
    if np.count_nonzero(valid) < 20:
        return None
    error = q_ref[valid] - q_meas[valid]
    motion_range = float(np.ptp(q_ref[valid]))
    rms = float(np.sqrt(np.mean(error ** 2)))
    finite_iq = iq_cmd[np.isfinite(iq_cmd)]
    finite_iq_meas = iq_meas[np.isfinite(iq_meas)]
    finite_loop = loop_dt[np.isfinite(loop_dt)]
    finite_age = ref_age[np.isfinite(ref_age)]
    return {
        'samples': int(np.count_nonzero(valid)),
        'period_s': float(t[-1] - t[0]),
        'rms_rad': rms,
        'peak_rad': float(np.max(np.abs(error))),
        'nrmse_pct': 100.0 * rms / motion_range if motion_range > 1e-6 else float('nan'),
        'phase_lag_ms': estimate_phase_lag_ms(t[valid], q_ref[valid], q_meas[valid]),
        'iq_cmd_rms': float(np.sqrt(np.mean(finite_iq ** 2))) if len(finite_iq) else float('nan'),
        'iq_cmd_peak': float(np.max(np.abs(finite_iq))) if len(finite_iq) else float('nan'),
        'iq_meas_rms': float(np.sqrt(np.mean(finite_iq_meas ** 2))) if len(finite_iq_meas) else float('nan'),
        'saturation_pct': (
            100.0 * float(np.mean(np.abs(finite_iq) >= 0.98 * torque_limit))
            if len(finite_iq) and torque_limit > 0.0 else float('nan')),
        'loop_dt_p99_ms': (
            1000.0 * float(np.percentile(finite_loop, 99)) if len(finite_loop) else float('nan')),
        'ref_age_p99_ms': (
            1000.0 * float(np.percentile(finite_age, 99)) if len(finite_age) else float('nan')),
    }
