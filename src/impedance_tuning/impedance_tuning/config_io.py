"""Small, comment-preserving helpers for impedance parameter persistence."""

import os
import re
import tempfile

import yaml


def _replace_array(text, key, values):
    pattern = re.compile(
        rf'^(\s*{re.escape(key)}\s*:\s*)\[[^\]\n]*\](\s*(?:#.*)?)$',
        re.MULTILINE)
    def format_double(value):
        formatted = f'{float(value):.6g}'
        if '.' not in formatted and 'e' not in formatted.lower():
            formatted += '.0'
        return formatted

    replacement = '[' + ', '.join(format_double(value) for value in values) + ']'
    updated, count = pattern.subn(
        lambda match: match.group(1) + replacement + match.group(2), text)
    if count != 1:
        raise ValueError(f'期望 YAML 中只有一个 {key}，实际找到 {count} 个')
    return updated


def _atomic_write(path, updated):
    directory = os.path.dirname(os.path.abspath(path))
    descriptor, temporary_path = tempfile.mkstemp(prefix='.impedance_params_', dir=directory)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(updated)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary_path, os.stat(path).st_mode)
        os.replace(temporary_path, path)
    except Exception:
        if os.path.exists(temporary_path):
            os.unlink(temporary_path)
        raise


def save_controller_parameters(
        path, joints, kp, kd, tube_half_width, tube_transition_width):
    """Replace only the tunable arrays in an existing controller YAML file."""
    arrays = (kp, kd, tube_half_width, tube_transition_width)
    if any(len(joints) != len(values) for values in arrays):
        raise ValueError('关节数与阻抗/虚拟管道参数数量不一致')
    if not joints:
        raise ValueError('关节列表为空')
    if any(value < 0.0 for values in arrays for value in values):
        raise ValueError('阻抗/虚拟管道参数不能为负数')

    with open(path, encoding='utf-8') as stream:
        original = stream.read()
    root = yaml.safe_load(original)
    params = root['exo_impedance_controller']['ros__parameters']
    if list(params['joints']) != list(joints):
        raise ValueError('YAML 中的关节顺序已变化，拒绝写入以免参数错位')

    updated = _replace_array(original, 'default_kp', kp)
    updated = _replace_array(updated, 'default_kd', kd)
    updated = _replace_array(updated, 'default_tube_half_width', tube_half_width)
    updated = _replace_array(
        updated, 'default_tube_transition_width', tube_transition_width)
    _atomic_write(path, updated)


def save_controller_gains(path, joints, kp, kd):
    """Backward-compatible gain-only persistence helper."""
    if len(joints) != len(kp) or len(joints) != len(kd):
        raise ValueError('关节数与 Kp/Kd 数量不一致')
    if not joints:
        raise ValueError('关节列表为空')
    if any(value < 0.0 for value in [*kp, *kd]):
        raise ValueError('Kp/Kd 不能为负数')
    with open(path, encoding='utf-8') as stream:
        original = stream.read()
    root = yaml.safe_load(original)
    params = root['exo_impedance_controller']['ros__parameters']
    if list(params['joints']) != list(joints):
        raise ValueError('YAML 中的关节顺序已变化，拒绝写入以免参数错位')
    updated = _replace_array(original, 'default_kp', kp)
    updated = _replace_array(updated, 'default_kd', kd)
    _atomic_write(path, updated)
