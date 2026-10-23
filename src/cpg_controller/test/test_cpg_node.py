"""Isolated process tests: every control topic is remapped; no hardware launch."""
import math
import os
import subprocess
import time
import uuid

import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from terrain_adaptive_cpg.msg import CPGDebug, CPGParameterTarget
import yaml

NAMES = ['left_hip3', 'left_knee', 'left_ankle',
         'right_hip3', 'right_knee', 'right_ankle']
TOPICS = ['/position_controller/commands', '/joint_states', '/cpg/joint_ref',
          '/cpg/state_cmd', '/cpg/state', '/cpg/switch_debug', '/cpg/debug',
          '/cpg/gait_mode_target', '/cpg/gait_mode', '/cpg/parameter_target',
          '/cpg/measured_joint_states']


@pytest.fixture(autouse=True)
def isolated_environment(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '179')


def parameters():
    modes = {}
    for name, omega, offset in [('flat', 3.0, 0.0), ('slope', 4.0, .15)]:
        modes[name] = {'omega': omega, 'joints': {
            joint: {'num_harmonics': 4, 'w_offset': offset,
                    'w_x': [0.0, .2, .03, .02], 'w_y': [0.0, .1, .02, -.01],
                    'limit_min': -.8, 'limit_max': .8} for joint in NAMES}}
    return {'gait_modes': ['flat', 'slope'], 'default_gait': 'flat',
            'ramp_time': .3, 'publish_ankle': False,
            'parameter_target_timeout_s': .25, 'modulation_transition_s': .2,
            'gait_library': modes}


@pytest.fixture
def running(tmp_path):
    prefix = '/cpg_test_' + uuid.uuid4().hex
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='179')
    config = tmp_path / 'cpg.yaml'
    config.write_text(yaml.safe_dump({'adaptive_cpg_node': {'ros__parameters': parameters()}}))
    args = [os.environ['CPG_BINARY'], '--ros-args', '--params-file', str(config)]
    for topic in TOPICS:
        args += ['-r', topic + ':=' + prefix + topic]
    log = (tmp_path / 'node.log').open('w')
    proc = subprocess.Popen(args, env=env, stdout=log, stderr=log)
    rclpy.init()
    node = rclpy.create_node('test_' + uuid.uuid4().hex)
    refs, debug, states = [], [], []
    node.create_subscription(JointState, prefix + '/cpg/joint_ref', refs.append, 100)
    node.create_subscription(CPGDebug, prefix + '/cpg/debug', debug.append, 100)
    node.create_subscription(String, prefix + '/cpg/state', lambda m: states.append(m.data), 10)
    cmd = node.create_publisher(String, prefix + '/cpg/state_cmd', 10)
    mode = node.create_publisher(String, prefix + '/cpg/gait_mode_target', 10)
    target = node.create_publisher(CPGParameterTarget, prefix + '/cpg/parameter_target', 10)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.005)
            assert proc.poll() is None, (tmp_path / 'node.log').read_text()

    try:
        spin(.8)
        assert refs and cmd.get_subscription_count() == 1
        yield node, refs, debug, states, cmd, mode, target, spin
    finally:
        node.destroy_node()
        rclpy.shutdown()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        log.close()


def test_start_switch_stop_and_modulation(running):
    node, refs, debug, states, cmd, mode, target, spin = running
    assert all(v == 0 for v in refs[-1].position + refs[-1].velocity)
    mode.publish(String(data='slope'))
    spin(.1)
    assert debug[-1].gait_mode == 'slope'
    assert debug[-1].omega == 4.0  # IDLE switch updates oscillator frequency
    cmd.publish(String(data='start'))
    spin(.45)
    assert debug[-1].state == 'WALKING'
    mode.publish(String(data='flat'))
    spin(.15)
    assert debug[-1].state == 'SWITCHING'
    mode.publish(String(data='slope'))  # must not replace the in-flight snapshot
    spin(.45)
    assert debug[-1].state == 'WALKING'
    assert debug[-1].gait_mode == 'flat'
    msg = CPGParameterTarget()
    msg.stamp = node.get_clock().now().to_msg()
    msg.continuous_modulation_enabled = True
    msg.omega_scale_valid = True
    msg.omega_scale = 1.1
    msg.joint_names = NAMES
    msg.amplitude_scale = [1.1] * 6
    msg.amplitude_scale_valid = [True] * 6
    msg.offset_delta = [.02] * 6
    msg.offset_delta_valid = [True] * 6
    valid_stamp = msg.stamp
    msg.stamp = type(valid_stamp)(sec=-1)
    target.publish(msg)
    spin(.03)
    assert not debug[-1].continuous_modulation_active
    msg.stamp = node.get_clock().now().to_msg()
    target.publish(msg)
    spin(.15)
    assert debug[-1].continuous_modulation_active
    target.publish(msg)  # replay must not refresh timeout
    spin(.4)
    assert not debug[-1].continuous_modulation_active
    assert abs(debug[-1].omega - 3.0) < 1e-6
    mode.publish(String(data='slope'))
    spin(.1)
    cmd.publish(String(data='stop'))
    spin(.1)
    cmd.publish(String(data='start'))  # no reset during stopping
    spin(.55)
    assert debug[-1].state == 'IDLE'
    assert debug[-1].gait_mode == 'slope'
    assert all(v == 0 for v in refs[-1].position + refs[-1].velocity)
    # Actual emitted velocity must follow the derivative of emitted position.
    errors = []
    for a, b, c in zip(refs, refs[1:], refs[2:]):
        dt = (c.header.stamp.sec - a.header.stamp.sec +
              1e-9 * (c.header.stamp.nanosec - a.header.stamp.nanosec))
        if .002 < dt < .012:
            errors.append(abs((c.position[0] - a.position[0]) / dt - b.velocity[0]))
    assert len(errors) > 100
    assert sorted(errors)[int(.99 * len(errors))] < .06
    for ref in refs:
        assert all(math.isfinite(v) for v in ref.position + ref.velocity)
        assert ref.position[2] == ref.velocity[2] == 0
        assert ref.position[5] == ref.velocity[5] == 0
    assert 'STOPPING' in states and 'SWITCHING' in states


@pytest.mark.parametrize('bad', ['harmonics', 'learning', 'self_input'])
def test_invalid_configuration(tmp_path, bad):
    params = parameters()
    if bad == 'harmonics':
        params['gait_library']['flat']['joints']['left_hip3']['num_harmonics'] = 5
    elif bad == 'learning':
        params['eta_w'] = .1
    else:
        params['observe_measurements'] = True
    config = tmp_path / 'bad.yaml'
    config.write_text(yaml.safe_dump({'adaptive_cpg_node': {'ros__parameters': params}}))
    args = [os.environ['CPG_BINARY'], '--ros-args', '--params-file', str(config)]
    prefix = '/invalid_' + uuid.uuid4().hex
    for topic in TOPICS:
        destination = '/joint_states' if bad == 'self_input' and topic == '/cpg/measured_joint_states' else topic
        args += ['-r', topic + ':=' + prefix + destination]
    result = subprocess.run(args, capture_output=True, timeout=5,
                            env=dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='179'))
    assert result.returncode != 0
    expected = {'harmonics': b'num_harmonics must', 'learning': b'Online learning is not implemented',
                'self_input': b'Measurement input must not alias'}
    assert expected[bad] in result.stderr


def test_stop_during_start_is_continuous(running):
    _, refs, debug, _, cmd, _, _, spin = running
    cmd.publish(String(data='start'))
    spin(.12)
    cmd.publish(String(data='stop'))
    spin(.45)
    assert debug[-1].state == 'IDLE'
    assert all(v == 0 for v in refs[-1].position + refs[-1].velocity)
    # A rapid stop may change acceleration; it must not reset position instantly.
    for a, b in zip(refs, refs[1:]):
        dt = (b.header.stamp.sec - a.header.stamp.sec +
              1e-9 * (b.header.stamp.nanosec - a.header.stamp.nanosec))
        if 0 < dt < .02:
            assert abs(b.position[0] - a.position[0]) < 2 * dt + .001
