"""Shadow node I/O tests in an isolated ROS domain; never launch hardware."""
import math
import os
from pathlib import Path
import subprocess
import time
import uuid

import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header
import yaml

NAMES = ['left_hip3', 'left_knee', 'left_ankle',
         'right_hip3', 'right_knee', 'right_ankle']


def flat_prior():
    filename = os.environ.get('CPG_PRIOR_FILE', str(Path(__file__).parents[1] / 'config' / 'cpg.yaml'))
    with open(filename, encoding='utf-8') as source:
        return yaml.safe_load(source)['adaptive_cpg_node']['ros__parameters']['gait_library']['flat_walk']


def parameters(enabled):
    gait = flat_prior()
    params = {'omega_initial': gait['omega'], 'shape_enabled': True, 'events_enabled': False}
    if enabled:
        params['enable_learning'] = True
    for name, joint in gait['joints'].items():
        for key in ['w_offset', 'w_x', 'w_y', 'limit_min', 'limit_max']:
            params['joint.' + name + '.' + key] = joint[key]
    return params


@pytest.fixture
def start(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '181')
    processes = []
    log_files = []
    rclpy.init()
    node = rclpy.create_node('shadow_test_' + uuid.uuid4().hex)

    def create(enabled):
        prefix = '/shadow_test_' + uuid.uuid4().hex
        config = tmp_path / (prefix[1:] + '.yaml')
        config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters(enabled)}}))
        args = [os.environ['ADAPTIVE_CPG_BINARY'], '--ros-args', '--params-file', str(config),
                '-r', '__ns:=' + prefix]
        log_file = config.with_suffix('.log')
        stream = log_file.open('w')
        log_files.append(stream)
        proc = subprocess.Popen(args, stdout=stream, stderr=stream)
        processes.append(proc)
        references, debug = [], []
        node.create_subscription(JointState, prefix + '/joint_ref_shadow', references.append, 100)
        node.create_subscription(Float64MultiArray, prefix + '/learning_debug', debug.append, 100)
        publisher = node.create_publisher(JointState, prefix + '/measured_joint_states', 10)

        def spin(duration):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.002)
                assert proc.poll() is None, log_file.read_text()

        spin(.5)
        assert references and debug
        assert len(debug[-1].data) == 74
        return references, debug, publisher, spin

    try:
        yield node, create
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for proc in processes:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        for stream in log_files:
            stream.close()


def test_disabled_by_default_and_shadow_output(start):
    node, create = start
    references, debug, publisher, spin = create(False)
    for _ in range(10):
        publisher.publish(JointState(header=Header(
            stamp=node.get_clock().now().to_msg()), name=NAMES, position=[0.0] * 6))
        spin(.01)
    assert debug[-1].data[1] == 0
    assert debug[-1].data[11] == 0
    assert debug[-1].data[2] == 0
    assert all(math.isfinite(v) for v in references[-1].position + references[-1].velocity)
    # No node in this isolated domain should publish the controller input topic.
    assert node.count_publishers('/cpg/joint_ref') == 0


def test_six_joint_measurements_and_dropout(start):
    node, create = start
    references, debug, publisher, spin = create(True)
    gait = flat_prior()
    begin = node.get_clock().now().nanoseconds * 1e-9
    last = None
    for _ in range(400):
        stamp = node.get_clock().now()
        phase = .4 + gait['omega'] * (stamp.nanoseconds * 1e-9 - begin)
        values = []
        for name in NAMES:
            joint = gait['joints'][name]
            values.append(joint['w_offset'] + sum(
                x * math.cos(n * phase) + y * math.sin(n * phase)
                for n, (x, y) in enumerate(zip(joint['w_x'], joint['w_y'])) if n))
        msg = JointState()
        msg.header.stamp = stamp.to_msg()
        msg.name, msg.position = NAMES, values
        publisher.publish(msg)
        last = msg
        spin(.01)
    assert debug[-1].data[1] == 1
    assert debug[-1].data[11] > 150
    assert any(msg.data[2] == 2 for msg in debug), 'shared phase did not lock on unchanged flat_walk'
    assert all(math.isfinite(v) for msg in references for v in msg.position + msg.velocity)
    spin(.03)  # drain the final new measurement before testing replay rejection
    accepted = debug[-1].data[11]
    for _ in range(25):
        publisher.publish(last)  # replayed stamps do not preserve freshness
        spin(.01)
    assert debug[-1].data[11] == accepted
    assert debug[-1].data[2] == 3
    assert debug[-1].data[9] == 0
    assert debug[-1].data[8] > .1


def test_measurement_alias_is_rejected(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    config = tmp_path / 'alias.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters(True)}}))
    proc = subprocess.run(
        [os.environ['ADAPTIVE_CPG_BINARY'], '--ros-args', '--params-file', str(config),
         '-r', '__ns:=/invalid_shadow_' + uuid.uuid4().hex,
         '-r', 'measured_joint_states:=joint_ref_shadow'],
        capture_output=True, timeout=5,
        env=dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='181'))
    assert proc.returncode != 0
    assert b'Measurement input must not alias shadow output' in proc.stderr
