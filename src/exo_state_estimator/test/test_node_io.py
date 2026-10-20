#!/usr/bin/env python3
"""Isolated synthetic ROS test; starts only estimator and synthetic publishers."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 70)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='exo_estimator_ros_test_')
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from std_msgs.msg import UInt8
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node('synthetic_estimator_io_test')
    received = []
    diagnostics = []
    contacts = []
    node.create_subscription(Odometry, '/fused/odometry', received.append, qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, '/fused/diagnostics', diagnostics.append, 10)
    node.create_subscription(UInt8, '/fused/contact_state', lambda msg: contacts.append(msg.data), 10)
    publisher = node.create_publisher(Odometry, '/synthetic/pelvis_vio', 10)
    joints = node.create_publisher(JointState, '/synthetic/joints', 10)
    log_path = Path(os.environ['ROS_LOG_DIR']) / 'estimator.log'
    log_file = log_path.open('w')
    process = subprocess.Popen([
        args.executable, '--ros-args', '-p', 'use_imu_factor:=false',
        '-p', 'vio_topic:=/synthetic/pelvis_vio', '-p', 'joint_state_topic:=/synthetic/joints',
        '-p', 'fusion_delay:=0.02', '-p', 'input_timeout:=1.0',
        '-p', 'prior_rot_sigma:=0.1', '-p', 'prior_pos_sigma:=0.02',
    ], stdout=log_file, stderr=subprocess.STDOUT)

    def spin_until(condition, timeout):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError(f'estimator exited: {log_path.read_text()}')
            rclpy.spin_once(node, timeout_sec=0.01)
        assert condition(), f'condition timeout; estimator log: {log_path.read_text()}'

    def spin_for(duration):
        start = time.monotonic()
        spin_until(lambda: time.monotonic() - start >= duration, duration + .2)

    try:
        spin_until(lambda: publisher.get_subscription_count() > 0 and
                   joints.get_subscription_count() > 0, 8)
        stamp = node.get_clock().now().nanoseconds - 200_000_000

        def odom(stamp_ns, x, child='base_link'):
            msg = Odometry()
            msg.header.stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
            msg.header.frame_id = 'global'
            msg.child_frame_id = child
            msg.pose.pose.position.x = float(x)
            msg.pose.pose.orientation.z = math.sqrt(.5)
            msg.pose.pose.orientation.w = math.sqrt(.5)
            return msg

        # Malformed joint arrays must not crash or be silently completed with zeros.
        invalid_joints = JointState()
        invalid_joints.name = ['left_hip3']
        joints.publish(invalid_joints)
        publisher.publish(odom(stamp, 100, 'imu'))  # Raw moving-leg frame rejected.
        publisher.publish(odom(stamp, .1))
        publisher.publish(odom(stamp + 50_000_000, .15))
        spin_until(lambda: len(received) >= 2, 3)
        assert len(received) == 2
        first, second = received
        actual_ns = second.header.stamp.sec * 1_000_000_000 + second.header.stamp.nanosec
        assert abs(actual_ns - (stamp + 50_000_000)) <= 500
        assert abs(second.pose.pose.position.x - .15) < 1e-6
        # World +x velocity with 90-deg yaw must become body -y.
        assert abs(second.twist.twist.linear.x) < 1e-6
        assert abs(second.twist.twist.linear.y + 1) < 1e-5
        assert abs(first.pose.covariance[0] - .0004) < 1e-8
        assert abs(first.pose.covariance[21] - .01) < 1e-8
        assert second.twist.covariance[35] > 1e5
        assert contacts and all(value == 4 for value in contacts)
        publisher.publish(odom(stamp + 50_000_000, 200))  # Duplicate.
        publisher.publish(odom(stamp + 25_000_000, 300))  # Out of order.
        spin_for(.3)
        assert len(received) == 2
        spin_for(1.0)
        assert len(received) == 2, 'Cached estimate was republished with a new stamp'
        assert diagnostics[-1].status[0].level != 0
        assert 'No fresh estimate' in diagnostics[-1].status[0].message
        print('ROS I/O passed: frame validation, missing joints, source stamps, body velocity, '
              'covariance order, duplicates, unknown contact and stale-output diagnostics.')
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
