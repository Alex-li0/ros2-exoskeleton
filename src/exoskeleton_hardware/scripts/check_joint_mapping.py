#!/usr/bin/env python3
"""Identify which ROS joint moves during a manual single-joint check."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINTS = ('left_hip3', 'left_knee', 'right_hip3', 'right_knee')


class JointMonitor(Node):
    """Keep the most recent named joint positions."""

    def __init__(self):
        super().__init__('joint_mapping_checker')
        self.positions = {}
        self.message_count = 0
        self.create_subscription(
            JointState, '/joint_states', self._callback, qos_profile_sensor_data)

    def _callback(self, msg):
        for index, name in enumerate(msg.name):
            if index < len(msg.position):
                self.positions[name] = float(msg.position[index])
        self.message_count += 1


def spin_for(node, duration):
    """Spin a node for a bounded wall-clock duration."""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))


def wait_for_joints(node, timeout):
    """Wait until all expected joints have appeared."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(name in node.positions for name in JOINTS):
            return True
    return False


def main():
    parser = argparse.ArgumentParser(
        description='手动移动一个物理关节，检查它对应的ROS关节名和方向')
    parser.add_argument('--expected-joint', required=True, choices=JOINTS)
    parser.add_argument(
        '--expected-direction', choices=('ignore', 'positive', 'negative'),
        default='ignore',
        help='移动并保持到窗口结束时，期望ROS角度的变化方向')
    parser.add_argument('--duration', type=float, default=8.0)
    parser.add_argument('--prepare-seconds', type=float, default=3.0)
    parser.add_argument('--wait', type=float, default=10.0)
    parser.add_argument('--min-motion-deg', type=float, default=3.0)
    args, ros_args = parser.parse_known_args()

    if args.duration <= 0.0 or args.prepare_seconds < 0.0 or args.min_motion_deg <= 0.0:
        parser.error('duration和min-motion-deg必须为正，prepare-seconds不能为负')

    rclpy.init(args=ros_args)
    node = JointMonitor()
    if not wait_for_joints(node, args.wait):
        print('[FAIL] 未在规定时间内收到四个关节的 /joint_states。')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    print(
        f'[READY] 准备检查物理关节 {args.expected_joint}。\n'
        f'请只移动这一个关节；{args.prepare_seconds:.1f}秒后开始采集，'
        f'采集持续{args.duration:.1f}秒。')
    spin_for(node, args.prepare_seconds)

    initial = {name: node.positions[name] for name in JOINTS}
    minimum = dict(initial)
    maximum = dict(initial)
    start_count = node.message_count
    deadline = time.monotonic() + args.duration
    print('[MEASURE] 现在缓慢移动目标关节；若检查方向，请向规定方向移动并保持。')
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        for name in JOINTS:
            value = node.positions.get(name, initial[name])
            minimum[name] = min(minimum[name], value)
            maximum[name] = max(maximum[name], value)

    final = {name: node.positions[name] for name in JOINTS}
    received = node.message_count - start_count
    node.destroy_node()
    rclpy.shutdown()
    if received < 2:
        print('[FAIL] 采集窗口内 /joint_states 数据不足。')
        return 2

    ranges = {name: maximum[name] - minimum[name] for name in JOINTS}
    deltas = {name: final[name] - initial[name] for name in JOINTS}
    dominant = max(JOINTS, key=lambda name: ranges[name])
    threshold = math.radians(args.min_motion_deg)

    print('\n检测结果（角度单位：deg）：')
    for name in JOINTS:
        print(
            f'  {name:12s} range={math.degrees(ranges[name]):8.3f} '
            f'final_delta={math.degrees(deltas[name]):+8.3f}')

    mapping_ok = dominant == args.expected_joint and ranges[dominant] >= threshold
    if mapping_ok:
        print(f'[PASS] 最大运动关节为 {dominant}，物理关节与ROS名称对应。')
    elif ranges[dominant] < threshold:
        print(
            f'[FAIL] 最大运动仅{math.degrees(ranges[dominant]):.2f}度，'
            '低于阈值；请增大手动位移后重试。')
    else:
        print(
            f'[FAIL] 期望 {args.expected_joint}，实际最大运动关节为 {dominant}；'
            '检查joint_motor_map.motor_id。')

    direction_ok = True
    delta = deltas[args.expected_joint]
    if args.expected_direction == 'positive':
        direction_ok = delta >= threshold
    elif args.expected_direction == 'negative':
        direction_ok = delta <= -threshold

    if args.expected_direction != 'ignore':
        if direction_ok:
            print(
                f'[PASS] {args.expected_joint} 的最终角度变化符合'
                f'{args.expected_direction}方向。')
        else:
            print(
                f'[FAIL] {args.expected_joint} 的方向不符合预期；确认移动后保持到'
                '窗口结束，若仍相反则翻转joint_sign。')

    return 0 if mapping_ok and direction_ok else 1


if __name__ == '__main__':
    sys.exit(main())
