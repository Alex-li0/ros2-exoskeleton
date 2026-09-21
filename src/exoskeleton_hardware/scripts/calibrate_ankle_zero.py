#!/usr/bin/env python3
"""Estimate BRT25 mechanical-zero counts from /dynamic_joint_states."""

import argparse
import math
import sys
import time

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node


JOINTS = ('left_ankle', 'right_ankle')
RESOLUTION = 4096


class AnkleZeroCalibrator(Node):
    def __init__(self, sample_count):
        super().__init__('ankle_zero_calibrator')
        self.sample_count = sample_count
        self.samples = {name: [] for name in JOINTS}
        self.messages_received = 0
        self.seen = {name: 0 for name in JOINTS}
        self.invalid = {name: 0 for name in JOINTS}
        self.stale = {name: 0 for name in JOINTS}
        self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self._callback, 50)

    def _callback(self, msg):
        self.messages_received += 1
        for name, interfaces in zip(msg.joint_names, msg.interface_values):
            if name not in self.samples:
                continue
            self.seen[name] += 1
            values = dict(zip(interfaces.interface_names, interfaces.values))
            if values.get('feedback_valid', 0.0) < 0.5:
                self.invalid[name] += 1
                continue
            if values.get('stale', 1.0) > 0.5 or 'raw_count' not in values:
                self.stale[name] += 1
                continue
            target = self.samples[name]
            if len(target) < self.sample_count:
                target.append(int(round(values['raw_count'])) % RESOLUTION)

    def complete(self):
        return all(len(values) >= self.sample_count for values in self.samples.values())


def circular_mean_count(values):
    angles = [2.0 * math.pi * value / RESOLUTION for value in values]
    mean_angle = math.atan2(
        sum(math.sin(angle) for angle in angles),
        sum(math.cos(angle) for angle in angles))
    return int(round((mean_angle % (2.0 * math.pi)) * RESOLUTION /
                     (2.0 * math.pi))) % RESOLUTION


def main():
    parser = argparse.ArgumentParser(
        description='Hold both ankles at mechanical zero and estimate zero_offset_counts')
    parser.add_argument('--samples', type=int, default=500)
    parser.add_argument('--timeout', type=float, default=15.0)
    args = parser.parse_args()
    if args.samples <= 0 or args.timeout <= 0.0:
        parser.error('--samples and --timeout must be positive')

    rclpy.init()
    node = AnkleZeroCalibrator(args.samples)
    print('[SAFETY] Keep the exoskeleton supported and both ankles at the defined '
          'mechanical zero. This tool is read-only and sends no motor command.', flush=True)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and time.monotonic() < deadline and not node.complete():
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.complete():
        publishers = node.count_publishers('/dynamic_joint_states')
        if publishers == 0:
            print('[FAIL] /dynamic_joint_states has no publisher. Start the hardware '
                  'bringup and joint_state_broadcaster before calibration.', flush=True)
        elif node.messages_received == 0:
            print(f'[FAIL] /dynamic_joint_states has {publishers} publisher(s), but no '
                  'message was received. Check ROS_DOMAIN_ID and QoS.', flush=True)
        else:
            details = ', '.join(
                f'{name}: seen={node.seen[name]} valid_fresh={len(node.samples[name])} '
                f'invalid={node.invalid[name]} stale_or_missing={node.stale[name]}'
                for name in JOINTS)
            print(f'[FAIL] Received {node.messages_received} dynamic-state messages, '
                  f'but not enough usable ankle samples ({details}). Expected '
                  f'{args.samples} per side.', flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    offsets = [circular_mean_count(node.samples[name]) for name in JOINTS]
    spreads = []
    for name, offset in zip(JOINTS, offsets):
        deltas = [((value - offset + RESOLUTION // 2) % RESOLUTION) -
                  RESOLUTION // 2 for value in node.samples[name]]
        spreads.append(max(deltas) - min(deltas))

    print(f'[OK] zero_offset_counts: [{offsets[0]}, {offsets[1]}]')
    print(f'     peak-to-peak noise [counts]: [{spreads[0]}, {spreads[1]}]')
    print('Copy the zero_offset_counts line into '
          'exo_controllers.yaml -> ankle_encoder_map -> ros__parameters, then restart the launch.')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
