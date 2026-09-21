#!/usr/bin/env python3
"""Check that required recording topics exist with the expected ROS types."""

import argparse
import sys
import time

import rclpy
import yaml
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--wait', type=float, default=15.0)
    args = parser.parse_args()

    with open(args.config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    required = config['required_topics']
    optional = config.get('optional_topics', {})
    required_content = config.get('required_joint_content', {})

    rclpy.init()
    node = Node('recording_topic_checker')
    observed_joints = {}
    observed_interfaces = {}

    def joint_state_callback(msg):
        observed_joints['/joint_states'] = set(msg.name)

    def dynamic_joint_state_callback(msg):
        observed_joints['/dynamic_joint_states'] = set(msg.joint_names)
        observed_interfaces['/dynamic_joint_states'] = {
            name: set(values.interface_names)
            for name, values in zip(msg.joint_names, msg.interface_values)
        }

    if '/joint_states' in required_content:
        node.create_subscription(
            JointState, '/joint_states', joint_state_callback, 10)
    if '/dynamic_joint_states' in required_content:
        node.create_subscription(
            DynamicJointState, '/dynamic_joint_states',
            dynamic_joint_state_callback, 10)

    deadline = time.monotonic() + args.wait
    graph = {}
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        graph = dict(node.get_topic_names_and_types())
        content_ready = all(topic in observed_joints for topic in required_content)
        if all(topic in graph for topic in required) and content_ready:
            break

    failures = []
    for topic, spec in required.items():
        if topic not in graph:
            failures.append(f'MISSING {topic} ({spec["type"]})')
        elif spec['type'] not in graph[topic]:
            failures.append(
                f'TYPE {topic}: expected {spec["type"]}, got {graph[topic]}')

    for topic, spec in required_content.items():
        expected_joints = set(spec.get('joints', []))
        missing_joints = expected_joints - observed_joints.get(topic, set())
        if topic not in observed_joints:
            failures.append(f'NO_DATA {topic}: no message received')
            continue
        if missing_joints:
            failures.append(
                f'CONTENT {topic}: missing joints {sorted(missing_joints)}')
        expected_interfaces = set(spec.get('interfaces', []))
        if expected_interfaces:
            by_joint = observed_interfaces.get(topic, {})
            for joint in expected_joints:
                missing = expected_interfaces - by_joint.get(joint, set())
                if missing:
                    failures.append(
                        f'CONTENT {topic} {joint}: missing interfaces {sorted(missing)}')

    optional_missing = [topic for topic in optional if topic not in graph]
    if failures:
        print('[FAIL] Required recording topics are not ready:')
        for item in failures:
            print(f'  - {item}')
    else:
        print(f'[OK] All {len(required)} required topics are present with matching types.')
    if optional_missing:
        print('[INFO] Optional topics currently absent:')
        for topic in optional_missing:
            print(f'  - {topic}')

    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
