#!/usr/bin/env python3
"""Check that required recording topics exist with the expected ROS types."""

import argparse
import sys
import time

import rclpy
import yaml
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--wait', type=float, default=15.0)
    args = parser.parse_args()

    with open(args.config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    required = config['required_topics']
    optional = config.get('optional_topics', {})

    rclpy.init()
    node = Node('recording_topic_checker')
    deadline = time.monotonic() + args.wait
    graph = {}
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        graph = dict(node.get_topic_names_and_types())
        if all(topic in graph for topic in required):
            break

    failures = []
    for topic, spec in required.items():
        if topic not in graph:
            failures.append(f'MISSING {topic} ({spec["type"]})')
        elif spec['type'] not in graph[topic]:
            failures.append(
                f'TYPE {topic}: expected {spec["type"]}, got {graph[topic]}')

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
