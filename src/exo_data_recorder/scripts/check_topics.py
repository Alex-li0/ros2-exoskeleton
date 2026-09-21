#!/usr/bin/env python3
"""Check required recording topic names and types before an experiment."""

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

    rclpy.init()
    node = Node('exo_recording_topic_checker')
    deadline = time.monotonic() + args.wait
    graph = {}
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        graph = dict(node.get_topic_names_and_types())
        if all(topic in graph for topic in config['required_topics']):
            break

    failures = []
    for topic, spec in config['required_topics'].items():
        if topic not in graph:
            failures.append(f'MISSING {topic} ({spec["type"]})')
        elif spec['type'] not in graph[topic]:
            failures.append(f'TYPE {topic}: expected {spec["type"]}, got {graph[topic]}')
    missing_optional = [
        topic for topic in config.get('optional_topics', {}) if topic not in graph]

    if failures:
        print('[FAIL] Required topics are not ready:')
        for failure in failures:
            print(f'  - {failure}')
    else:
        print(f'[OK] {len(config["required_topics"])} required topics are ready.')
    if missing_optional:
        print('[INFO] Optional topics currently absent:')
        for topic in missing_optional:
            print(f'  - {topic}')

    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
