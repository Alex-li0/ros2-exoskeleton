#!/usr/bin/env python3
"""Validate rosbag topic coverage and approximate whole-bag message rates."""

import argparse
import os
import sys

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='Bag directory containing metadata.yaml')
    parser.add_argument('--config', required=True)
    args = parser.parse_args()

    metadata_path = os.path.join(args.bag, 'metadata.yaml')
    if not os.path.isfile(metadata_path):
        print(f'[FAIL] metadata.yaml not found: {metadata_path}')
        return 2

    with open(args.config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    with open(metadata_path, encoding='utf-8') as stream:
        info = yaml.safe_load(stream)['rosbag2_bagfile_information']

    duration = info['duration']['nanoseconds'] / 1e9
    recorded = {}
    for entry in info.get('topics_with_message_count', []):
        meta = entry['topic_metadata']
        recorded[meta['name']] = (meta['type'], int(entry['message_count']))

    failures = []
    print(f'Bag duration: {duration:.3f} s, total messages: {info["message_count"]}')
    for topic, spec in config['required_topics'].items():
        if topic not in recorded:
            failures.append(f'MISSING {topic}')
            continue
        msg_type, count = recorded[topic]
        rate = count / duration if duration > 0 else 0.0
        if msg_type != spec['type']:
            failures.append(f'TYPE {topic}: {msg_type} != {spec["type"]}')
        min_rate = float(spec.get('min_rate_hz', 0.0))
        # 离散状态和 transient-local 话题只要求至少一条消息。
        if min_rate == 0.0 and count == 0:
            failures.append(f'EMPTY {topic}')
        elif min_rate > 0.0 and rate < min_rate:
            failures.append(
                f'RATE {topic}: {rate:.2f} Hz < {min_rate:.2f} Hz')
        print(f'  {topic}: {count} msgs, {rate:.2f} Hz')

    optional_present = sum(
        topic in recorded for topic in config.get('optional_topics', {}))
    print(
        f'Optional coverage: {optional_present}/'
        f'{len(config.get("optional_topics", {}))}')

    if failures:
        print('[FAIL] Bag is incomplete:')
        for item in failures:
            print(f'  - {item}')
        return 1
    print('[OK] Required topic coverage and approximate rates passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
