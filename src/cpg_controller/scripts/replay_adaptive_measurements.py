#!/usr/bin/env python3
"""Replay deformed flat_walk as 100 Hz JointState measurements for the shadow CPG.

Uses the same truth/template helpers and 500 Hz generation grid as the offline
experiment. Only angles and optional synthetic contact events enter the learner;
truth phase/frequency remain private to generation and optional CSV scoring.
"""
import argparse
from collections import deque
import csv
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from exo_state_estimator.msg import GaitEvent

from evaluate_adaptive_learning import CASES, JOINTS, template, truth


class AdaptiveMeasurementReplay(Node):
    def __init__(self, args):
        super().__init__('adaptive_measurement_replay', namespace=args.namespace)
        self.args = args
        self.omega, self.prior = template(args.prior_file)
        self.time_grid = np.arange(round(args.duration / .002) + 1) * .002
        self.phase, _, self.clean, _ = truth(self.time_grid, args.case, self.omega, self.prior)
        measured = self.clean.copy()
        self.delay = .03 if args.case == 'combined' else 0.
        if args.case == 'combined':
            # Matches the default six-case offline run, including sample indices.
            rng = np.random.default_rng(20261023 + CASES.index(args.case))
            measured += rng.normal(size=measured.shape) * np.array([.002, .003, .001, .002, .003, .001])
            measured[np.abs(self.time_grid - 32) < .001, 0] += .7
        self.measured = measured
        self.measurements = deque(k for k in range(0, len(self.time_grid), 5)
                                  if not self.dropout(self.time_grid[k]))
        self.events = deque()
        event_ids = [0, 0]
        if args.events:
            for k in range(1, len(self.time_grid)):
                before, after = int(np.floor(self.phase[k-1] / np.pi)), int(np.floor(self.phase[k] / np.pi))
                if after > before and not self.dropout(self.time_grid[k]):
                    crossing = self.time_grid[k-1] + (after*np.pi-self.phase[k-1]) / (
                        self.phase[k]-self.phase[k-1]) * .002
                    foot = after % 2
                    event_ids[foot] += 1
                    self.events.append((crossing, foot, event_ids[foot]))
        self.measurement_pub = self.create_publisher(JointState, 'measured_joint_states', qos_profile_sensor_data)
        self.event_pub = self.create_publisher(GaitEvent, 'gait_events', qos_profile_sensor_data) if args.events else None
        self.create_subscription(Float64MultiArray, 'learning_debug', self.debug_callback, 10)
        self.create_subscription(JointState, 'joint_ref_shadow', self.reference_callback, 10)
        self.debug = self.reference = None
        self.latest_measurement_time = float('nan')
        self.latest_measurement = np.full(6, np.nan)
        self.measurement_count = self.event_count = 0
        self.start_ns = None
        self.created_ns = self.get_clock().now().nanoseconds
        self.finished = False
        self.exit_code = 0
        self.output = args.output.open('w', newline='') if args.output else None
        self.writer = csv.writer(self.output) if self.output else None
        if self.writer:
            self.writer.writerow([
                'elapsed', 'measurement_source_time', 'measurement_delay', 'measurements_published',
                'events_published', 'learner_time', 'state', 'phase', 'omega_base', 'phase_rate',
                'shape_active', 'accepted_measurements', 'accepted_events', 'truth_phase', 'truth_omega',
                *['truth_q'+str(j) for j in range(6)], *['measured_q'+str(j) for j in range(6)],
                *['raw_q'+str(j) for j in range(6)], *['shadow_q'+str(j) for j in range(6)]])
        self.timer = self.create_timer(.01, self.tick)

    def dropout(self, t):
        return self.args.case == 'combined' and 24 <= t < 24.5

    def debug_callback(self, msg):
        if len(msg.data) == 74:
            self.debug = msg.data

    def reference_callback(self, msg):
        if list(msg.name) == JOINTS and len(msg.position) == 6:
            self.reference = msg

    def message_stamp(self, relative_time):
        return rclpy.time.Time(nanoseconds=self.start_ns + round(relative_time*1e9)).to_msg()

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.start_ns is None:
            # Discovery is outside the synthetic experiment's time coordinate.
            if self.measurement_pub.get_subscription_count() and self.debug is not None:
                self.start_ns = now_ns
                self.get_logger().info(
                    f'Synthetic {self.args.case}: {self.args.duration:g}s, 100Hz, '
                    f'events={self.args.events}, namespace={self.get_namespace()}')
            elif (now_ns-self.created_ns)*1e-9 > 5:
                self.get_logger().error('No shadow learner discovered within 5s; start adaptive_cpg_shadow first')
                self.exit_code, self.finished = 1, True
            return
        elapsed = (now_ns-self.start_ns)*1e-9
        while self.measurements and self.time_grid[self.measurements[0]] + self.delay <= elapsed:
            k = self.measurements.popleft()
            source_time = float(self.time_grid[k])
            message = JointState()
            message.header.stamp = self.message_stamp(source_time)
            message.name = JOINTS
            message.position = self.measured[k].tolist()
            self.measurement_pub.publish(message)
            self.measurement_count += 1
            self.latest_measurement_time = source_time
            self.latest_measurement = self.measured[k]
        while self.events and self.events[0][0] + self.delay <= elapsed:
            crossing, foot, event_id = self.events.popleft()
            event = GaitEvent()
            event.header.stamp = self.message_stamp(crossing)
            event.foot_id = GaitEvent.LEFT if foot == 0 else GaitEvent.RIGHT
            event.event = GaitEvent.INITIAL_CONTACT
            event.event_id, event.confidence, event.valid = event_id, 1.0, True
            self.event_pub.publish(event)
            self.event_count += 1
        if self.writer and self.debug is not None:
            d = self.debug
            learner_time = d[0] - self.start_ns*1e-9
            phase, omega, clean, _ = truth(np.array([learner_time]), self.args.case, self.omega, self.prior)
            reference = self.reference.position if self.reference else [float('nan')]*6
            self.writer.writerow([
                elapsed, self.latest_measurement_time, elapsed-self.latest_measurement_time,
                self.measurement_count, self.event_count, learner_time, d[2], d[3], d[4], d[5],
                d[9], d[11], d[13], phase[0], omega[0], *clean[0], *self.latest_measurement,
                *d[14:20], *reference])
        if elapsed >= self.args.duration + self.delay + .05:
            self.get_logger().info(
                f'Replay complete: {self.measurement_count} measurements, {self.event_count} synthetic events')
            self.finished = True

    def destroy_node(self):
        if self.output:
            self.output.close()
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES, default='combined')
    parser.add_argument('--duration', type=float, default=50.)
    parser.add_argument('--prior-file', type=Path, default=Path(
        get_package_share_directory('cpg_controller')) / 'config' / 'cpg.yaml')
    parser.add_argument('--namespace', default='cpg_adaptive')
    parser.add_argument('--events', action='store_true', help='Publish explicitly synthetic 0/pi contact events')
    parser.add_argument('--output', type=Path, help='Optional CSV; truth fields are scored privately, not published')
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be finite and positive')
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=sys.argv)
    node = AdaptiveMeasurementReplay(args)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        result = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    sys.exit(main())
