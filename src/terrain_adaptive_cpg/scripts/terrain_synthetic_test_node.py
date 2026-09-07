#!/usr/bin/env python3
"""simulation_only publisher for scheduler safety-path offline checks."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String, UInt8

from terrain_adaptive_cpg.msg import CPGDebug, TerrainModulation, TerrainSchedulerDebug


class SyntheticTerrainTest(Node):
    def __init__(self):
        super().__init__('terrain_synthetic_test_node')
        self.started = self.get_clock().now()
        self.terrain_pub = self.create_publisher(TerrainModulation, '/terrain/modulation', 10)
        self.odom_pub = self.create_publisher(Odometry, '/fused/odometry', 10)
        self.contact_pub = self.create_publisher(UInt8, '/fused/contact_state', 10)
        self.state_pub = self.create_publisher(String, '/cpg/state_cmd', 10)
        self.mode_pub = self.create_publisher(String, '/cpg/gait_mode_cmd', 10)
        self.result_pub = self.create_publisher(String, '/terrain/test_result', 10)
        self.create_subscription(TerrainSchedulerDebug, '/terrain/scheduler_debug', self.on_scheduler, 10)
        self.create_subscription(CPGDebug, '/cpg/debug', self.on_cpg, 10)
        self.reasons = set()
        self.saw_limited_target = False
        self.saw_switch = False
        self.sent_start = False
        self.sent_mode = False
        self.timer = self.create_timer(0.05, self.tick)

    def on_scheduler(self, msg):
        self.reasons.add(msg.decision_reason)

    def on_cpg(self, msg):
        if msg.continuous_modulation_active and msg.amplitude_scale:
            self.saw_limited_target |= max(msg.amplitude_scale) <= 1.300001
        self.saw_switch |= msg.state == 'SWITCHING' or msg.gait_mode == 'slope_up'

    def publish_state_inputs(self, contact):
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'global'
        odom.child_frame_id = 'pelvis_fused'
        odom.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[index] = 0.01
        self.odom_pub.publish(odom)
        self.contact_pub.publish(UInt8(data=contact))

    def tick(self):
        elapsed = (self.get_clock().now() - self.started).nanoseconds * 1e-9
        if not self.sent_start and elapsed > 0.2:
            self.state_pub.publish(String(data='start'))
            self.sent_start = True

        # 0-1 s: structurally invalid input; unavailable values remain invalid.
        if elapsed < 1.0:
            self.publish_state_inputs(3)
            message = TerrainModulation()
            message.stamp = self.get_clock().now().to_msg()
            message.terrain_mode = 'synthetic_invalid'
            message.terrain_mode_valid = False
            message.simulation_only = True
            self.terrain_pub.publish(message)
        # 1-2 s: valid fields but rejected confidence.
        elif elapsed < 2.0:
            self.publish_state_inputs(3)
            message = TerrainModulation()
            message.stamp = self.get_clock().now().to_msg()
            message.terrain_mode = 'synthetic_stairs'
            message.terrain_mode_valid = True
            message.confidence = 0.2
            message.confidence_valid = True
            message.simulation_only = True
            self.terrain_pub.publish(message)
        # 2-4 s: valid synthetic request. First right stance, then double stance.
        elif elapsed < 4.0:
            self.publish_state_inputs(1 if elapsed < 2.7 else 3)
            message = TerrainModulation()
            message.stamp = self.get_clock().now().to_msg()
            message.terrain_mode = 'synthetic_stairs'
            message.terrain_mode_valid = True
            message.confidence = 0.9
            message.confidence_valid = True
            message.simulation_only = True
            self.terrain_pub.publish(message)
            if not self.sent_mode:
                self.mode_pub.publish(String(data='slope_up'))
                self.sent_mode = True
        # 4-5 s: intentionally stop terrain messages to exercise timeout fallback.
        elif elapsed < 5.2:
            self.publish_state_inputs(3)
        else:
            required = {'invalid_terrain_fields', 'confidence_rejected_or_unconfigured', 'terrain_timeout'}
            passed = required.issubset(self.reasons) and self.saw_limited_target and self.saw_switch
            result = String(data=(
                f"simulation_only {'PASS' if passed else 'FAIL'}; "
                f"reasons={sorted(self.reasons)}; limited={self.saw_limited_target}; switch={self.saw_switch}"))
            self.result_pub.publish(result)
            self.get_logger().info(result.data)
            rclpy.shutdown()


def main():
    rclpy.init()
    rclpy.spin(SyntheticTerrainTest())


if __name__ == '__main__':
    main()
