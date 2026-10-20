#!/usr/bin/env python3
"""Read CrownTo 18-point insoles through explicit TTL-USB ports for ROS recording."""

import math
import os
import struct
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from exoskeleton_hardware.msg import FootPressureSample
from rclpy.node import Node
import serial


class FrameParser:
    """Slide over fixed 39-byte frames; keep separate statistics for each foot."""

    def __init__(self, foot_id):
        self.foot_id = foot_id
        self.buffer = bytearray()
        self.last_byte_time = None
        self.checksum_errors = 0
        self.wrong_side_frames = 0
        self.partial_timeouts = 0

    def reset(self):
        self.buffer.clear()
        self.last_byte_time = None

    def feed(self, data, now):
        if (self.buffer and self.last_byte_time is not None and
                now - self.last_byte_time > 0.1):
            self.partial_timeouts += 1
            self.reset()
        if data:
            self.last_byte_time = now
            self.buffer.extend(data)
        samples = []
        while self.buffer:
            start = self.buffer.find(b'\xaa')
            if start < 0:
                self.buffer.clear()
                break
            del self.buffer[:start]
            if len(self.buffer) < 2:
                break
            if self.buffer[1] not in (1, 2):
                del self.buffer[0]
                continue
            if len(self.buffer) < 39:
                break
            if sum(self.buffer[:38]) & 0xFF != self.buffer[38]:
                self.checksum_errors += 1
                del self.buffer[0]
                continue
            if self.buffer[1] == self.foot_id:
                samples.append(struct.unpack('>18H', self.buffer[2:38]))
            else:
                self.wrong_side_frames += 1
            del self.buffer[:39]
        return samples


class PressurePort:
    """Own one nonblocking serial port; never transmit sensor or motor commands."""

    def __init__(self, path, foot_id, baud_rate):
        self.path = path
        self.foot_id = foot_id
        self.baud_rate = baud_rate
        self.parser = FrameParser(foot_id)
        self.serial = None
        self.retry_at = 0.0
        self.last_sample_time = None
        self.sample_count = 0
        self.connections = 0
        self.last_error = ''

    def close(self):
        if self.serial is not None:
            self.serial.close()
            self.serial = None
        self.parser.reset()

    def read(self, now):
        if self.serial is None:
            if now < self.retry_at:
                return []
            self.retry_at = now + 1.0
            try:
                self.serial = serial.Serial(
                    self.path, baudrate=self.baud_rate, timeout=0, exclusive=True)
                self.connections += 1
                self.last_sample_time = None
                self.last_error = ''
            except (serial.SerialException, OSError) as exc:
                self.last_error = str(exc)
                return []
        try:
            samples = self.parser.feed(self.serial.read(4096), now)
        except (serial.SerialException, OSError) as exc:
            self.last_error = str(exc)
            self.close()
            self.retry_at = now + 1.0
            return []
        result = []
        for values in samples:
            self.sample_count += 1
            self.last_sample_time = now
            result.append((self.sample_count, values))
        return result

    def health(self, now, stale_timeout):
        age = math.inf if self.last_sample_time is None else now - self.last_sample_time
        if self.serial is None:
            return DiagnosticStatus.ERROR, 'serial disconnected', age
        if age > stale_timeout:
            return DiagnosticStatus.WARN, 'waiting for fresh valid samples', age
        return DiagnosticStatus.OK, 'receiving', age


class FootPressureReader(Node):
    """Publish only new validated samples plus independently timed health reports."""

    def __init__(self, **kwargs):
        super().__init__('foot_pressure_reader', **kwargs)
        self.ports = []
        try:
            left = self.declare_parameter('left_port', '').value
            right = self.declare_parameter('right_port', '').value
            baud = self.declare_parameter('baud_rate', 115200).value
            self.stale_timeout = self.declare_parameter('stale_timeout_s', 0.2).value
            if not left and not right:
                raise ValueError('Set left_port and/or right_port to explicit TTL-USB paths')
            if left and right and os.path.realpath(left) == os.path.realpath(right):
                raise ValueError('Left and right sensors must use different serial ports')
            if baud not in (9600, 115200):
                raise ValueError('baud_rate must be 9600 or 115200')
            if not math.isfinite(self.stale_timeout) or self.stale_timeout <= 0:
                raise ValueError('stale_timeout_s must be finite and positive')
            self.ports = [PressurePort(path, foot, baud)
                          for foot, path in ((1, left), (2, right)) if path]
            self.raw_pub = self.create_publisher(FootPressureSample, 'foot_pressure/raw', 100)
            self.health_pub = self.create_publisher(DiagnosticArray, 'foot_pressure/health', 10)
            self.create_timer(0.005, self.poll)
            self.create_timer(0.1, self.publish_health)
        except Exception:
            self.destroy_node()
            raise
        self.get_logger().info('Foot pressure recording reader: explicit ports, no serial writes')

    def poll(self):
        now = time.monotonic()
        for port in self.ports:
            for index, values in port.read(now):
                msg = FootPressureSample()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.foot_id = port.foot_id
                msg.raw_g = list(values)
                msg.sample_index = index
                self.raw_pub.publish(msg)

    def publish_health(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        now = time.monotonic()
        for port in self.ports:
            status = DiagnosticStatus()
            status.name = 'foot_pressure/' + ('left' if port.foot_id == 1 else 'right')
            status.hardware_id = port.path
            status.level, status.message, age = port.health(now, self.stale_timeout)
            fields = {
                'host_age_s': age,
                'accepted_samples': port.sample_count,
                'checksum_errors': port.parser.checksum_errors,
                'wrong_side_frames': port.parser.wrong_side_frames,
                'partial_timeouts': port.parser.partial_timeouts,
                'connections': port.connections,
                'last_serial_error': port.last_error,
            }
            status.values = [KeyValue(key=key, value=str(value)) for key, value in fields.items()]
            msg.status.append(status)
        self.health_pub.publish(msg)

    def destroy_node(self):
        for port in self.ports:
            port.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FootPressureReader()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
