"""Exercise manufacturer parsing, real PTYs, reconnect and publication without hardware."""

import importlib.util
import os
from pathlib import Path
import struct
from unittest.mock import Mock

import tempfile
import unittest
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'foot_pressure_reader.py'
SPEC = importlib.util.spec_from_file_location('foot_pressure_reader', SCRIPT)
reader = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(reader)


def frame(foot=1, values=None):
    if values is None:
        values = list(range(18))
    data = bytes([0xAA, foot]) + struct.pack('>18H', *values)
    return data + bytes([sum(data) & 0xFF])


class FootPressureTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_every_split_and_big_endian(self):
        for split in range(1, 39):
            with self.subTest(split=split):
                values = [0xAA01, 0x1234] + [65535] * 16
                data = frame(values=values)
                parser = reader.FrameParser(1)
                assert parser.feed(data[:split], 1.0) == []
                assert parser.feed(data[split:], 1.005) == [tuple(values)]
                assert parser.feed(b'', 1.01) == []

    def test_ros_message_serialization(self):
        from rclpy.serialization import deserialize_message, serialize_message
        msg = reader.FootPressureSample(foot_id=2, raw_g=list(range(18)), sample_index=9)
        restored = deserialize_message(serialize_message(msg), reader.FootPressureSample)
        assert restored == msg

    def test_manufacturer_example(self):
        data = bytes.fromhex(
            'AA 01 00 4E 00 11 01 06 00 00 01 6A 00 30 01 3F 00 '
            '0E 00 00 02 BD 00 00 00 B2 00 37 00 AF 01 E2 00 08 '
            '00 15 10 23 84')
        assert reader.FrameParser(1).feed(data, 0) == [
            (78, 17, 262, 0, 362, 48, 319, 14, 0, 701, 0, 178, 55, 175, 482, 8, 21, 4131)]

    def test_corruption_wrong_side_noise_and_concatenation(self):
        parser = reader.FrameParser(1)
        bad = bytearray(frame())
        bad[-1] ^= 1
        data = b'noise' + bytes(bad) + frame(2) + frame() + frame()
        assert parser.feed(data, 0) == [tuple(range(18)), tuple(range(18))]
        assert parser.checksum_errors >= 1
        assert parser.wrong_side_frames == 1
        assert len(parser.buffer) == 0

    def test_missing_byte_resynchronization_and_partial_timeout(self):
        parser = reader.FrameParser(1)
        damaged = frame()[:12] + frame()[13:]
        assert parser.feed(damaged + frame(), 1) == [tuple(range(18))]
        assert parser.feed(frame()[:7], 2) == []
        assert parser.feed(b'', 2.2) == []
        assert parser.partial_timeouts == 1
        assert not parser.buffer
        assert parser.feed(frame(), 2.21) == [tuple(range(18))]
        assert parser.feed(b'\x00' * 10000, 3) == []
        assert len(parser.buffer) <= 38

    def test_two_ptys_independent_stale_disconnect_and_no_transmission(self):
        tmp_path = Path(self.tmp.name)
        masters, slaves, ports = [], [], []
        try:
            for foot in (1, 2):
                master, slave = os.openpty()
                os.set_blocking(master, False)
                masters.append(master)
                slaves.append(slave)
                ports.append(reader.PressurePort(os.ttyname(slave), foot, 115200))
            for port in ports:
                assert port.read(1) == []
                assert port.health(1, 0.2)[0] == DiagnosticStatus.WARN
            # Reading/configuring the node's serial ports must not send bytes to the sensors.
            for master in masters:
                with self.assertRaises(BlockingIOError):
                    os.read(master, 4096)
            os.write(masters[0], frame(1) + frame(1))
            os.write(masters[1], frame(2, [1000] * 18))
            assert ports[0].read(1.01) == [(1, tuple(range(18))), (2, tuple(range(18)))]
            assert ports[1].read(1.01) == [(1, (1000,) * 18)]
            assert ports[0].read(1.02) == []
            assert ports[0].health(1.1, 0.2)[0] == DiagnosticStatus.OK
            assert ports[0].health(1.3, 0.2)[0] == DiagnosticStatus.WARN
            os.close(masters[0])
            masters[0] = -1
            assert ports[0].read(1.31) == []
            assert ports[0].health(1.31, 0.2)[0] == DiagnosticStatus.ERROR
            os.write(masters[1], frame(2))
            assert ports[1].read(1.32) == [(2, tuple(range(18)))]
        finally:
            for port in ports:
                port.close()
            for fd in masters + slaves:
                if fd >= 0:
                    os.close(fd)

    def test_reconnect_through_stable_symlink_requires_new_sample(self):
        tmp_path = Path(self.tmp.name)
        path = tmp_path / 'sensor'
        master, slave = os.openpty()
        path.symlink_to(os.ttyname(slave))
        port = reader.PressurePort(str(path), 1, 115200)
        next_master, next_slave = os.openpty()
        try:
            assert port.read(1) == []
            os.write(master, frame())
            assert port.read(1.01)[0][0] == 1
            os.close(master)
            master = -1
            assert port.read(1.02) == []
            path.unlink()
            path.symlink_to(os.ttyname(next_slave))
            assert port.read(1.5) == []
            assert port.connections == 1
            assert port.read(2.1) == []
            assert port.connections == 2
            assert port.health(2.1, 10)[0] == DiagnosticStatus.WARN
            os.write(next_master, frame())
            assert port.read(2.11)[0][0] == 2
        finally:
            port.close()
            for fd in (master, slave, next_master, next_slave):
                if fd >= 0:
                    os.close(fd)

    def test_missing_port_reports_error_and_retries(self):
        tmp_path = Path(self.tmp.name)
        port = reader.PressurePort(str(tmp_path / 'missing'), 1, 115200)
        assert port.read(1) == []
        assert port.health(1, 0.2)[0] == DiagnosticStatus.ERROR
        assert port.last_error and port.sample_count == 0
        assert port.read(1.5) == []
        assert port.retry_at == 2.0

    def test_node_publish_preserves_raw_and_only_new_samples(self):
        # Drive the actual node callback with injected serial data, without DDS/hardware.
        node = Mock()
        port = reader.PressurePort('/unused', 2, 115200)
        node.ports = [port]
        node.get_clock().now().to_msg.return_value = Time(sec=123, nanosec=456)
        port.serial = Mock()
        port.serial.read.side_effect = [frame(2, [0, 65535] + [12] * 16), b'']
        reader.FootPressureReader.poll(node)
        reader.FootPressureReader.poll(node)
        node.raw_pub.publish.assert_called_once()
        msg = node.raw_pub.publish.call_args.args[0]
        assert msg.foot_id == 2 and msg.sample_index == 1
        assert list(msg.raw_g) == [0, 65535] + [12] * 16
        assert msg.header.stamp == node.get_clock().now().to_msg()


if __name__ == '__main__':
    unittest.main(verbosity=2)
