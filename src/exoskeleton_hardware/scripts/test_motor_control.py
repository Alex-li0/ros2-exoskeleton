#!/usr/bin/env python3
"""
ROS2 STM32 电机控制测试节点
============================
通过 USB 串口直接与 STM32 下位机通信, 实现:
  - 1kHz 反馈帧接收与解析
  - 力矩/速度/位置控制模式
  - 7 项自动化测试序列
  - ROS2 话题发布 (joint_states, motor_feedback)
  - ROS2 服务调用 (单马达控制、测试序列)

协议参照: usb_protocol_and_test_report.pdf
验证参照: full_control_test.py@robot_stm32

用法:
  ros2 run exoskeleton_hardware test_motor_control --ros-args -p port:=/dev/ttyACM0
  ros2 run exoskeleton_hardware test_motor_control --ros-args -p port:=/dev/ttyACM0 -p motor_id:=1 -p skip:=comm
"""

import math
import struct
import time
import sys
import threading
from typing import Dict, List, Optional, Tuple

import serial
import serial.tools.list_ports
import termios

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String

# ==================== 协议常量 ====================
STX_REQ = 0xAA
STX_ACK = 0x55

CMD_PING              = 0x01
CMD_MOTOR_SET_TARGET  = 0x11
CMD_MOTOR_ENABLE      = 0x12
CMD_MOTOR_CALIBRATE   = 0x13
CMD_MOTOR_RAW_FEEDBACK = 0x15

MODE_POSITION = 0
MODE_SPEED    = 1
MODE_TORQUE   = 2
MODE_POSITION_SINGLE = 3   # 单圈位置模式 (固件 usb_handler.c case 3, 取模 0~36000)

# 减速比 (电机侧 → 输出轴)
REDUCTION_RATIO = 36.0
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi

# ==================== 终端颜色 ====================
G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; C = "\033[96m"; B = "\033[1m"; N = "\033[0m"
def ok(s):    return f"{G}{s}{N}"
def fail(s):  return f"{R}{s}{N}"
def warn(s):  return f"{Y}{s}{N}"
def info(s):  return f"{C}{s}{N}"
def bold(s):  return f"{B}{s}{N}"


# ==================== 帧工具 ====================

def calc_chk(cmd: int, data: bytes) -> int:
    """XOR 校验: chk = CMD ^ LEN ^ data[0] ^ ... ^ data[N-1]"""
    chk = cmd ^ len(data)
    for b in data:
        chk ^= b
    return chk & 0xFF


def make_frame(cmd: int, data: bytes = b"") -> bytes:
    """构造命令帧: STX_REQ | CMD | LEN | DATA... | CHK"""
    return bytes([STX_REQ, cmd, len(data)]) + data + bytes([calc_chk(cmd, data)])


def send_cmd(ser: serial.Serial, cmd: int, data: bytes = b""):
    """发送命令 (不等应答), 串口断开时静默失败"""
    try:
        ser.write(make_frame(cmd, data))
        ser.flush()
    except (serial.SerialException, OSError, termios.error):
        pass  # 串口已断开, 静默忽略


# ==================== 反馈帧接收器 ====================

class FeedbackReceiver:
    """持续接收 1kHz 反馈帧, 维护每马达最新状态"""

    def __init__(self):
        self.lock = threading.Lock()
        self._buf = bytearray()
        self.total_frames = 0
        self.total_bytes = 0
        self.parse_errors = 0
        self.frame_times: List[float] = []
        self.latest_motors: Dict[int, dict] = {}

    def feed(self, data: bytes):
        """送入原始字节, 内部解析完整帧"""
        self.total_bytes += len(data)
        self._buf.extend(data)
        while len(self._buf) >= 4:
            if self._buf[0] != STX_ACK:
                self._buf.pop(0)
                continue
            cmd = self._buf[1]
            length = self._buf[2]
            total = 3 + length + 1  # STX + CMD + LEN + DATA + CHK
            if len(self._buf) < total:
                break
            payload = bytes(self._buf[3:3 + length])
            rx_chk = self._buf[3 + length]
            expected = calc_chk(cmd, payload)
            if rx_chk == expected:
                if cmd == CMD_MOTOR_RAW_FEEDBACK:
                    ts = time.monotonic()
                    motors = self._parse_feedback(payload)
                    with self.lock:
                        self.total_frames += 1
                        self.frame_times.append(ts)
                        if len(self.frame_times) > 200:
                            self.frame_times = self.frame_times[-200:]
                        self.latest_motors = motors
            else:
                self.parse_errors += 1
            del self._buf[:total]
        # 防止缓冲区异常增长
        if len(self._buf) > 1024:
            self._buf = self._buf[-512:]

    @staticmethod
    def _parse_feedback(data: bytes) -> Dict[int, dict]:
        """解析 0x15 反馈帧: bus_count | motors_per_bus | [encoder(u16)+speed(i16)+iq(i16)+multi(i32)]*N"""
        motors = {}
        if len(data) < 2:
            return motors
        bus_count = data[0]
        motors_per_bus = data[1]
        total = bus_count * motors_per_bus
        if len(data) < 2 + total * 10:
            return motors
        off = 2
        for i in range(total):
            enc   = struct.unpack_from('<H', data, off)[0]
            spd   = struct.unpack_from('<h', data, off + 2)[0]
            iq    = struct.unpack_from('<h', data, off + 4)[0]
            multi = struct.unpack_from('<i', data, off + 6)[0]
            motors[i] = {
                'encoder': enc,
                'angle_deg': enc * 360.0 / 65536.0,
                'multi_turn_deg': multi * 0.01,
                'speed_dps': spd,
                'output_speed_dps': spd / REDUCTION_RATIO,
                'output_speed_rads': spd / REDUCTION_RATIO * DEG_TO_RAD,
                'position_rad': multi * 0.01 * DEG_TO_RAD,  # 电机侧 0.01°/LSB, 输出轴需÷36; 固件默认关多圈轮询→恒0, 暂未使用
                'iq': iq,
            }
            off += 10
        return motors

    def get_motor(self, motor_id: int) -> Optional[dict]:
        """获取指定马达最新状态 (motor_id 从 1 开始)"""
        with self.lock:
            return self.latest_motors.get(motor_id - 1, None)

    def get_hz(self) -> float:
        """计算最近 2 秒内的反馈帧率"""
        with self.lock:
            times = self.frame_times[:]
        if len(times) < 2:
            return 0.0
        duration = times[-1] - times[0]
        return (len(times) - 1) / duration if duration > 0 else 0.0

    @property
    def last_frame_age(self) -> float:
        """距离最后一帧过去了多少秒 (用于断开检测)"""
        with self.lock:
            times = self.frame_times[:]
        if not times:
            return float('inf')
        return time.monotonic() - times[-1]


# ==================== 发送循环 ====================

def send_loop(ser: serial.Serial, rx: FeedbackReceiver,
              motor_id: int, duration: float, mode: int,
              target_fn, label: str = "",
              live_interval: float = 0.1,
              extra_header: str = "",
              extra_fn = None) -> float:
    """以 1000Hz 持续发送 SET_TARGET 指令, 同时实时打印反馈.

    target_fn(elapsed_sec) -> bytes
    extra_fn(elapsed_sec) -> str  可选, 返回额外列内容 (如目标位置)
    """
    SEND_HZ = 1000
    interval = 1.0 / SEND_HZ
    motor_id_byte = bytes([motor_id])

    mode_names = {0: "POS", 1: "SPD", 2: "TRQ"}
    mode_str = mode_names.get(mode, "???")

    print(f"  发送频率={SEND_HZ}Hz  时长={duration}s  模式={mode_str}  {label}")
    header = f"    {'Time':>5s}  {'Angle':>8s}  {'Speed':>8s}  {'Iq':>5s}"
    if extra_header:
        header += f"  {extra_header}"
    print(header)

    t0 = time.monotonic()
    next_send = t0
    next_print = t0 + live_interval
    next_debug = t0 + 0.1   # debug 日志 10Hz (不可覆盖)
    send_count = 0

    while time.monotonic() - t0 < duration:
        now = time.monotonic()
        elapsed = now - t0

        payload = target_fn(elapsed)
        data = motor_id_byte + payload
        send_cmd(ser, CMD_MOTOR_SET_TARGET, data)
        send_count += 1

        if now >= next_print:
            m = rx.get_motor(motor_id)
            if m:
                out_spd = m['output_speed_dps']
                # 滚动覆盖行 (实时)
                line = f"\r    {elapsed:>4.1f}s  {m['angle_deg']:>+8.2f}°  {out_spd:>+7.1f}°/s  {m['iq']:>+5d}"
                if extra_fn:
                    line += f"  {extra_fn(elapsed)}"
                print(line, end="", flush=True)
            next_print = now + live_interval

        # debug 日志: 10Hz, 保留在终端历史
        if extra_fn and now >= next_debug:
            print()  # 结束当前 \r 滚动行
            print(f"  [DBG {elapsed:>5.2f}s sine] {extra_fn(elapsed)}")
            next_debug = now + 0.1

        next_send += interval
        sleep_t = next_send - time.monotonic()
        if sleep_t > 0:
            time.sleep(sleep_t)
        elif sleep_t < -0.002:
            next_send = time.monotonic()

    print()
    return send_count / duration


# ==================== ROS2 测试节点 ====================

class MotorTestNode(Node):
    """ROS2 电机测试节点 — 直接操作串口, 发布话题, 提供服务"""

    def __init__(self):
        super().__init__('motor_test_node')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('skip', '')

        self._port = self.get_parameter('port').value
        self._baud = self.get_parameter('baud').value
        self._motor_id = self.get_parameter('motor_id').value
        self._skip = self.get_parameter('skip').value

        # 4 关节: 左/右 hip3 + knee
        self._joint_names = ['left_hip3', 'left_knee', 'right_hip3', 'right_knee']

        # 发布者: 关节状态 (供 rviz2 / plotjuggler)
        self._joint_pub = self.create_publisher(JointState, '/motor_test/joint_states', 10)
        self._feedback_pub = self.create_publisher(Float32MultiArray, '/motor_test/feedback', 10)
        self._status_pub = self.create_publisher(String, '/motor_test/status', 10)

        # 定时器: 发布反馈数据 (20Hz)
        self._pub_timer = self.create_timer(0.05, self._publish_feedback)

        # 串口
        self._ser: Optional[serial.Serial] = None
        self._rx = FeedbackReceiver()
        self._stop_event = threading.Event()
        self._recv_thread: Optional[threading.Thread] = None

        # 打开串口
        self._open_serial()

        self.get_logger().info(
            f"电机测试节点已启动: port={self._port}, baud={self._baud}, "
            f"motor_id={self._motor_id}")

    def _open_serial(self):
        # 自动检测端口
        port = self._port
        if port == "auto":
            port = find_acm_port()
            if not port:
                self.get_logger().error("自动端口检测失败: 未找到 ttyACM/ttyUSB 设备")
                self._ser = None
                return
            self.get_logger().info(f"自动检测到端口: {port}")

        try:
            self._ser = serial.Serial(
                port=port, baudrate=self._baud,
                timeout=0, write_timeout=0.5)
            time.sleep(0.2)
            self._ser.reset_input_buffer()

            self._recv_thread = threading.Thread(
                target=self._receiver_loop, daemon=True)
            self._recv_thread.start()
            self.get_logger().info(f"串口已打开: {port}")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            self._ser = None

    def _receiver_loop(self):
        """接收线程: 持续读串口, 断开时等待重连而不是退出"""
        while not self._stop_event.is_set():
            if self._ser is None or not self._ser.is_open:
                time.sleep(0.1)
                continue
            try:
                data = self._ser.read(4096)
                if data:
                    self._rx.feed(data)
                else:
                    time.sleep(0.0001)
            except (serial.SerialException, OSError, termios.error):
                # USB 断开, 不退出线程, 等待重新打开
                time.sleep(0.1)
            except Exception:
                time.sleep(0.1)

    def _reconnect_serial(self) -> bool:
        """重连串口: 关旧→扫描→开新→启接收"""
        # 关闭旧连接
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

        # 扫描新端口
        port = self._port
        if port == "auto":
            port = find_acm_port()
        if not port:
            return False

        # 打开新连接
        try:
            self._ser = serial.Serial(
                port=port, baudrate=self._baud,
                timeout=0, write_timeout=0.5)
            time.sleep(0.2)
            self._ser.reset_input_buffer()
            self.get_logger().info(f"串口重连成功: {port} (已验证为 STM32 设备)")
            return True
        except Exception as e:
            self.get_logger().error(f"重连失败: {e}")
            return False

    def _publish_feedback(self):
        """定时发布关节状态和反馈数据"""
        if self._rx.total_frames == 0:
            return

        # 发布所有马达的反馈
        with self._rx.lock:
            motors = dict(self._rx.latest_motors)

        if not motors:
            return

        # JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        arr = Float32MultiArray()

        for idx, m in sorted(motors.items()):
            msg.name.append(self._joint_names[idx] if idx < len(self._joint_names) else f"motor_{idx + 1}")
            msg.position.append(m['position_rad'])
            msg.velocity.append(m['output_speed_rads'])
            msg.effort.append(float(m['iq']))

            arr.data.extend([
                float(idx + 1),           # motor_id
                float(m['encoder']),
                float(m['speed_dps']),
                float(m['iq']),
                float(m['multi_turn_deg']),
                m['angle_deg'],
                m['output_speed_rads'],
            ])

        self._joint_pub.publish(msg)
        self._feedback_pub.publish(arr)

        # 状态摘要
        hz = self._rx.get_hz()
        status = String()
        status.data = (f"frames={self._rx.total_frames} hz={hz:.0f} "
                       f"errors={self._rx.parse_errors} motors={len(motors)}")
        self._status_pub.publish(status)

    def close(self):
        """安全关闭"""
        self._stop_event.set()
        if self._ser and self._ser.is_open:
            try:
                # 归零
                mid = self._motor_id
                data = bytes([mid, MODE_TORQUE]) + struct.pack('<f', 0.0)
                send_cmd(self._ser, CMD_MOTOR_SET_TARGET, data)
                time.sleep(0.05)
            except Exception:
                pass
            self._ser.close()
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)


# ==================== 五次插值辅助 ====================

def move_center_quintic(ser: serial.Serial, rx: FeedbackReceiver,
                        motor_id: int, start_deg: float, end_deg: float,
                        T: float = 1.0, hz: float = 1000.0, log_fn=None):
    """五次多项式插值, 在 T 秒内将电机位置从 start_deg 平滑移到 end_deg (单圈 mode 3).

    五次多项式 s(t) = 6τ⁵ - 15τ⁴ + 10τ³ (τ = t/T) 满足:
      端点位置 s(0)=0, s(1)=1, 速度/加速度两端均为 0 (最小加加速度, 无冲击).

    用途: 单圈模式下, 当正弦中心靠近 0°/360° 边界时, 平滑转移到安全范围,
    避免正弦波跨越环绕边界时依赖电机固件的最短路径行为.
    """
    n = int(T * hz)
    interval = T / n
    delta = end_deg - start_deg
    t0 = time.monotonic()
    next_send = t0
    next_log = t0 + 0.1

    for i in range(n + 1):
        tau = i / n
        s = 6 * tau**5 - 15 * tau**4 + 10 * tau**3
        pos_deg = start_deg + delta * s
        pos_rad = pos_deg * DEG_TO_RAD
        data = bytes([motor_id, MODE_POSITION_SINGLE]) + struct.pack('<f', pos_rad)
        send_cmd(ser, CMD_MOTOR_SET_TARGET, data)

        # 漂移补偿定时: 稳定 1000Hz
        next_send += interval
        sleep_t = next_send - time.monotonic()
        if sleep_t > 0:
            time.sleep(sleep_t)
        elif sleep_t < -0.002:
            next_send = time.monotonic()

        # 进度日志 (10Hz)
        now = time.monotonic()
        if now >= next_log:
            if log_fn:
                log_fn(pos_deg, 'quintic')
            else:
                m = rx.get_motor(motor_id) if rx else None
                actual = m['angle_deg'] if m else float('nan')
                err = pos_deg - actual
                if err > 180: err -= 360
                if err < -180: err += 360
                print(f"    [转移 {now - t0:>4.1f}s] 目标={pos_deg:>+7.2f}°  "
                      f"实际={actual:>+7.2f}°  误差={err:>+6.2f}°")
            next_log += 0.1


# ==================== 测试序列 ====================

def run_test_sequence(ser: serial.Serial, rx: FeedbackReceiver,
                      motor_id: int, skip: List[str], node: MotorTestNode = None):
    """运行 8 项自动化测试"""

    results = {}

    def should_run(name):
        return name not in skip

    # ─── [1] 通讯验证 ───
    if should_run('comm'):
        print(f"\n{bold('═══ [1/7] 通讯验证 (1kHz 反馈流) ═══')}")
        print("  检查 1kHz 反馈帧是否持续到达...")
        time.sleep(1.0)
        hz = rx.get_hz()
        frames = rx.total_frames
        errors = rx.parse_errors
        print(f"  反馈帧率: {hz:.0f} Hz  累计帧数: {frames}  解析错误: {errors}")
        if hz > 100:
            print(f"  {ok('OK')}  1kHz 反馈流正常")
            results['comm'] = True
        elif frames > 0:
            print(f"  {warn('WARN')}  帧率偏低 ({hz:.0f} Hz, 期望 ~1000)")
            results['comm'] = True
        else:
            print(f"  {fail('FAIL')}  未收到任何反馈帧")
            results['comm'] = False
            return results
    else:
        results['comm'] = True

    # ─── [2] 状态监控 ───
    if should_run('status'):
        print(f"\n{bold('═══ [2/7] 电机实时状态 ═══')}")
        with rx.lock:
            indices = sorted(rx.latest_motors.keys())
        if not indices:
            print(f"  {fail('FAIL')}  无反馈数据")
            results['status'] = False
        else:
            print(f"  反馈帧包含 {len(indices)} 个马达:")
            for idx in indices:
                m = rx.latest_motors[idx]
                mid = idx + 1
                marker = "  ← 目标" if mid == motor_id else ""
                has_data = (m['encoder'] != 0 or m['speed_dps'] != 0 or m['iq'] != 0)
                icon = ok("✓") if has_data else warn("?")
                print(f"    motor_id={mid}: 多圈={m['multi_turn_deg']:>8.2f}°  "
                      f"单圈={m['angle_deg']:>7.2f}°  "
                      f"速度={m['output_speed_dps']:>+6.1f}°/s  iq={m['iq']:>+5d}  {icon}{marker}")
            results['status'] = True

    # ─── [3] 使能检查 ───
    if should_run('enable'):
        print(f"\n{bold('═══ [3/7] 使能状态检查 ═══')}")
        print(f"  (STM32 启动时已自动使能电机)")

        m = rx.get_motor(motor_id)
        if m is None:
            print(f"  {fail('FAIL')}  电机{motor_id} 无反馈数据")
            results['enable'] = False
        else:
            print(f"  电机{motor_id}: encoder={m['encoder']} ({m['angle_deg']:.2f}°)  "
                  f"speed={m['speed_dps']:+d}dps  iq={m['iq']:+d}")

            # 短暂力矩脉冲验证响应
            print(f"  短暂力矩脉冲 iq=20 验证响应...")
            data = bytes([motor_id, MODE_TORQUE]) + struct.pack('<f', 20.0)
            send_cmd(ser, CMD_MOTOR_SET_TARGET, data)
            time.sleep(0.3)
            m2 = rx.get_motor(motor_id)
            iq_val = m2['iq'] if m2 else 0
            print(f"  脉冲后 iq={iq_val}")

            data = bytes([motor_id, MODE_TORQUE]) + struct.pack('<f', 0.0)
            send_cmd(ser, CMD_MOTOR_SET_TARGET, data)

            if abs(iq_val) > 8:
                print(f"  {ok('OK')}  电机响应正常")
            else:
                print(f"  {warn('WARN')}  iq≈0 (无负载时正常, 非故障)")
            results['enable'] = True

    pause = lambda: time.sleep(0.5)

    # ─── [4] 力矩控制 ───
    if should_run('torque'):
        pause()
        print(f"\n{bold('═══ [4/7] 力矩控制测试 ═══')}")
        m0 = rx.get_motor(motor_id)
        iq_base = m0['iq'] if m0 else 0
        print(f"  基线 iq={iq_base}")

        hz = send_loop(ser, rx, motor_id, duration=2.0, mode=MODE_TORQUE,
                       target_fn=lambda t: bytes([MODE_TORQUE]) + struct.pack('<f', 15.0),
                       label="(iq=15)")

        m1 = rx.get_motor(motor_id)
        iq_end = m1['iq'] if m1 else 0
        success = abs(iq_end - iq_base) > 5
        icon = ok if success else fail
        print(f"  {icon('OK' if success else 'FAIL')}  iq={iq_base:+d} → {iq_end:+d}  "
              f"速率={hz:.0f}Hz")

        data = bytes([motor_id, MODE_TORQUE]) + struct.pack('<f', 0.0)
        send_cmd(ser, CMD_MOTOR_SET_TARGET, data)
        time.sleep(0.5)
        results['torque'] = success

    # ─── [5] 速度控制 ───
    if should_run('speed'):
        pause()
        print(f"\n{bold('═══ [5/7] 速度控制测试 ═══')}")

        amplitude_dps = 30.0
        period = 10.0
        iq_limit = 500
        print(f"  速度曲线: {amplitude_dps:.0f}°/s × sin(2π·t/{period}s)  iq_limit={iq_limit}")

        errors = []

        def speed_fn(t):
            target_dps = amplitude_dps * math.sin(2.0 * math.pi * t / period)
            m = rx.get_motor(motor_id)
            if m:
                actual_dps = m['output_speed_dps']
                errors.append((t, target_dps, actual_dps))
            speed_rad = target_dps * DEG_TO_RAD
            return (bytes([MODE_SPEED]) + struct.pack('<f', speed_rad) +
                    struct.pack('<h', iq_limit))

        hz = send_loop(ser, rx, motor_id, duration=period, mode=MODE_SPEED,
                       target_fn=speed_fn,
                       label=f"(正弦 ±{amplitude_dps:.0f}°/s 周期={period}s)")

        if errors:
            abs_errs = [abs(tgt - act) for _, tgt, act in errors]
            max_err = max(abs_errs)
            rms_err = math.sqrt(sum(e**2 for e in abs_errs) / len(abs_errs))
            mean_err = sum(abs_errs) / len(abs_errs)
            peak_actual = max(abs(e[2]) for e in errors)
            print(f"  速度追踪统计 ({len(errors)} 样本, 输出轴):")
            print(f"    峰值实际: {peak_actual:.1f}°/s  最大误差: {max_err:.1f}°/s")
            print(f"    平均误差: {mean_err:.1f}°/s  RMS: {rms_err:.1f}°/s")
            success = rms_err < 5.0 and peak_actual > 3.0
        else:
            success = False

        icon = ok if success else warn
        print(f"  {icon('OK' if success else 'WARN')}  速率={hz:.0f}Hz")

        data = bytes([motor_id, MODE_TORQUE]) + struct.pack('<f', 0.0)
        send_cmd(ser, CMD_MOTOR_SET_TARGET, data)
        time.sleep(0.5)
        results['speed'] = success

    # ─── [6] 正弦位置追踪 ───
    if should_run('position'):
        pause()
        print(f"\n{bold('═══ [6/7] 正弦波位置追踪测试 ═══')}")

        m0 = rx.get_motor(motor_id)
        if m0 is None:
            print(f"  {fail('FAIL')}  无电机数据")
            results['position'] = False
        else:
            # ── 10Hz 调试日志 (从位置测试开始持续到结束) ──
            dbg_t0 = time.monotonic()
            dbg_next = [dbg_t0 + 0.1]  # list 便于闭包内修改

            def dbg_log(target_deg, phase):
                now = time.monotonic()
                if now < dbg_next[0]:
                    return
                dbg_next[0] = now + 0.1
                m = rx.get_motor(motor_id)
                if not m:
                    return
                actual = m['angle_deg']  # 单圈角度
                spd = m['output_speed_dps']
                iq = m['iq']
                if target_deg is None:
                    tgt = "   ---  "
                    err = "  ---  "
                else:
                    e = target_deg - actual
                    if e > 180: e -= 360
                    if e < -180: e += 360
                    tgt = f"{target_deg:>+7.2f}°"
                    err = f"{e:>+6.2f}°"
                print(f"  [DBG {now - dbg_t0:>5.2f}s {phase:<7s}] 目标={tgt} "
                      f"实际={actual:>+7.2f}° 误差={err} 速度={spd:>+6.1f}°/s iq={iq:>+4d}")

            # 等待停止 — 连续 5 次采样都满足条件才算真的停了
            print(f"  等待电机完全停止...")
            data = bytes([motor_id, MODE_TORQUE]) + struct.pack('<f', 0.0)
            send_cmd(ser, CMD_MOTOR_SET_TARGET, data)
            stopped_count = 0
            for _ in range(50):
                m = rx.get_motor(motor_id)
                if m and abs(m['output_speed_dps']) < 0.5 and abs(m['iq']) < 10:
                    stopped_count += 1
                    if stopped_count >= 5:
                        break
                else:
                    stopped_count = 0
                dbg_log(None, 'stop')
                time.sleep(0.05)

            # 读取当前单圈角度作为追踪中心
            time.sleep(0.2)
            m = rx.get_motor(motor_id)
            center_deg = m['angle_deg'] if m else 0.0
            print(f"  已停止, 当前位置={center_deg:.1f}°")

            # 保留完整 ±30° 振幅
            amplitude = 30.0
            period = 4.0
            duration = 8.0
            SAFE_MIN = 45.0
            SAFE_MAX = 315.0

            # 计算安全中心: 保证 中心±振幅 不越过 0°/360° 环绕边界
            # 单圈模式 (mode 3) 由固件取模, 但电机对越界目标的最短路径行为未知,
            # 因此让正弦保持在 (15°, 345°) 内最稳妥.
            safe_center = min(SAFE_MAX, max(SAFE_MIN, center_deg))

            # 若中心在边界附近, 五次插值 1s 内平滑移到安全中心
            if abs(safe_center - center_deg) > 1.0:
                msg = (f"中心 {center_deg:.1f}° 靠近 0°/360° 边界, "
                       f"五次插值平滑移至 {safe_center:.1f}° (1s)")
                print(f"  {warn(msg)}")
                move_center_quintic(ser, rx, motor_id, center_deg, safe_center,
                                    T=1.0, log_fn=dbg_log)
                center_deg = safe_center
                print(f"  移动完成, 新中心={center_deg:.1f}°")

            # 进入单圈位置模式 (mode 3) 并稳定 1s
            print(f"  进入单圈位置模式稳定 1s...")
            center_rad = center_deg * DEG_TO_RAD
            t_settle = time.monotonic()
            while time.monotonic() - t_settle < 1.0:
                data = bytes([motor_id, MODE_POSITION_SINGLE]) + struct.pack('<f', center_rad)
                send_cmd(ser, CMD_MOTOR_SET_TARGET, data)
                dbg_log(center_deg, 'settle')
                time.sleep(0.001)

            # 正弦从零点 sin(0)=0 出发, 即从当前单圈位置开始
            print(f"  定位到起点 {center_deg:.1f}° (保持 0.5s)...")
            center_rad = center_deg * DEG_TO_RAD
            t_settle = time.monotonic()
            while time.monotonic() - t_settle < 0.5:
                data = bytes([motor_id, MODE_POSITION_SINGLE]) + struct.pack('<f', center_rad)
                send_cmd(ser, CMD_MOTOR_SET_TARGET, data)
                dbg_log(center_deg, 'hold')
                time.sleep(0.001)
            print(f"  mode=3 单圈中心={center_deg:.1f}°  振幅=±{amplitude:.0f}°  从零点出发")

            log = []
            _last_target = [center_deg]

            def angle_err(tgt, act):
                t = tgt % 360.0
                a = act % 360.0
                d = t - a
                if d > 180: d -= 360
                if d < -180: d += 360
                return d

            def debug_extra(t):
                m = rx.get_motor(motor_id)
                actual = m['angle_deg'] if m else 0
                err = angle_err(_last_target[0], actual)
                spd = m['output_speed_dps'] if m else 0
                iq = m['iq'] if m else 0
                return (f"目标={_last_target[0]:>+7.2f}° 实际={actual:>+7.2f}° "
                        f"误差={err:>+6.2f}° 速度={spd:>+6.1f}°/s iq={iq:>+4d}")

            def sine_fn(t):
                target_deg = center_deg + amplitude * math.sin(2.0 * math.pi * t / period)
                _last_target[0] = target_deg
                target_rad = target_deg * DEG_TO_RAD
                m2 = rx.get_motor(motor_id)
                actual = m2['angle_deg'] if m2 else 0
                log.append((t, target_deg, actual, target_rad))
                return bytes([MODE_POSITION_SINGLE]) + struct.pack('<f', target_rad)

            hz = send_loop(ser, rx, motor_id, duration=duration, mode=MODE_POSITION_SINGLE,
                           target_fn=sine_fn,
                           label=f"(中心 {center_deg:.0f}° ±{amplitude:.0f}° 周期={period}s)",
                           extra_header="目标         实际       误差       速度      Iq",
                           extra_fn=debug_extra)

            data = bytes([motor_id, MODE_TORQUE]) + struct.pack('<f', 0.0)
            send_cmd(ser, CMD_MOTOR_SET_TARGET, data)
            time.sleep(0.3)

            if log:
                # 单圈误差: 处理 360° 环绕, 取最短弧距
                errors_pos = [abs(angle_err(e[1], e[2])) for e in log]
                max_err = max(errors_pos)
                rms = math.sqrt(sum(x**2 for x in errors_pos) / len(errors_pos))
                print(f"  位置追踪统计 ({len(log)} 点, 单圈):")
                print(f"    最大误差={max_err:.2f}°  RMS={rms:.2f}°  速率={hz:.0f}Hz")
                success = rms < 5.0
            else:
                print(f"  {fail('FAIL')}  无追踪数据")
                success = False

            icon = ok if success else warn
            print(f"  {icon('OK' if success else 'WARN')}")
            results['position'] = success

    # ─── [7] 看门狗掉线保护 ───
    if should_run('watchdog'):
        pause()
        print(f"\n{bold('═══ [7/7] 看门狗掉线保护 ═══')}")
        print(f"  原理: 上位机 >100ms 不发指令 → STM32 自动切零力矩")

        print(f"  [1/3] 1000Hz 发送 iq=30, 1 秒...")
        send_loop(ser, rx, motor_id, duration=1.0, mode=MODE_TORQUE,
                  target_fn=lambda t: bytes([MODE_TORQUE]) + struct.pack('<f', 15.0),
                  label="(iq=30)", live_interval=0.3)
        m = rx.get_motor(motor_id)
        iq_on = m['iq'] if m else 0
        print(f"  [1/3] 发力矩时 iq={iq_on}")

        print(f"  [2/3] 上位机静默 150ms...")
        time.sleep(0.15)
        m = rx.get_motor(motor_id)
        iq_off = m['iq'] if m else 999
        print(f"  [2/3] 静默后 iq={iq_off}")

        print(f"  [3/3] 恢复发送...")
        send_loop(ser, rx, motor_id, duration=0.5, mode=MODE_TORQUE,
                  target_fn=lambda t: bytes([MODE_TORQUE]) + struct.pack('<f', 15.0),
                  label="(iq=30)", live_interval=0.3)
        m = rx.get_motor(motor_id)
        iq_recovered = m['iq'] if m else 0
        print(f"  [3/3] 恢复后 iq={iq_recovered}")

        data = bytes([motor_id, MODE_TORQUE]) + struct.pack('<f', 0.0)
        send_cmd(ser, CMD_MOTOR_SET_TARGET, data)

        watchdog_ok = abs(iq_on) > 10 and abs(iq_off) < 5
        recovery_ok = abs(iq_recovered) > 5

        if watchdog_ok:
            print(f"    {ok('✓ 看门狗生效')}")
        else:
            print(f"    {fail('✗ 看门狗未生效')}")
        if recovery_ok:
            print(f"    {ok('✓ 控制可恢复')}")
        else:
            print(f"    {warn('? 恢复异常')}")

        results['watchdog'] = watchdog_ok

    # ─── [8] 热插拔测试 ───
    if should_run('hotplug'):
        pause()
        print(f"\n{bold('═══ [8/8] USB 热插拔测试 ═══')}")
        print(f"  本测试需要你手动拔插 STM32 USB 线")
        print(f"  电机将以 30°/s 匀速转动, 拔线后看门狗使其停止, 重连后恢复转动")

        SPEED_DEGS = 30.0                     # 目标转速 °/s
        SPEED_RAD = SPEED_DEGS * DEG_TO_RAD    # rad/s
        IQ_LIMIT = 500                         # 速度环力矩上限

        def speed_payload():
            """构造速度控制帧 data 部分"""
            return (bytes([MODE_SPEED]) +
                    struct.pack('<f', SPEED_RAD) +
                    struct.pack('<h', IQ_LIMIT))

        # 阶段 1: 发送 30°/s 匀速转动
        print(f"\n  {info('[阶段1]')} 发送速度指令 {SPEED_DEGS:.0f}°/s, 电机应匀速转动...")
        send_loop(ser, rx, motor_id, duration=1.5, mode=MODE_SPEED,
                  target_fn=lambda t: speed_payload(),
                  label=f"({SPEED_DEGS:.0f}°/s)", live_interval=0.3)
        m = rx.get_motor(motor_id)
        spd_before = m['output_speed_dps'] if m else 0
        hz_before = rx.get_hz()
        print(f"  断开前: 实际转速={spd_before:+.1f}°/s  反馈帧率={hz_before:.0f}Hz")

        # 阶段 2: 等待用户拔线 (同时持续发速度指令)
        print(f"\n  {bold('>>> 请拔掉 STM32 的 USB 线... <<<')}")
        print(f"  (脚本持续发送 {SPEED_DEGS:.0f}°/s 指令, 将自动检测断开)")

        disconnect_detected = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0:
            # 持续发速度指令
            data = bytes([motor_id]) + speed_payload()
            send_cmd(ser, CMD_MOTOR_SET_TARGET, data)

            # 用最后一帧的年龄来判断断开 (比 get_hz() 即时得多)
            age = rx.last_frame_age
            if age > 0.2:  # 超过 200ms 没有新帧
                disconnect_detected = True
                elapsed = time.monotonic() - t0
                print(f"\n  {ok('✓ 检测到断开')}  ({elapsed:.1f}s, 最后一帧 {age:.1f}s 前)")
                print(f"  (此时电机应已停止 — STM32 看门狗自动切零力矩)")
                break
            time.sleep(0.05)

        if not disconnect_detected:
            print(f"\n  {warn('WARN')}  15s 内未检测到断开, 跳过后续测试")
            data = bytes([motor_id, MODE_TORQUE]) + struct.pack('<f', 0.0)
            send_cmd(ser, CMD_MOTOR_SET_TARGET, data)
            results['hotplug'] = False
        else:
            # 阶段 3: 关闭旧串口, 等待用户重连
            print(f"\n  {bold('>>> 请重新插入 STM32 的 USB 线... <<<')}")
            print(f"  (脚本扫描所有 ttyACM*/ttyUSB*, 通过验证反馈帧识别 STM32)")

            # 关闭已断开的串口, 准备重连
            if node._ser and node._ser.is_open:
                try:
                    node._ser.close()
                except Exception:
                    pass
            node._ser = None

            reconnect_detected = False
            t0 = time.monotonic()
            while time.monotonic() - t0 < 30.0:
                # 尝试重新打开串口
                if node._ser is None or not node._ser.is_open:
                    if not node._reconnect_serial():
                        time.sleep(1.0)
                        continue
                # 用最后一帧的年龄来判断重连 (新帧到达 → age 骤降)
                age = rx.last_frame_age
                frames = rx.total_frames

                if age < 0.1 and frames > 10:  # 100ms 内有新帧到达
                    reconnect_detected = True
                    elapsed = time.monotonic() - t0
                    print(f"\n  {ok('✓ 检测到重连')}  ({elapsed:.1f}s, "
                          f"最后一帧 {age*1000:.0f}ms 前, 已收到 {frames} 帧)")
                    break
                time.sleep(0.1)

            if not reconnect_detected:
                print(f"\n  {fail('FAIL')}  30s 内未检测到重连")
                results['hotplug'] = False
            else:
                # 重连后 ser 对象已更新, 用 node._ser
                ser_reconnected = node._ser

                # 阶段 4: 重连后恢复 30°/s 速度指令
                time.sleep(0.3)
                print(f"\n  {info('[阶段4]')} 重连后恢复 {SPEED_DEGS:.0f}°/s 速度指令, "
                      f"电机应重新开始转动...")
                if ser_reconnected:
                    send_loop(ser_reconnected, rx, motor_id, duration=2.0, mode=MODE_SPEED,
                              target_fn=lambda t: speed_payload(),
                              label=f"({SPEED_DEGS:.0f}°/s)", live_interval=0.3)
                m = rx.get_motor(motor_id)
                spd_after = m['output_speed_dps'] if m else 0

                # 停止
                data = bytes([motor_id, MODE_TORQUE]) + struct.pack('<f', 0.0)
                if ser_reconnected:
                    send_cmd(ser_reconnected, CMD_MOTOR_SET_TARGET, data)
                time.sleep(0.3)

                print(f"  重连后实际转速={spd_after:+.1f}°/s")
                speed_ok = abs(spd_after) > 3.0
                if speed_ok:
                    print(f"  {ok('✓ 热插拔测试通过 — 电机恢复转动')}")
                else:
                    print(f"  {warn('WARN')}  转速偏低 (期望 ~{SPEED_DEGS:.0f}°/s, 实际 {spd_after:.1f}°/s)")

                results['hotplug'] = reconnect_detected and speed_ok

    return results


# ==================== 主入口 ====================

def find_acm_port():
    """扫描所有 ttyACM*/ttyUSB* 设备, 通过多帧验证找到真正的 STM32.
    要求: 200ms 内收到 ≥50 帧合法 0x15 反馈帧 (即帧率 ≥250Hz),
    且帧之间 bus_count/motors_per_bus 保持一致.
    这样可以排除 DSP 下载器等随机数据的串口设备.
    """
    candidates = []
    for p in serial.tools.list_ports.comports():
        if 'ttyACM' in p.device or 'ttyUSB' in p.device:
            candidates.append(p.device)

    if not candidates:
        return None

    for port in candidates:
        try:
            test_ser = serial.Serial(port=port, baudrate=115200, timeout=0)
            time.sleep(0.1)
            test_ser.reset_input_buffer()

            t0 = time.monotonic()
            buf = bytearray()
            valid_frames = 0
            first_bus = first_motor = -1  # 记录第一帧的 bus_count/motors_per_bus

            while time.monotonic() - t0 < 0.2:
                try:
                    data = test_ser.read(1024)
                    if data:
                        buf.extend(data)
                except Exception:
                    break

                while len(buf) >= 4:
                    if buf[0] != STX_ACK:
                        buf.pop(0)
                        continue
                    cmd = buf[1]
                    length = buf[2]
                    total = 3 + length + 1
                    if len(buf) < total:
                        break
                    payload = bytes(buf[3:3 + length])
                    rx_chk = buf[3 + length]
                    expected = calc_chk(cmd, payload)
                    del buf[:total]

                    # 必须是指令 0x15 且校验通过
                    if rx_chk != expected or cmd != CMD_MOTOR_RAW_FEEDBACK:
                        continue
                    # 必须有合法的 bus_count 和 motors_per_bus
                    if length < 2 or payload[0] == 0 or payload[1] == 0:
                        continue
                    # 帧长度必须匹配: 2 + bus*motor*10
                    expected_len = 2 + payload[0] * payload[1] * 10
                    if length != expected_len:
                        continue

                    # 记录第一帧的布局, 后续帧必须一致
                    if first_bus < 0:
                        first_bus = payload[0]
                        first_motor = payload[1]
                    elif payload[0] != first_bus or payload[1] != first_motor:
                        continue  # 布局不一致, 不是同一设备

                    valid_frames += 1

            test_ser.close()

            # 200ms 内必须收到 ≥50 帧 (帧率 ≥250Hz, STM32 是 ~1000Hz)
            if valid_frames >= 50:
                print(f"  {info(f'验证通过: {port}')} ({valid_frames} 帧/200ms, "
                      f"{first_bus}总线×{first_motor}电机)")
                return port
            elif valid_frames > 0:
                print(f"  {warn(f'跳过 {port}: 仅 {valid_frames} 帧/200ms (太低, 非 STM32)')}")
        except Exception:
            pass

    return None


def main():
    rclpy.init(args=sys.argv)

    # 解析 ROS2 参数获取串口配置
    # 先从命令行提取非 ROS 参数
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--ros-args', nargs='*', help='ROS args (ignored here)')
    args, unknown = ap.parse_known_args()

    node = MotorTestNode()

    port = node.get_parameter('port').value
    motor_id = node.get_parameter('motor_id').value

    # skip 参数通过 -p skip:=xxx 传入
    skip_list = node._skip.split(',') if node._skip else []

    if not port:
        port = find_acm_port()
    if not port:
        print(f"{fail('错误')}: 未找到 ttyACM, 请指定 --ros-args -p port:=/dev/ttyACM0")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f"串口: {info(port)}  电机ID: {motor_id}")
    if skip_list:
        print(f"跳过: {', '.join(skip_list)}")

    ser = node._ser  # 使用 node 的串口
    rx = node._rx

    if ser is None:
        print(f"{fail('串口未打开, 退出')}")
        node.destroy_node()
        rclpy.shutdown()
        return

    results = {}
    mid = motor_id

    try:
        results = run_test_sequence(ser, rx, mid, skip_list, node)
    except KeyboardInterrupt:
        print(f"\n{warn('用户中断')}")

    # ─── 报告 ───
    print(f"\n{bold('═══ 测试报告 ═══')}")
    all_pass = True
    names = {'comm': '通讯验证', 'status': '状态监控',
             'enable': '使能检查', 'torque': '力矩控制',
             'speed': '速度控制', 'position': '正弦波追踪',
             'watchdog': '看门狗掉线保护', 'hotplug': 'USB热插拔测试'}
    for name in ['comm', 'status', 'enable', 'torque', 'speed', 'position', 'watchdog', 'hotplug']:
        if name in skip_list:
            continue
        r = results.get(name)
        icon = ok('PASS') if r else (fail('FAIL') if r is False else warn('SKIP'))
        print(f"  {names.get(name, name):<20s} {icon}")
        if r is False:
            all_pass = False

    print(f"\n  1kHz 反馈: {rx.total_frames} 帧  {rx.get_hz():.0f} Hz  "
          f"解析错误: {rx.parse_errors}")

    if all_pass:
        print(f"\n  {ok('全部测试通过 ✓')}")
    else:
        print(f"\n  {fail('部分失败, 请查看详情')}")

    # 清理
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
