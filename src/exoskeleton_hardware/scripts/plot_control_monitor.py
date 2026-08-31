#!/usr/bin/env python3
# plot_control_monitor.py — 阻抗控制器 误差/延迟 实时监控与图表
#
# 依赖话题 (由 exo_impedance_controller 发布, 200Hz):
#   /impedance/debug   sensor_msgs/JointState  position=q_ref, velocity=q_measured, effort=tau
#   /impedance/timing  std_msgs/Float64MultiArray [loop_dt, ref_age_s]
#
# 用法:
#   python3 plot_control_monitor.py --duration 20          # 采集20s后出图并保存PNG
#   python3 plot_control_monitor.py --duration 20 --joint left_knee
#   python3 plot_control_monitor.py --duration 20 --motor 2   # 按电机号选关节 (见下)
#   python3 plot_control_monitor.py --duration 20 --live   # 实时滚动窗口 (边测边看)
#   python3 plot_control_monitor.py --duration 20 --csv out.csv  # 额外导出CSV
#
# 关节↔电机对应: 从 --motor-map-file 指定的 yaml (默认 exo_controllers.yaml 的
#   joint_motor_map 段) 读取, 与 URDF motor_map_file 一致。改配置文件里的
#   motor_id 数组, 画图跟随 (--motor N 选的就是对应新关节)。标题会标注 (motor X)。
#
# 提示: 先启动步态 (ros2 topic pub /cpg/state_cmd std_msgs/msg/String "data: start" --once)
#       再运行本脚本, 图表才有意义。

import argparse
import csv
import math
import os
import sys
import threading
import time

import yaml

import numpy as np

import matplotlib
matplotlib.use('TkAgg')  # 交互窗口
# 中文字体 (避免图表中文乱码)
matplotlib.rcParams['font.sans-serif'] = [
    'Noto Sans CJK SC', 'WenQuanYi Zen Hei', 'AR PL UMing CN', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

NAN = float('nan')

# 控制器 debug 发布器用 SensorDataQoS (best_effort), 订阅必须一致
BE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

# 默认关节↔电机映射文件 (与 URDF motor_map_file 一致)
DEFAULT_MOTOR_MAP_FILE = (
    '/home/lsy/vins_ws/src/exoskeleton_hardware/config/exo_controllers.yaml')


def load_motor_map(path):
    """从 yaml 读取 joint_motor_map, 返回 {motor_id: joint_name} (按关节名)。
    失败抛异常, 由调用方处理。"""
    with open(path) as f:
        cfg = yaml.safe_load(f)
    mm = cfg['joint_motor_map']['ros__parameters']
    joints = mm['joints']
    ids = mm['motor_id']
    return {int(mid): name for name, mid in zip(joints, ids)}


class ControlMonitor(Node):
    """订阅 debug/timing 话题, 记录控制误差与延迟数据"""

    def __init__(self, joint, duration):
        super().__init__('control_monitor')
        self.joint = joint
        self.duration = duration
        self.t0 = time.time()
        self.data = {}          # joint -> {t, q_ref, q_meas, tau}
        self.timing = {'t': [], 'loop_dt': [], 'ref_age': []}
        self.joint_names = []
        self.lock = threading.Lock()

        self.create_subscription(
            JointState, '/impedance/debug', self.cb_debug, BE_QOS)
        self.create_subscription(
            Float64MultiArray, '/impedance/timing', self.cb_timing, BE_QOS)

    def cb_debug(self, msg):
        t = time.time() - self.t0
        with self.lock:
            if not self.joint_names:
                self.joint_names = list(msg.name)
            for i, name in enumerate(msg.name):
                if name not in self.data:
                    self.data[name] = {'t': [], 'q_ref': [], 'q_meas': [], 'tau': []}
                d = self.data[name]
                d['t'].append(t)
                d['q_ref'].append(msg.position[i] if i < len(msg.position) else NAN)
                d['q_meas'].append(msg.velocity[i] if i < len(msg.velocity) else NAN)
                d['tau'].append(msg.effort[i] if i < len(msg.effort) else NAN)

    def cb_timing(self, msg):
        t = time.time() - self.t0
        with self.lock:
            self.timing['t'].append(t)
            self.timing['loop_dt'].append(msg.data[0] if len(msg.data) > 0 else NAN)
            self.timing['ref_age'].append(msg.data[1] if len(msg.data) > 1 else NAN)

    def done(self):
        return time.time() - self.t0 >= self.duration

    def snapshot(self):
        with self.lock:
            return dict(self.data), dict(self.timing), list(self.joint_names)

    def clear_data(self):
        """清空已采集数据并重置计时 (用于 wait_for_start: 只统计步态启动后的数据)"""
        with self.lock:
            self.data = {}
            self.timing = {'t': [], 'loop_dt': [], 'ref_age': []}
            self.joint_names = []
            self.t0 = time.time()

    def joint_active(self, joint, threshold=0.01):
        """判断参考是否已偏离 0 (步态是否启动)"""
        with self.lock:
            jd = self.data.get(joint, {})
            refs = jd.get('q_ref', [])
        return bool(refs) and any(abs(r) > threshold for r in refs[-30:])


def compute_stats(d):
    """给定 {t, q_ref, q_meas, tau} 数组, 计算误差统计 (忽略 NaN)"""
    ref = np.array(d['q_ref'], dtype=float)
    meas = np.array(d['q_meas'], dtype=float)
    err = ref - meas
    err = err[np.isfinite(err)]
    if len(err) == 0:
        return {'rms': NAN, 'max': NAN, 'mean': NAN, 'samples': 0}
    return {
        'rms': float(np.sqrt(np.mean(err ** 2))),
        'max': float(np.max(np.abs(err))),
        'mean': float(np.mean(np.abs(err))),
        'samples': int(len(err)),
    }


def plot_figure(data, timing, joint, out_png, title_suffix=''):
    """绘制多面板图: 位置跟踪 / 误差 / 力矩 / 时序"""
    jd = data.get(joint)
    if jd is None or not jd['t']:
        print(f'[WARN] 关节 {joint} 无数据, 检查 /impedance/debug 是否有消息')
        return False

    t = np.array(jd['t'])
    q_ref = np.array(jd['q_ref'], dtype=float)
    q_meas = np.array(jd['q_meas'], dtype=float)
    tau = np.array(jd['tau'], dtype=float)
    err = q_ref - q_meas
    st = compute_stats(jd)

    # 滑动窗口 RMS (1s, 按实际采样率自适应, 支持 200/500Hz)
    dt_med = float(np.median(np.diff(t))) if len(t) > 1 else 0.002
    win = max(20, int(round(1.0 / max(dt_med, 1e-4))))
    err_rms = np.full_like(err, NAN)
    for i in range(win, len(err)):
        seg = err[i - win:i]
        seg = seg[np.isfinite(seg)]
        err_rms[i] = np.sqrt(np.mean(seg ** 2)) if len(seg) else NAN

    # 时序 (延迟)
    tt = np.array(timing['t'])
    loop_dt = np.array(timing['loop_dt'], dtype=float) * 1000  # ms
    ref_age = np.array(timing['ref_age'], dtype=float) * 1000  # ms

    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    fig.suptitle(
        f'阻抗控制器监控 — {joint}{title_suffix}\n'
        f'RMS误差={st["rms"]:.4f} rad ({st["rms"]*57.3:.2f}°)  '
        f'最大误差={st["max"]:.4f} rad ({st["max"]*57.3:.2f}°)  '
        f'平均误差={st["mean"]:.4f} rad',
        fontsize=11)

    # 1. 位置跟踪
    ax = axes[0]
    ax.plot(t, q_ref, 'b-', lw=1.2, label='q_ref (参考)')
    ax.plot(t, q_meas, 'r-', lw=1.0, alpha=0.8, label='q_meas (实际)')
    ax.set_ylabel('位置 [rad]')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # 2. 误差
    ax = axes[1]
    ax.plot(t, err, 'k-', lw=1.0, label='误差 (ref-meas)')
    ax.plot(t, err_rms, 'm--', lw=1.5, label='滑动RMS误差 (1s)')
    ax.axhline(0, color='gray', lw=0.8)
    ax.set_ylabel('误差 [rad]')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # 3. 力矩
    ax = axes[2]
    ax.plot(t, tau, 'g-', lw=1.0, label='命令力矩 τ')
    ax.axhline(0, color='gray', lw=0.8)
    ax.set_ylabel('力矩 [iq]')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # 4. 时序/延迟
    ax = axes[3]
    ax.plot(tt, loop_dt, 'c-', lw=1.0, label='控制周期 (标称5ms)')
    ax.plot(tt, ref_age, 'orange', lw=1.2, label='参考陈旧度 (延迟)')
    ax.set_ylabel('时间 [ms]')
    ax.set_xlabel('时间 [s]')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    if out_png:
        fig.savefig(out_png, dpi=120)
        print(f'[OK] 图表已保存: {out_png}')
    plt.show()
    return True


def main():
    # 过滤 launch Node 追加的 --ros-args ... (否则 argparse 报错)
    if '--ros-args' in sys.argv:
        sys.argv = sys.argv[:sys.argv.index('--ros-args')]

    parser = argparse.ArgumentParser(description='阻抗控制器误差/延迟监控')
    parser.add_argument('--duration', type=float, default=15.0, help='采集时长 [s]')
    parser.add_argument('--joint', default='left_knee', help='要展示的关节 (默认 left_knee)')
    parser.add_argument('--motor', type=int, default=None,
                        help='按电机号选关节 (从 motor_map_file 解析, 优先于 --joint)')
    parser.add_argument('--motor-map-file', default=DEFAULT_MOTOR_MAP_FILE,
                        help='关节↔电机映射文件 (默认 exo_controllers.yaml)')
    parser.add_argument('--live', action='store_true', help='实时滚动窗口模式')
    parser.add_argument('--csv', default=None, help='导出CSV路径 (可选)')
    parser.add_argument('--wait-for-start', action='store_true',
                        help='等待步态启动 (参考偏离0) 后再开始采集, 避免空闲数据污染统计')
    args = parser.parse_args()

    # 从配置文件解析 关节↔电机 对应 (失败仅告警, 不阻塞)
    mapping = None
    try:
        mapping = load_motor_map(args.motor_map_file)
    except Exception as e:
        print(f'[WARN] 读取映射文件失败: {e} (仅按关节名显示)', flush=True)

    # 解析要展示的关节: --motor N 优先, 否则用 --joint
    joint = args.joint
    title_suffix = ''
    if args.motor is not None:
        if mapping is None or args.motor not in mapping:
            avail = sorted(mapping) if mapping else []
            print(f'[ERR] 电机 {args.motor} 不在映射中, 可用电机号: {avail}', flush=True)
            rclpy.init(args=sys.argv)
            rclpy.shutdown()
            return 1
        joint = mapping[args.motor]
        title_suffix = f' (motor {args.motor})'
        print(f'[INFO] 电机 {args.motor} → 关节 {joint} (来自 {args.motor_map_file})', flush=True)
    elif mapping is not None:
        rev = {j: m for m, j in mapping.items()}
        if joint in rev:
            title_suffix = f' (motor {rev[joint]})'
            print(f'[INFO] 关节 {joint} ↔ 电机 {rev[joint]} (来自 {args.motor_map_file})', flush=True)

    rclpy.init()
    monitor = ControlMonitor(joint, args.duration)
    spin_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spin_thread.start()

    # 等待步态启动后再开始正式采集 (只统计步态运行中的数据)
    if args.wait_for_start:
        print('[WAIT] 等待步态启动 (发送: ros2 topic pub /cpg/state_cmd ... start) ...', flush=True)
        while not monitor.joint_active(joint):
            time.sleep(0.1)
        monitor.clear_data()
        print('[WAIT] 检测到步态, 开始采集 ...', flush=True)

    if args.live:
        plt.ion()
        fig, axes = plt.subplots(2, 1, figsize=(11, 7))
        print(f'[LIVE] 实时监控 {joint}{title_suffix} ... Ctrl+C 结束')
        last_plot = 0.0
        try:
            while not monitor.done():
                data, timing, _ = monitor.snapshot()
                jd = data.get(joint)
                if jd and len(jd['t']) > 5:
                    t = np.array(jd['t']); qr = np.array(jd['q_ref']); qm = np.array(jd['q_meas'])
                    ax = axes[0]
                    ax.clear(); ax.plot(t, qr, 'b-', label='ref'); ax.plot(t, qm, 'r-', label='meas')
                    ax.set_ylabel('rad'); ax.legend(); ax.grid(True, alpha=0.3)
                    ax = axes[1]
                    ax.clear(); ax.plot(t, qr - qm, 'k-', label='err')
                    ax.set_ylabel('rad'); ax.legend(); ax.grid(True, alpha=0.3)
                    plt.pause(0.05)
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            plt.ioff()
    else:
        print(f'[REC] 采集 {args.duration}s 的 {joint}{title_suffix} 控制数据 ... (先确保步态已 start)')
        while not monitor.done():
            time.sleep(0.05)
        print('[REC] 采集完成, 生成图表 ...')

    data, timing, joint_names = monitor.snapshot()
    rclpy.shutdown()

    # 统计汇总 (flush 确保被 kill 前也能看到)
    out = ['\n===== 控制性能汇总 =====']
    for j in joint_names:
        st = compute_stats(data.get(j, {}))
        if st['samples'] > 0:
            out.append(f'  {j}: 采样={st["samples"]}  RMS误差={st["rms"]*1000:.1f} mrad '
                       f'({st["rms"]*57.3:.2f}°)  最大={st["max"]*57.3:.2f}°  平均={st["mean"]*57.3:.2f}°')
    ld = [x for x in timing['loop_dt'] if x == x]
    ra = [x for x in timing['ref_age'] if x == x]
    if ld:
        out.append(f'  控制周期: 均值={np.mean(ld)*1000:.2f}ms  抖动(σ)={np.std(ld)*1000:.3f}ms  '
                   f'最大={np.max(ld)*1000:.2f}ms')
    if ra:
        out.append(f'  参考陈旧度(延迟): 均值={np.mean(ra)*1000:.2f}ms  最大={np.max(ra)*1000:.2f}ms')
    out.append('=======================\n')
    text = '\n'.join(out)
    print(text, flush=True)
    with open('control_monitor_stats.txt', 'w') as f:
        f.write(text)
    print('[OK] 统计已写入 control_monitor_stats.txt', flush=True)

    if args.csv and joint_names:
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'q_ref', 'q_meas', 'tau'])
            jd = data.get(joint, {})
            for k in range(min(len(jd.get('t', [])), 100000)):
                w.writerow([jd['t'][k], jd['q_ref'][k], jd['q_meas'][k], jd['tau'][k]])
        print(f'[OK] CSV 已导出: {args.csv}')

    # 出图
    out_png = f'control_monitor_{joint}.png'
    try:
        plot_figure(data, timing, joint, out_png, title_suffix)
    except Exception as e:
        print(f'[ERR] 出图失败: {e}')


if __name__ == '__main__':
    main()
