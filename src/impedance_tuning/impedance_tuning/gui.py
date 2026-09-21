#!/usr/bin/env python3
"""Qt GUI for safe, multi-joint impedance tuning with live cycle metrics."""

import csv
import json
import math
import os
import sys
import threading
import time

import numpy as np
import yaml

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib import font_manager


def configure_chinese_font():
    """Register a known CJK font by path; do not rely on Matplotlib's stale cache."""
    candidates = [
        '/usr/share/fonts/winfonts/simhei.ttf',
        '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    ]
    for path in candidates:
        if not os.path.exists(path):
            continue
        font_manager.fontManager.addfont(path)
        family = font_manager.FontProperties(fname=path).get_name()
        matplotlib.rcParams['font.sans-serif'] = [family, 'DejaVu Sans']
        return family
    matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans']
    return 'DejaVu Sans'


CHINESE_FONT_FAMILY = configure_chinese_font()
matplotlib.rcParams['axes.unicode_minus'] = False
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFileDialog, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QVBoxLayout, QWidget,
)

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from terrain_adaptive_cpg.msg import CPGDebug

from impedance_tuning.metrics import analyze_cycle
from impedance_tuning.config_io import save_controller_parameters


DEFAULT_CONFIG = '/home/lsy/vins_ws/src/exoskeleton_hardware/config/exo_controllers.yaml'
SENSOR_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)


def load_controller_config(path):
    with open(path, encoding='utf-8') as stream:
        root = yaml.safe_load(stream)
    params = root['exo_impedance_controller']['ros__parameters']
    return {
        'joints': list(params['joints']),
        'kp': [float(v) for v in params['default_kp']],
        'kd': [float(v) for v in params['default_kd']],
        'tube_half_width': [
            float(v) for v in params.get(
                'default_tube_half_width', [0.0] * len(params['joints']))],
        'tube_transition_width': [
            float(v) for v in params.get(
                'default_tube_transition_width', [0.0] * len(params['joints']))],
        'limits': [float(v) for v in params['torque_limit']],
    }


class TuningNode(Node):
    def __init__(self):
        super().__init__('impedance_tuning_gui')
        self.declare_parameter('controllers_config', DEFAULT_CONFIG)
        config_path = self.get_parameter('controllers_config').value
        self.config_path = config_path
        self.config = load_controller_config(config_path)
        self.lock = threading.Lock()
        self.selected_index = 0
        self.kp = list(self.config['kp'])
        self.kd = list(self.config['kd'])
        self.tube_half_width = list(self.config['tube_half_width'])
        self.tube_transition_width = list(self.config['tube_transition_width'])
        self.enabled = [False] * len(self.config['joints'])
        self.zero_sequence = 0
        self.running = True
        self.started_at = time.monotonic()
        self.latest_phase = float('nan')
        self.previous_sample_phase = float('nan')
        self.latest_loop_dt = float('nan')
        self.latest_ref_age = float('nan')
        self.last_debug_at = None
        self.actual_iq = {}
        self.latest_positions = {}
        self.plot_samples = []
        self.session_samples = []
        self.current_cycle = []
        self.have_cycle_boundary = False
        self.cycle_metrics = []

        self.command_pub = self.create_publisher(
            Float64MultiArray, '/impedance_tuning/command', 10)
        self.cpg_command_pub = self.create_publisher(String, '/cpg/state_cmd', 10)
        self.create_subscription(
            JointState, '/impedance/debug', self._debug_cb, SENSOR_QOS)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, SENSOR_QOS)
        self.create_subscription(
            Float64MultiArray, '/impedance/timing', self._timing_cb, SENSOR_QOS)
        self.create_subscription(CPGDebug, '/cpg/debug', self._cpg_cb, 20)
        self.create_subscription(
            DynamicJointState, '/dynamic_joint_states', self._dynamic_cb, 20)
        self.create_timer(0.05, self._publish_tuning_command)

    def _publish_tuning_command(self):
        with self.lock:
            if not self.running:
                return
            enabled_mask = sum(1 << i for i, enabled in enumerate(self.enabled) if enabled)
            # V3: [version, mask, zero_seq, kp, kd, tube_half, transition, ...]
            values = [3.0, float(enabled_mask), float(self.zero_sequence)]
            for parameters in zip(
                    self.kp, self.kd, self.tube_half_width,
                    self.tube_transition_width):
                values.extend(parameters)
        msg = Float64MultiArray()
        msg.data = values
        self.command_pub.publish(msg)

    def _cpg_cb(self, msg):
        with self.lock:
            self.latest_phase = float(msg.phase)

    def _timing_cb(self, msg):
        with self.lock:
            if len(msg.data) >= 2:
                self.latest_loop_dt = float(msg.data[0])
                self.latest_ref_age = float(msg.data[1])

    def _dynamic_cb(self, msg):
        values = {}
        for joint, interfaces in zip(msg.joint_names, msg.interface_values):
            try:
                index = list(interfaces.interface_names).index('effort')
                values[joint] = float(interfaces.values[index])
            except (ValueError, IndexError):
                continue
        with self.lock:
            self.actual_iq.update(values)

    def _debug_cb(self, msg):
        with self.lock:
            self.last_debug_at = time.monotonic()
            for i, name in enumerate(msg.name):
                if i < len(msg.velocity):
                    self.latest_positions[name] = float(msg.velocity[i])
            if self.selected_index >= len(self.config['joints']):
                return
            joint = self.config['joints'][self.selected_index]
            try:
                index = list(msg.name).index(joint)
            except ValueError:
                return
            if index >= len(msg.position) or index >= len(msg.velocity):
                return
            sample = {
                't': time.monotonic() - self.started_at,
                'joint': joint,
                'q_ref': float(msg.position[index]),
                # /impedance/debug.velocity is intentionally the measured position.
                'q_meas': float(msg.velocity[index]),
                'iq_cmd': float(msg.effort[index]) if index < len(msg.effort) else float('nan'),
                'iq_meas': self.actual_iq.get(joint, float('nan')),
                'loop_dt': self.latest_loop_dt,
                'ref_age': self.latest_ref_age,
                'phase': self.latest_phase,
                'kp': self.kp[self.selected_index],
                'kd': self.kd[self.selected_index],
                'tube_half_width': self.tube_half_width[self.selected_index],
                'tube_transition_width': self.tube_transition_width[self.selected_index],
                'armed': self.enabled[self.selected_index],
            }
            self.plot_samples.append(sample)
            if len(self.plot_samples) > 10000:
                del self.plot_samples[:2000]
            if not self.enabled[self.selected_index]:
                self.previous_sample_phase = sample['phase']
                return
            self.session_samples.append(sample)
            phase = sample['phase']
            wrapped = (
                math.isfinite(self.previous_sample_phase) and math.isfinite(phase) and
                self.previous_sample_phase > 2.5 and phase < -2.5)
            if wrapped:
                if self.have_cycle_boundary and self.current_cycle:
                    result = analyze_cycle(
                        self.current_cycle, self.config['limits'][self.selected_index])
                    if result is not None:
                        result.update({
                            'cycle': len(self.cycle_metrics) + 1,
                            'joint': joint,
                            'kp': self.kp[self.selected_index],
                            'kd': self.kd[self.selected_index],
                            'tube_half_width': self.tube_half_width[self.selected_index],
                            'tube_transition_width': self.tube_transition_width[
                                self.selected_index],
                        })
                        self.cycle_metrics.append(result)
                self.have_cycle_boundary = True
                self.current_cycle = []
            self.current_cycle.append(sample)
            self.previous_sample_phase = phase

    def _joint_state_cb(self, msg):
        with self.lock:
            for index, name in enumerate(msg.name):
                if index < len(msg.position):
                    self.latest_positions[name] = float(msg.position[index])

    def controller_health(self):
        publisher_count = self.count_publishers('/impedance/debug')
        with self.lock:
            age = (time.monotonic() - self.last_debug_at
                   if self.last_debug_at is not None else float('inf'))
        return publisher_count > 0 and age < 0.5, publisher_count, age

    def set_joint(self, index):
        with self.lock:
            self.selected_index = int(index)
            self.current_cycle = []
            self.have_cycle_boundary = False
            self.previous_sample_phase = float('nan')

    def set_parameters(self, kp, kd, tube_half_width, tube_transition_width):
        with self.lock:
            self.kp[self.selected_index] = float(kp)
            self.kd[self.selected_index] = float(kd)
            self.tube_half_width[self.selected_index] = float(tube_half_width)
            self.tube_transition_width[self.selected_index] = float(tube_transition_width)
            self.current_cycle = []
            self.have_cycle_boundary = False
            self.previous_sample_phase = float('nan')

    def set_joint_enabled(self, index, enabled):
        with self.lock:
            self.selected_index = int(index)
            self.enabled[index] = bool(enabled)
            self.current_cycle = []
            self.have_cycle_boundary = False
            self.previous_sample_phase = float('nan')
        self._publish_tuning_command()

    def stop_all_outputs(self):
        with self.lock:
            self.enabled = [False] * len(self.enabled)
            self.current_cycle = []
            self.have_cycle_boundary = False
            self.previous_sample_phase = float('nan')
        self._publish_tuning_command()

    def save_parameters(self):
        with self.lock:
            kp = list(self.kp)
            kd = list(self.kd)
            tube_half_width = list(self.tube_half_width)
            tube_transition_width = list(self.tube_transition_width)
        save_controller_parameters(
            self.config_path, self.config['joints'], kp, kd,
            tube_half_width, tube_transition_width)
        with self.lock:
            self.config['kp'] = kp
            self.config['kd'] = kd
            self.config['tube_half_width'] = tube_half_width
            self.config['tube_transition_width'] = tube_transition_width
        return self.config_path

    def command_cpg(self, command):
        msg = String()
        msg.data = command
        self.cpg_command_pub.publish(msg)

    def request_zero_position(self):
        with self.lock:
            self.enabled = [False] * len(self.enabled)
            self.zero_sequence = max(
                self.zero_sequence + 1, int(time.time() * 1000.0))
            self.current_cycle = []
            self.have_cycle_boundary = False
            self.previous_sample_phase = float('nan')
        self._publish_tuning_command()

    def reset_session(self):
        with self.lock:
            self.session_samples = []
            self.current_cycle = []
            self.have_cycle_boundary = False
            self.cycle_metrics = []
            self.previous_sample_phase = float('nan')

    def snapshot(self):
        with self.lock:
            return (
                list(self.plot_samples[-5000:]),
                list(self.cycle_metrics), list(self.enabled),
                self.selected_index, list(self.kp), list(self.kd),
                list(self.tube_half_width), list(self.tube_transition_width),
                dict(self.latest_positions),
            )

    def export(self, csv_path):
        with self.lock:
            samples = list(self.session_samples)
            metrics = list(self.cycle_metrics)
        if not samples:
            raise RuntimeError('当前会话还没有启用关节输出后的调参数据')
        fields = list(samples[0].keys())
        with open(csv_path, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(samples)
        summary_path = os.path.splitext(csv_path)[0] + '_cycles.json'
        with open(summary_path, 'w', encoding='utf-8') as stream:
            json.dump(metrics, stream, ensure_ascii=False, indent=2, allow_nan=True)
        return summary_path

    def safe_shutdown(self):
        self.command_cpg('stop')
        self.stop_all_outputs()
        for _ in range(3):
            self._publish_tuning_command()
            time.sleep(0.02)
        with self.lock:
            self.running = False


class TuningWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('CPG 多关节阻抗调参')
        self.resize(1250, 900)
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        controls = QGroupBox('多关节联调（关节按钮按下=该关节允许输出；可同时选择）')
        grid = QGridLayout(controls)
        self.joint_buttons = []
        joint_row = QHBoxLayout()
        for index, name in enumerate(node.config['joints']):
            button = QPushButton(name)
            button.setCheckable(True)
            button.setToolTip(
                '按下后该关节使用自己的虚拟管道参数；再次点击停止该关节输出')
            button.toggled.connect(
                lambda checked, joint_index=index: self._joint_toggled(joint_index, checked))
            self.joint_buttons.append(button)
            joint_row.addWidget(button)
        self.edit_joint_combo = QComboBox()
        self.edit_joint_combo.addItems(node.config['joints'])
        self.kp_spin = self._gain_spin(node.config['kp'][0])
        self.kd_spin = self._gain_spin(node.config['kd'][0])
        self.kp_step = self._step_spin(5.0)
        self.kd_step = self._step_spin(1.0)
        self.tube_spin = self._angle_spin(
            math.degrees(node.config['tube_half_width'][0]), 45.0)
        self.transition_spin = self._angle_spin(
            math.degrees(node.config['tube_transition_width'][0]), 20.0)
        grid.addWidget(QLabel('关节输出开关'), 0, 0)
        grid.addLayout(joint_row, 0, 1, 1, 5)
        grid.addWidget(QLabel('Kp/Kd 编辑对象'), 1, 0)
        grid.addWidget(self.edit_joint_combo, 1, 1)
        grid.addWidget(QLabel('Kp'), 1, 2)
        grid.addWidget(self.kp_spin, 1, 3)
        grid.addWidget(QLabel('Kp步长'), 1, 4)
        grid.addWidget(self.kp_step, 1, 5)
        grid.addWidget(QLabel('Kd'), 2, 2)
        grid.addWidget(self.kd_spin, 2, 3)
        grid.addWidget(QLabel('Kd步长'), 2, 4)
        grid.addWidget(self.kd_step, 2, 5)
        grid.addWidget(QLabel('管道半宽 [°]'), 3, 2)
        grid.addWidget(self.tube_spin, 3, 3)
        grid.addWidget(QLabel('壁面平滑宽 [°]'), 3, 4)
        grid.addWidget(self.transition_spin, 3, 5)

        self.apply_button = QPushButton('应用阻抗 / 管道参数')
        self.kp_down_button = QPushButton('Kp −')
        self.kp_up_button = QPushButton('Kp +')
        self.kd_down_button = QPushButton('Kd −')
        self.kd_up_button = QPushButton('Kd +')
        self.disarm_button = QPushButton('停止全部输出')
        self.disarm_button.setToolTip('四个关节均发送零iq命令；这不等于硬件断电或抱闸')
        self.disarm_button.setStyleSheet('background:#b22222;color:white;font-weight:bold')
        self.start_button = QPushButton('启动 CPG 轨迹')
        self.stop_button = QPushButton('停止 CPG 并停输出')
        self.zero_button = QPushButton('当前位置设为上位机零位')
        self.zero_button.setToolTip(
            '仅在上位机记录编码器偏置；不修改STM32或电机内部零点')
        self.reset_button = QPushButton('清空本次统计')
        self.save_button = QPushButton('保存 CSV + 周期报告')
        self.save_params_button = QPushButton('保存阻抗/管道参数到 YAML')
        button_row = QHBoxLayout()
        for button in (
            self.kp_down_button, self.kp_up_button,
            self.kd_down_button, self.kd_up_button, self.apply_button,
            self.disarm_button,
            self.start_button, self.stop_button,
        ):
            button_row.addWidget(button)
        grid.addLayout(button_row, 4, 0, 1, 6)
        utility_row = QHBoxLayout()
        utility_row.addWidget(self.zero_button)
        utility_row.addWidget(self.reset_button)
        utility_row.addWidget(self.save_button)
        utility_row.addWidget(self.save_params_button)
        utility_row.addStretch(1)
        grid.addLayout(utility_row, 5, 0, 1, 6)
        root.addWidget(controls)

        self.flow = QLabel(
            '信息流：参考源（当前CPG）→ 虚拟管道壁阻抗 '
            '+ 透明度补偿(0) + 助力曲线(0) → iq命令 → 电机')
        self.flow.setStyleSheet('color:#444')
        self.status = QLabel(
            '状态：关节输出未启用。先选择关节并设置较低Kp/Kd，再启用所选关节并启动CPG。')
        self.status.setStyleSheet('font-size:15px;font-weight:bold')
        self.controller_status = QLabel('阻抗控制器：正在检查……')
        self.position_status = QLabel('当前关节角：等待编码器反馈……')
        self.metrics = QLabel('等待完成一个CPG周期……')
        self.metrics.setWordWrap(True)
        root.addWidget(self.flow)
        root.addWidget(self.status)
        root.addWidget(self.controller_status)
        root.addWidget(self.position_status)
        root.addWidget(self.metrics)

        self.figure = Figure(figsize=(11, 7), tight_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.axes = self.figure.subplots(4, 1, sharex=True)
        root.addWidget(self.canvas, stretch=1)

        self.edit_joint_combo.currentIndexChanged.connect(self._select_joint)
        self.apply_button.clicked.connect(self._apply_gains)
        self.kp_down_button.clicked.connect(
            lambda: self._step_gain(self.kp_spin, -self.kp_step.value()))
        self.kp_up_button.clicked.connect(
            lambda: self._step_gain(self.kp_spin, self.kp_step.value()))
        self.kd_down_button.clicked.connect(
            lambda: self._step_gain(self.kd_spin, -self.kd_step.value()))
        self.kd_up_button.clicked.connect(
            lambda: self._step_gain(self.kd_spin, self.kd_step.value()))
        self.disarm_button.clicked.connect(self._disarm)
        self.start_button.clicked.connect(self._start_cpg)
        self.stop_button.clicked.connect(self._stop_cpg)
        self.zero_button.clicked.connect(self._zero_current_position)
        self.reset_button.clicked.connect(self._reset)
        self.save_button.clicked.connect(self._save)
        self.save_params_button.clicked.connect(self._save_params)

        timer = QTimer(self)
        timer.timeout.connect(self._refresh)
        timer.start(200)
        self.plot_timer = timer

    @staticmethod
    def _gain_spin(value):
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 10000.0)
        spin.setDecimals(2)
        spin.setValue(value)
        return spin

    @staticmethod
    def _step_spin(value):
        spin = QDoubleSpinBox()
        spin.setRange(0.01, 1000.0)
        spin.setDecimals(2)
        spin.setValue(value)
        return spin

    @staticmethod
    def _angle_spin(value, maximum):
        spin = QDoubleSpinBox()
        spin.setRange(0.0, maximum)
        spin.setDecimals(2)
        spin.setSingleStep(0.5)
        spin.setValue(value)
        return spin

    def _select_joint(self, index):
        self.node.set_joint(index)
        snapshot = self.node.snapshot()
        self.kp_spin.setValue(snapshot[4][index])
        self.kd_spin.setValue(snapshot[5][index])
        self.tube_spin.setValue(math.degrees(snapshot[6][index]))
        self.transition_spin.setValue(math.degrees(snapshot[7][index]))

    def _joint_toggled(self, index, checked):
        if checked:
            online, _, _ = self.node.controller_health()
            if not online:
                self.joint_buttons[index].blockSignals(True)
                self.joint_buttons[index].setChecked(False)
                self.joint_buttons[index].blockSignals(False)
                QMessageBox.warning(
                    self, '阻抗控制器离线',
                    '没有收到 /impedance/debug。关节输出未启用。\n'
                    '请检查 exo_impedance_controller 是否已成功加载为 active。')
                self.status.setText('启用被拒绝：阻抗控制器离线，未发送输出许可。')
                return
        if self.edit_joint_combo.currentIndex() != index:
            self.edit_joint_combo.setCurrentIndex(index)
        else:
            self._select_joint(index)
        self.node.set_joint_enabled(index, checked)
        self.joint_buttons[index].setStyleSheet(
            'background:#2e8b57;color:white;font-weight:bold' if checked else '')
        action = '已启用' if checked else '已停止'
        self.status.setText(
            f'{action} {self.node.config["joints"][index]} 输出；按下的其他关节状态不变。')

    def _apply_gains(self):
        self.node.set_parameters(
            self.kp_spin.value(), self.kd_spin.value(),
            math.radians(self.tube_spin.value()),
            math.radians(self.transition_spin.value()))
        self.status.setText(
            f'已应用 Kp={self.kp_spin.value():.2f}, Kd={self.kd_spin.value():.2f}, '
            f'管道半宽={self.tube_spin.value():.2f}°, '
            f'平滑宽={self.transition_spin.value():.2f}°；周期统计重新开始。')

    def _step_gain(self, spin, delta):
        spin.setValue(max(0.0, spin.value() + delta))
        self._apply_gains()

    def _disarm(self):
        self.node.stop_all_outputs()
        for button in self.joint_buttons:
            button.blockSignals(True)
            button.setChecked(False)
            button.setStyleSheet('')
            button.blockSignals(False)
        self.status.setText('全部关节输出已停止，四个关节均降为零iq命令。')

    def _start_cpg(self):
        online, _, _ = self.node.controller_health()
        if not online:
            QMessageBox.warning(
                self, '阻抗控制器离线',
                '为避免CPG运行但下游控制器未工作，已拒绝启动。')
            return
        if not any(self.node.snapshot()[2]):
            QMessageBox.warning(
                self, '关节输出未启用',
                '请先设置低增益并按下至少一个关节按钮，再启动CPG。')
            return
        self.node.command_cpg('start')
        enabled = [
            name for name, active in
            zip(self.node.config['joints'], self.node.snapshot()[2]) if active]
        self.status.setText('CPG运行中，允许输出：' + '、'.join(enabled))

    def _stop_cpg(self):
        self.node.command_cpg('stop')
        self._disarm()
        self.status.setText('CPG已停止，全部关节输出已停止。')

    def _zero_current_position(self):
        answer = QMessageBox.question(
            self, '确认设置上位机软件零位',
            '请先把四个关节放到与URDF零姿态一致的位置并保持不动。\n\n'
            '继续后将停止CPG和全部控制输出，并把四关节当前位置定义为0°。'
            '只会保存上位机编码器偏置，不会改写STM32或电机参数。是否继续？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        self.node.command_cpg('stop')
        self._disarm()
        self.node.request_zero_position()
        self.status.setText(
            '已发送上位机清零请求；控制器将在四关节iq命令降为零后更新编码器偏置。请保持机构不动。')

    def _reset(self):
        self.node.reset_session()
        self.metrics.setText('统计已清空，等待完成一个新的CPG周期……')

    def _save(self):
        path, _ = QFileDialog.getSaveFileName(
            self, '保存调参数据', os.path.expanduser('~/impedance_tuning.csv'), 'CSV (*.csv)')
        if not path:
            return
        try:
            summary = self.node.export(path)
            self.status.setText(f'已保存：{path}；周期报告：{summary}')
        except RuntimeError as exc:
            QMessageBox.warning(self, '无法保存', str(exc))

    def _save_params(self):
        self._apply_gains()
        answer = QMessageBox.question(
            self, '确认保存控制器参数',
            '将把四个关节当前的 Kp/Kd、管道半宽和壁面平滑宽写入：\n'
            f'{self.node.config_path}\n\n运行中的控制器继续使用界面值；下次启动从该 YAML 读取。是否继续？',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        try:
            path = self.node.save_parameters()
            self.status.setText(f'四关节阻抗/管道参数已保存到 {path}；下次启动仍会生效。')
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            QMessageBox.critical(self, '保存参数失败', str(exc))

    def _refresh(self):
        online, publisher_count, debug_age = self.node.controller_health()
        if online:
            self.controller_status.setText(
                f'阻抗控制器：在线，/impedance/debug 龄期={1000.0 * debug_age:.1f} ms')
            self.controller_status.setStyleSheet('color:#207a3c;font-weight:bold')
        else:
            detail = '无发布者' if publisher_count == 0 else '有发布者但未收到新数据'
            self.controller_status.setText(
                f'阻抗控制器：离线（/impedance/debug {detail}）；'
                'RViz同步不代表阻抗控制器已激活')
            self.controller_status.setStyleSheet('color:#b22222;font-weight:bold')
        (samples, cycles, enabled, index, kp_values, kd_values,
         tube_values, transition_values, positions) = self.node.snapshot()
        kp = kp_values[index]
        kd = kd_values[index]
        tube_half_width = tube_values[index]
        transition_width = transition_values[index]
        if positions:
            values = [
                f'{name}={math.degrees(positions[name]):.1f}°'
                for name in self.node.config['joints'] if name in positions
            ]
            self.position_status.setText('当前关节角：' + '，'.join(values))
        if cycles:
            m = cycles[-1]
            self.metrics.setText(
                f'周期 {m["cycle"]}｜Kp={kp:.2f}, Kd={kd:.2f}｜'
                f'RMS={m["rms_rad"]:.4f} rad ({math.degrees(m["rms_rad"]):.2f}°)，'
                f'峰值={math.degrees(m["peak_rad"]):.2f}°，NRMSE={m["nrmse_pct"]:.2f}%｜'
                f'相位滞后={m["phase_lag_ms"]:.1f} ms｜'
                f'命令iq RMS/峰值={m["iq_cmd_rms"]:.2f}/{m["iq_cmd_peak"]:.2f}，'
                f'实测iq RMS={m["iq_meas_rms"]:.2f}，'
                f'饱和={m["saturation_pct"]:.2f}%｜'
                f'周期P99={m["loop_dt_p99_ms"]:.2f} ms，参考龄期P99={m["ref_age_p99_ms"]:.2f} ms')
        if not samples:
            return
        end = samples[-1]['t']
        visible = [s for s in samples if s['t'] >= end - 8.0]
        if len(visible) < 2:
            return
        stride = max(1, int(math.ceil(len(visible) / 1600)))
        visible = visible[::stride]
        t = np.asarray([s['t'] - end for s in visible])
        q_ref = np.asarray([s['q_ref'] for s in visible])
        q_meas = np.asarray([s['q_meas'] for s in visible])
        iq_cmd = np.asarray([s['iq_cmd'] for s in visible])
        iq_meas = np.asarray([s['iq_meas'] for s in visible])
        loop_ms = 1000.0 * np.asarray([s['loop_dt'] for s in visible])
        age_ms = 1000.0 * np.asarray([s['ref_age'] for s in visible])
        for axis in self.axes:
            axis.clear()
            axis.grid(True, alpha=0.25)
        self.axes[0].plot(t, q_ref, label='CPG参考')
        self.axes[0].plot(t, q_meas, label='编码器实测', alpha=0.85)
        self.axes[0].set_ylabel('角度 [rad]')
        self.axes[0].legend(loc='upper right')
        self.axes[1].plot(t, q_ref - q_meas, color='black')
        self.axes[1].axhspan(
            -tube_half_width, tube_half_width, color='#2e8b57', alpha=0.14,
            label='虚拟管道内')
        self.axes[1].set_ylabel('误差 [rad]')
        self.axes[1].legend(loc='upper right')
        self.axes[2].plot(t, iq_cmd, label='命令 iq')
        if np.any(np.isfinite(iq_meas)):
            self.axes[2].plot(t, iq_meas, label='实测 iq', alpha=0.8)
        self.axes[2].set_ylabel('iq')
        self.axes[2].legend(loc='upper right')
        self.axes[3].plot(t, loop_ms, label='控制周期')
        self.axes[3].plot(t, age_ms, label='参考龄期')
        self.axes[3].axhline(2.0, color='gray', ls='--', lw=0.8, label='500 Hz标称')
        self.axes[3].set_ylabel('时间 [ms]')
        self.axes[3].set_xlabel('最近时间 [s]')
        self.axes[3].legend(loc='upper right')
        self.figure.suptitle(
            f'{self.node.config["joints"][index]}  Kp={kp:.2f}  Kd={kd:.2f}  '
            f'管道半宽={math.degrees(tube_half_width):.2f}°  '
            f'平滑宽={math.degrees(transition_width):.2f}°  '
            f'{"该关节输出已启用" if enabled[index] else "该关节输出已停止"}')
        self.canvas.draw_idle()

    def closeEvent(self, event):
        self.node.safe_shutdown()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = TuningNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    app = QApplication(sys.argv)
    window = TuningWindow(node)
    window.show()
    exit_code = app.exec_()
    node.safe_shutdown()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
