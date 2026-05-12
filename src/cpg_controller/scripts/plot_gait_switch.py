#!/usr/bin/env python3
"""
可视化 Gait Mode 切换过程中的关节角度变化
用法:
  1. 启动 CPG 节点
  2. 运行本脚本: python3 plot_gait_switch.py
  3. 在另一个终端发送切换命令:
     ros2 topic pub /cpg/gait_mode_cmd std_msgs/msg/String "data: 'slope_up'"
  4. 等待切换完成，按 Ctrl+C 保存图像
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import threading
import time


class GaitSwitchPlotter(Node):
    def __init__(self):
        super().__init__('gait_switch_plotter')

        # 订阅 joint_states
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 订阅 gait_mode 状态
        self.gait_mode_sub = self.create_subscription(
            String,
            '/cpg/gait_mode',
            self.gait_mode_callback,
            10
        )

        # 数据存储
        self.joint_names = ['left_hip3', 'left_knee', 'left_ankle',
                           'right_hip3', 'right_knee', 'right_ankle']
        self.data = {name: {'time': [], 'angle': []} for name in self.joint_names}
        self.gait_mode_changes = []  # (time, mode_name)

        self.start_time = None
        self.recording = True
        self.lock = threading.Lock()

        self.get_logger().info('Gait Switch Plotter 已启动，开始记录数据...')

    def joint_state_callback(self, msg):
        if not self.recording:
            return

        with self.lock:
            # 使用节点时钟
            now = self.get_clock().now()
            current_time = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9

            if self.start_time is None:
                self.start_time = current_time

            relative_time = current_time - self.start_time

            # 记录关节角度
            for i, name in enumerate(msg.name):
                if name in self.data and i < len(msg.position):
                    self.data[name]['time'].append(relative_time)
                    self.data[name]['angle'].append(np.degrees(msg.position[i]))

    def gait_mode_callback(self, msg):
        if not self.recording:
            return

        with self.lock:
            if self.start_time is not None:
                now = self.get_clock().now()
                current_time = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
                relative_time = current_time - self.start_time

                self.gait_mode_changes.append((relative_time, msg.data))
                self.get_logger().info(f'Gait Mode 切换: {msg.data} at t={relative_time:.3f}s')

    def save_plot(self, filename='gait_switch_plot.png'):
        with self.lock:
            fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
            fig.suptitle('Gait Mode 切换过程 - 关节角度变化', fontsize=14)

            joint_labels = {
                'left_hip3': ('左髋', 'Hip'),
                'left_knee': ('左膝', 'Knee'),
                'left_ankle': ('左踝', 'Ankle'),
                'right_hip3': ('右髋', 'Hip'),
                'right_knee': ('右膝', 'Knee'),
                'right_ankle': ('右踝', 'Ankle'),
            }

            for idx, name in enumerate(self.joint_names):
                row = idx // 3
                col = idx % 2
                # 重新映射：左腿在左列，右腿在右列
                if idx < 3:
                    row = idx
                    col = 0
                else:
                    row = idx - 3
                    col = 1

                ax = axes[row][col]
                data = self.data[name]

                if data['time']:
                    ax.plot(data['time'], data['angle'], 'b-', linewidth=1.5, label='角度')

                    # 标记 gait mode 切换点
                    for t, mode in self.gait_mode_changes:
                        if data['time'][0] <= t <= data['time'][-1]:
                            ax.axvline(x=t, color='r', linestyle='--', alpha=0.7, linewidth=1)
                            ax.text(t, ax.get_ylim()[1], mode,
                                   rotation=90, va='top', ha='right', fontsize=8, color='r')

                    side = '左' if 'left' in name else '右'
                    joint = joint_labels[name][1]
                    ax.set_ylabel(f'{side}{joint}\n角度 (°)')
                    ax.grid(True, alpha=0.3)
                    ax.legend(loc='upper right', fontsize=8)

            axes[-1][0].set_xlabel('时间 (s)')
            axes[-1][1].set_xlabel('时间 (s)')

            plt.tight_layout()
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            plt.close()

            self.get_logger().info(f'图像已保存: {filename}')


def main():
    rclpy.init()
    node = GaitSwitchPlotter()

    print("\n" + "="*60)
    print("Gait Mode 切换可视化工具")
    print("="*60)
    print("\n使用方法:")
    print("1. 本脚本已开始记录 /joint_states 数据")
    print("2. 在另一个终端发送 gait mode 切换命令:")
    print("   ros2 topic pub /cpg/gait_mode_cmd std_msgs/msg/String \"data: 'slope_up'\"")
    print("3. 等待切换完成")
    print("4. 按 Ctrl+C 停止记录并保存图像")
    print("="*60 + "\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n正在保存图像...")
        node.save_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
