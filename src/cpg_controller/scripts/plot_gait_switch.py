#!/usr/bin/env python3
"""
Gait Mode Switch - Left Hip (left_hip3) Full Parameter Recording

Subscribes:
  /cpg/switch_debug  - switch_alpha, omega, per-joint weights, theta, output
  /cpg/state         - SWITCHING / WALKING state

Auto-plots 1s before switch start to 1s after switch complete.
Prints data table with all weight parameters.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import threading
import time as _time

WINDOW_BEFORE = 1.0
WINDOW_AFTER = 1.0
BUFFER_DURATION = 10.0
HIP_IDX = 0  # left_hip3 index in joint_names_

# Debug message layout for joint j:
#   [0]       = switch_alpha
#   [1]       = omega_current
#   [2+j*7+0] = w_offset
#   [2+j*7+1] = w_x[1]
#   [2+j*7+2] = w_x[2]
#   [2+j*7+3] = w_x[3]
#   [2+j*7+4] = w_y[1]
#   [2+j*7+5] = w_y[2]
#   [2+j*7+6] = w_y[3]
#   [2+42+j]  = theta (before clamp)
#   [2+48+j]  = output (after clamp)
BASE = 2 + HIP_IDX * 7
THETA_IDX = 2 + 42 + HIP_IDX
OUTPUT_IDX = 2 + 48 + HIP_IDX


class GaitSwitchPlotter(Node):
    def __init__(self):
        super().__init__('gait_switch_plotter')
        self.create_subscription(Float64MultiArray, '/cpg/switch_debug', self.debug_cb, 10)
        self.create_subscription(String, '/cpg/state', self.state_cb, 10)

        # (t, alpha, omega, w_off, wx1, wx2, wx3, wy1, wy2, wy3, theta, output)
        self.buf = deque()
        self.start_time = None
        self.lock = threading.Lock()
        self.pending_switch = None
        self.completed_switches = []

        self.get_logger().info('Gait Switch Plotter started')

    def _rel_t(self):
        now = self.get_clock().now().seconds_nanoseconds()
        t = now[0] + now[1] * 1e-9
        if self.start_time is None:
            self.start_time = t
        return t - self.start_time

    def debug_cb(self, msg):
        d = msg.data
        if len(d) < 56:
            return
        with self.lock:
            t = self._rel_t()
            row = (
                t,
                d[0],                          # switch_alpha
                d[1],                          # omega
                d[BASE + 0],                   # w_offset
                d[BASE + 1], d[BASE + 2], d[BASE + 3],  # w_x[1..3]
                d[BASE + 4], d[BASE + 5], d[BASE + 6],  # w_y[1..3]
                d[THETA_IDX],                  # theta before clamp
                d[OUTPUT_IDX],                 # output after clamp
            )
            self.buf.append(row)
            cutoff = t - BUFFER_DURATION
            while self.buf and self.buf[0][0] < cutoff:
                self.buf.popleft()

    def state_cb(self, msg):
        state = msg.data.strip()
        with self.lock:
            t = self._rel_t()
            if state == 'SWITCHING' and self.pending_switch is None:
                self.pending_switch = {'start': t}
                self.get_logger().info(f'[SWITCHING] t={t:.3f}s')
            elif state == 'WALKING' and self.pending_switch is not None:
                sw = self.pending_switch
                sw['end'] = t
                self.get_logger().info(f'[WALKING] t={t:.3f}s  duration={t - sw["start"]:.3f}s')
                self.completed_switches.append(sw)
                self.pending_switch = None
                threading.Thread(target=self._delayed_plot, args=(sw,), daemon=True).start()

    def _delayed_plot(self, sw):
        _time.sleep(WINDOW_AFTER)
        self._plot(sw)

    def _plot(self, sw):
        t_lo = sw['start'] - WINDOW_BEFORE
        t_hi = sw['end'] + WINDOW_AFTER

        data = [r for r in self.buf if t_lo <= r[0] <= t_hi]
        if not data:
            self.get_logger().warn('No data in window')
            return

        ts     = np.array([r[0]  for r in data])
        alphas = np.array([r[1]  for r in data])
        omegas = np.array([r[2]  for r in data])
        woffs  = np.array([r[3]  for r in data])
        wx1s   = np.array([r[4]  for r in data])
        wx2s   = np.array([r[5]  for r in data])
        wx3s   = np.array([r[6]  for r in data])
        wy1s   = np.array([r[7]  for r in data])
        wy2s   = np.array([r[8]  for r in data])
        wy3s   = np.array([r[9]  for r in data])
        thetas = np.array([r[10] for r in data])
        outs   = np.array([r[11] for r in data])

        fig, axes = plt.subplots(5, 1, figsize=(14, 14), sharex=True)
        fig.suptitle(
            f'Left Hip (left_hip3) — Gait Mode Switch\n'
            f'start t={sw["start"]:.2f}s  end t={sw["end"]:.2f}s  '
            f'duration={sw["end"] - sw["start"]:.3f}s',
            fontsize=13)

        # 1) switch_alpha
        axes[0].plot(ts, alphas, 'b-', linewidth=1.5)
        axes[0].set_ylabel('switch_alpha')
        axes[0].set_ylim(-0.05, 1.1)
        axes[0].axhline(y=1.0, color='gray', linestyle=':', alpha=0.5)

        # 2) omega
        axes[1].plot(ts, omegas, 'g-', linewidth=1.5)
        axes[1].set_ylabel('omega (rad/s)')

        # 3) weights: w_offset, w_x[1..3]
        axes[2].plot(ts, woffs, '-', linewidth=1.2, label='w_offset')
        axes[2].plot(ts, wx1s,  '-', linewidth=1.2, label='w_x[1]')
        axes[2].plot(ts, wx2s,  '-', linewidth=1.2, label='w_x[2]')
        axes[2].plot(ts, wx3s,  '-', linewidth=1.2, label='w_x[3]')
        axes[2].set_ylabel('w_offset / w_x')
        axes[2].legend(fontsize=7, loc='upper right', ncol=2)

        # 4) weights: w_y[1..3]
        axes[3].plot(ts, wy1s, '-', linewidth=1.2, label='w_y[1]')
        axes[3].plot(ts, wy2s, '-', linewidth=1.2, label='w_y[2]')
        axes[3].plot(ts, wy3s, '-', linewidth=1.2, label='w_y[3]')
        axes[3].set_ylabel('w_y')
        axes[3].legend(fontsize=7, loc='upper right')

        # 5) theta (pre-clamp) vs output (post-clamp)
        axes[4].plot(ts, np.degrees(thetas), 'b-', linewidth=1.2, label='theta (before clamp)')
        axes[4].plot(ts, np.degrees(outs), 'r--', linewidth=1.0, label='output (after clamp)')
        diff_mask = np.abs(thetas - outs) > 1e-6
        if np.any(diff_mask):
            axes[4].scatter(ts[diff_mask], np.degrees(outs[diff_mask]),
                            c='red', s=20, zorder=5, label='clamped')
        axes[4].set_ylabel('angle (deg)')
        axes[4].set_xlabel('time (s)')
        axes[4].legend(fontsize=8, loc='upper right')

        # switch markers
        for ax in axes:
            ax.axvline(x=sw['start'], color='red', linestyle='--', linewidth=1.5, alpha=0.7)
            ax.axvline(x=sw['end'], color='green', linestyle='--', linewidth=1.5, alpha=0.7)
            ax.grid(True, alpha=0.3)

        fig.legend(['switch start', 'switch end'], loc='upper right', fontsize=9,
                   handles=[
                       plt.Line2D([0], [0], color='red', linestyle='--', linewidth=1.5),
                       plt.Line2D([0], [0], color='green', linestyle='--', linewidth=1.5),
                   ])

        plt.tight_layout(rect=[0, 0, 1, 0.94])
        ts_str = _time.strftime('%Y%m%d_%H%M%S')
        filename = f'left_hip_switch_{ts_str}.png'
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f'Plot saved: {filename}')

        self._print_data(sw, ts, alphas, omegas, woffs, wx1s, wx2s, wx3s,
                         wy1s, wy2s, wy3s, thetas, outs)

    def _print_data(self, sw, ts, alphas, omegas, woffs, wx1s, wx2s, wx3s,
                    wy1s, wy2s, wy3s, thetas, outs):
        print('\n' + '=' * 120)
        print(f'Left Hip (left_hip3)  switch data  '
              f't={sw["start"]:.3f}s -> {sw["end"]:.3f}s')
        print('=' * 120)
        header = (f'{"t(s)":>8} {"alpha":>7} {"omega":>7} '
                  f'{"w_off":>8} {"wx[1]":>8} {"wx[2]":>8} {"wx[3]":>8} '
                  f'{"wy[1]":>8} {"wy[2]":>8} {"wy[3]":>8} '
                  f'{"theta":>8} {"output":>8} {"clmp":>5}')
        print(header)
        print('-' * 120)

        n = len(ts)
        sw_s, sw_e = sw['start'], sw['end']
        pre  = [i for i in range(n) if ts[i] < sw_s]
        mid  = [i for i in range(n) if sw_s <= ts[i] <= sw_e]
        post = [i for i in range(n) if ts[i] > sw_e]

        def sample(lst, k):
            if len(lst) <= k:
                return lst
            step = max(1, len(lst) // k)
            return lst[::step]

        indices = sample(pre, 8) + sample(mid, 15) + sample(post, 8)

        for i in indices:
            clmp = 'Y' if abs(thetas[i] - outs[i]) > 1e-6 else ''
            print(f'{ts[i]:8.3f} {alphas[i]:7.4f} {omegas[i]:7.3f} '
                  f'{woffs[i]:8.4f} {wx1s[i]:8.4f} {wx2s[i]:8.4f} {wx3s[i]:8.4f} '
                  f'{wy1s[i]:8.4f} {wy2s[i]:8.4f} {wy3s[i]:8.4f} '
                  f'{np.degrees(thetas[i]):8.3f} {np.degrees(outs[i]):8.3f} {clmp:>5}')
        print('=' * 120)


def main():
    rclpy.init()
    node = GaitSwitchPlotter()
    print('\n' + '=' * 60)
    print('Left Hip Gait Switch Parameter Recorder')
    print('=' * 60)
    print('Subscribes: /cpg/switch_debug, /cpg/state')
    print('Auto-plots on switch complete (1s delay)')
    print('=' * 60 + '\n')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nExit')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
