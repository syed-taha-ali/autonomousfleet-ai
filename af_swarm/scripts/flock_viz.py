#!/usr/bin/env python3
"""Live matplotlib visualization of the batch flocking simulator."""
import sys
import threading

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class FlockViz(Node):
    def __init__(self, num_agents=100, prefix='robot_'):
        super().__init__('flock_viz')
        self._n = num_agents
        self._x = np.zeros(num_agents)
        self._y = np.zeros(num_agents)
        self._vx = np.zeros(num_agents)
        self._vy = np.zeros(num_agents)
        self._order = 0.0
        self._cohesion = 0.0
        self._collisions = 0
        self._speed = 0.0

        for i in range(num_agents):
            self.create_subscription(
                Odometry, f'/{prefix}{i}/odom',
                lambda msg, idx=i: self._on_odom(msg, idx), 10)

        self.create_subscription(
            Float64MultiArray, '/fleet/flocking_metrics',
            self._on_metrics, 10)

    def _on_odom(self, msg, idx):
        self._x[idx] = msg.pose.pose.position.x
        self._y[idx] = msg.pose.pose.position.y
        self._vx[idx] = msg.twist.twist.linear.x
        self._vy[idx] = msg.twist.twist.linear.y

    def _on_metrics(self, msg):
        if len(msg.data) >= 5:
            self._order = msg.data[0]
            self._cohesion = msg.data[1]
            self._collisions = int(msg.data[3])
            self._speed = msg.data[4]


def main():
    num = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    prefix = sys.argv[2] if len(sys.argv) > 2 else 'robot_'

    rclpy.init()
    node = FlockViz(num, prefix)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(9, 9))
    fig.patch.set_facecolor('#1a1a2e')
    ax.set_facecolor('#16213e')
    ax.set_aspect('equal')
    ax.set_title('Reynolds Flocking — 100 Agents', color='white', fontsize=14)
    ax.tick_params(colors='#888888')
    for spine in ax.spines.values():
        spine.set_color('#333333')

    scatter = ax.scatter(np.zeros(num), np.zeros(num), s=18, c='#00ff88',
                         alpha=0.9, edgecolors='none')
    quiver = ax.quiver(np.zeros(num), np.zeros(num),
                       np.zeros(num), np.zeros(num),
                       color='#ff6b6b', alpha=0.6, scale=3.0, width=0.003)
    info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                        color='white', fontsize=11, verticalalignment='top',
                        fontfamily='monospace',
                        bbox=dict(boxstyle='round,pad=0.4',
                                  facecolor='#0f3460', alpha=0.8))

    def update(_frame):
        x = node._x.copy()
        y = node._y.copy()
        vx = node._vx.copy()
        vy = node._vy.copy()

        scatter.set_offsets(np.column_stack([x, y]))

        quiver.set_offsets(np.column_stack([x, y]))
        quiver.set_UVC(vx, vy)

        margin = 3.0
        cx, cy = np.mean(x), np.mean(y)
        spread = max(np.ptp(x), np.ptp(y), 5.0) / 2 + margin
        ax.set_xlim(cx - spread, cx + spread)
        ax.set_ylim(cy - spread, cy + spread)

        info_text.set_text(
            f'Order:      {node._order:.3f}\n'
            f'Cohesion:   {node._cohesion:.2f} m\n'
            f'Collisions: {node._collisions}\n'
            f'Avg Speed:  {node._speed:.3f} m/s')

        return scatter, quiver, info_text

    ani = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
