#!/usr/bin/env python3
"""Flocking benchmark: measure order, cohesion, collisions across agent counts.

Launches batch_simulator_node for each agent count, records metrics for
a fixed duration, saves CSV and generates plots.

Usage:
  python3 run_flocking_benchmark.py
  python3 run_flocking_benchmark.py --counts 25 50 100 200 --duration 60
"""
import argparse
import csv
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class MetricCollector(Node):
    def __init__(self):
        super().__init__('flocking_benchmark')
        self.samples = []
        self._t0 = time.time()
        self.create_subscription(
            Float64MultiArray, '/fleet/flocking_metrics',
            self._on_metrics, 10)

    def _on_metrics(self, msg):
        if len(msg.data) >= 5:
            self.samples.append({
                't': time.time() - self._t0,
                'order': msg.data[0],
                'cohesion': msg.data[1],
                'num_agents': int(msg.data[2]),
                'collisions': int(msg.data[3]),
                'avg_speed': msg.data[4],
            })

    def reset(self):
        self.samples.clear()
        self._t0 = time.time()


def run_trial(collector, num_agents, duration, ros_setup):
    cmd = (
        f'source {ros_setup} && '
        f'ros2 launch af_swarm flocking_demo.launch.py '
        f'num_agents:={num_agents}'
    )
    proc = subprocess.Popen(
        ['bash', '-c', cmd],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)

    collector.reset()
    t0 = time.time()

    print(f'  Collecting for {duration}s...', flush=True)
    while time.time() - t0 < duration:
        rclpy.spin_once(collector, timeout_sec=0.5)

    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    proc.wait(timeout=10)
    time.sleep(2)

    return list(collector.samples)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--counts', nargs='+', type=int,
                        default=[25, 50, 100, 150, 200])
    parser.add_argument('--duration', type=int, default=60)
    parser.add_argument('--output-dir', default=None)
    args = parser.parse_args()

    ws = os.path.expanduser('~/ros2_ws')
    ros_setup = f'{ws}/install/setup.bash'
    if args.output_dir:
        out_dir = args.output_dir
    else:
        out_dir = os.path.join(ws, 'src/project/logs/phase6/benchmarks')
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    collector = MetricCollector()

    all_results = {}

    for n in args.counts:
        print(f'\n=== Flocking benchmark: {n} agents ===', flush=True)
        samples = run_trial(collector, n, args.duration, ros_setup)
        all_results[n] = samples
        print(f'  Collected {len(samples)} samples')

        if samples:
            orders = [s['order'] for s in samples]
            colls = [s['collisions'] for s in samples]
            speeds = [s['avg_speed'] for s in samples]
            print(f'  Order: {sum(orders)/len(orders):.3f} '
                  f'(peak {max(orders):.3f})')
            print(f'  Collisions: {sum(colls)/len(colls):.1f} avg')
            print(f'  Speed: {sum(speeds)/len(speeds):.3f} avg')

    csv_path = os.path.join(out_dir, 'flocking_benchmark.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(
            f, fieldnames=['num_agents', 't', 'order', 'cohesion',
                           'collisions', 'avg_speed'])
        writer.writeheader()
        for n, samples in all_results.items():
            for s in samples:
                writer.writerow({
                    'num_agents': n,
                    't': f'{s["t"]:.2f}',
                    'order': f'{s["order"]:.4f}',
                    'cohesion': f'{s["cohesion"]:.3f}',
                    'collisions': s['collisions'],
                    'avg_speed': f'{s["avg_speed"]:.4f}',
                })
    print(f'\nCSV saved: {csv_path}')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Flocking Benchmark Results', fontsize=14)

        for n, samples in all_results.items():
            if not samples:
                continue
            t = [s['t'] for s in samples]
            axes[0, 0].plot(t, [s['order'] for s in samples],
                            label=f'N={n}', alpha=0.8)
            axes[0, 1].plot(t, [s['cohesion'] for s in samples],
                            label=f'N={n}', alpha=0.8)

        axes[0, 0].set_ylabel('Order Parameter')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].legend()
        axes[0, 0].set_ylim(0, 1)
        axes[0, 0].axhline(y=0.8, color='r', linestyle='--', alpha=0.5)
        axes[0, 1].set_ylabel('Cohesion (m)')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].legend()

        ns = sorted(all_results.keys())
        steady_order = []
        steady_coll = []
        for n in ns:
            samples = all_results[n]
            if not samples:
                steady_order.append(0)
                steady_coll.append(0)
                continue
            late = [s for s in samples if s['t'] > 20]
            if late:
                steady_order.append(
                    sum(s['order'] for s in late) / len(late))
                steady_coll.append(
                    sum(s['collisions'] for s in late) / len(late))
            else:
                steady_order.append(0)
                steady_coll.append(0)

        axes[1, 0].bar([str(n) for n in ns], steady_order, color='#2ecc71')
        axes[1, 0].set_ylabel('Steady-State Order')
        axes[1, 0].set_xlabel('Num Agents')
        axes[1, 0].axhline(y=0.8, color='r', linestyle='--', alpha=0.5)
        axes[1, 0].set_ylim(0, 1)

        axes[1, 1].bar([str(n) for n in ns], steady_coll, color='#e74c3c')
        axes[1, 1].set_ylabel('Steady-State Collisions')
        axes[1, 1].set_xlabel('Num Agents')

        plt.tight_layout()
        plot_path = os.path.join(out_dir, 'flocking_benchmark.png')
        plt.savefig(plot_path, dpi=150)
        print(f'Plot saved: {plot_path}')
    except ImportError:
        print('matplotlib not available — skipping plots')

    collector.destroy_node()
    rclpy.shutdown()
    print('\nFlocking benchmark complete.')


if __name__ == '__main__':
    main()
