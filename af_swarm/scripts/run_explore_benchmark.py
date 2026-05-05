#!/usr/bin/env python3
"""Exploration benchmark: measure time-to-complete, coverage, objects found.

For each robot count, launches MVSim + explore_demo, subscribes to
/fleet/status, records time-series to CSV, and generates plots.

Usage:
  python3 run_explore_benchmark.py --counts 1 2 3 --timeout 300
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
from af_msgs.msg import FleetStatus


class StatusCollector(Node):
    def __init__(self):
        super().__init__('explore_benchmark')
        self.samples = []
        self._t0 = time.time()
        self._latest = None
        self.create_subscription(
            FleetStatus, '/fleet/status', self._on_status, 10)

    def _on_status(self, msg):
        # Reject stale messages from a prior trial's orphan coordinator:
        # FleetStatus reports elapsed_s counted inside the coordinator, so
        # a fresh trial must have elapsed_s roughly aligned with wall time
        # since reset. A message whose elapsed_s leads wall-clock by >30 s
        # is from a leaked publisher and must be ignored.
        wall_since_reset = time.time() - self._t0
        if msg.elapsed_s > wall_since_reset + 30.0:
            return
        self._latest = msg
        self.samples.append({
            't': time.time() - self._t0,
            'elapsed': msg.elapsed_s,
            'state': msg.mission_state,
            'objects': msg.objects_found,
            'target': msg.objects_target,
            'coverage': msg.coverage_percent,
        })

    def reset(self):
        self.samples.clear()
        self._latest = None
        self._t0 = time.time()


def run_trial(collector, num_robots, timeout, ros_setup):
    # Kill any stragglers from a previous trial — an orphan swarm_coordinator
    # republishing 'done' over TRANSIENT_LOCAL QoS will make the new trial
    # break out immediately on the latched stale sample.
    subprocess.run(
        ['bash', '-c',
         "pkill -9 -f swarm_coordinator; pkill -9 -f object_registry; "
         "pkill -9 -f distributed_explore; pkill -9 -f ground_truth; "
         "pkill -9 -f mvsim_node; pkill -9 -f slam_toolbox; "
         "pkill -9 -f controller_server; pkill -9 -f planner_server; "
         "pkill -9 -f bt_navigator; pkill -9 -f behavior_server; "
         "pkill -9 -f lifecycle_manager; pkill -9 -f amcl; "
         "pkill -9 -f map_server; pkill -9 -f velocity_smoother; "
         "pkill -9 -f waypoint_follower; pkill -9 -f smoother_server; "
         "sleep 2"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    sim_cmd = (
        f'source {ros_setup} && '
        f'ros2 launch af_sim simulation_mvsim.launch.py '
        f'num_robots:={num_robots}'
    )
    sim_proc = subprocess.Popen(
        ['bash', '-c', sim_cmd],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)

    time.sleep(30)

    swarm_cmd = (
        f'source {ros_setup} && '
        f'ros2 launch af_swarm explore_demo.launch.py '
        f'num_robots:={num_robots}'
    )
    swarm_proc = subprocess.Popen(
        ['bash', '-c', swarm_cmd],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)

    collector.reset()
    t0 = time.time()
    done = False

    print(f'  Running for up to {timeout}s...', flush=True)
    while time.time() - t0 < timeout + 60:
        rclpy.spin_once(collector, timeout_sec=1.0)
        if collector._latest and collector._latest.mission_state in ('done', 'returning'):
            if collector._latest.mission_state == 'done':
                print('  Mission completed: done', flush=True)
                done = True
                break
            if collector._latest.elapsed_s > timeout:
                print('  Timeout reached', flush=True)
                break

    try:
        os.killpg(os.getpgid(swarm_proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    swarm_proc.wait(timeout=10)
    sim_proc.wait(timeout=10)
    time.sleep(5)

    return list(collector.samples), done


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--counts', nargs='+', type=int, default=[1, 2, 3])
    parser.add_argument('--timeout', type=int, default=300)
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
    collector = StatusCollector()

    all_results = {}

    for n in args.counts:
        print(f'\n=== Exploration benchmark: {n} robot(s) ===', flush=True)
        samples, done = run_trial(collector, n, args.timeout, ros_setup)
        all_results[n] = samples
        print(f'  Collected {len(samples)} samples')

        if samples:
            last = samples[-1]
            print(f'  Objects: {last["objects"]}/{last["target"]}')
            print(f'  Coverage: {last["coverage"]:.1f}%')
            print(f'  Elapsed: {last["elapsed"]:.0f}s')
            print(f'  Final state: {last["state"]}')

    csv_path = os.path.join(out_dir, 'explore_benchmark.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(
            f, fieldnames=['num_robots', 't', 'elapsed', 'state',
                           'objects', 'target', 'coverage'])
        writer.writeheader()
        for n, samples in all_results.items():
            for s in samples:
                writer.writerow({
                    'num_robots': n,
                    't': f'{s["t"]:.2f}',
                    'elapsed': f'{s["elapsed"]:.2f}',
                    'state': s['state'],
                    'objects': s['objects'],
                    'target': s['target'],
                    'coverage': f'{s["coverage"]:.2f}',
                })
    print(f'\nCSV saved: {csv_path}')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig.suptitle('Exploration Benchmark Results', fontsize=14)

        for n, samples in all_results.items():
            if not samples:
                continue
            t = [s['elapsed'] for s in samples]
            axes[0].plot(t, [s['coverage'] for s in samples],
                         label=f'N={n}', alpha=0.8)
            axes[1].plot(t, [s['objects'] for s in samples],
                         label=f'N={n}', alpha=0.8)

        axes[0].set_ylabel('Coverage (%)')
        axes[0].set_xlabel('Elapsed (s)')
        axes[0].legend()
        axes[1].set_ylabel('Objects Found')
        axes[1].set_xlabel('Elapsed (s)')
        axes[1].legend()

        ns = sorted(all_results.keys())
        final_cov = []
        final_obj = []
        for n in ns:
            samples = all_results[n]
            if samples:
                final_cov.append(samples[-1]['coverage'])
                final_obj.append(samples[-1]['objects'])
            else:
                final_cov.append(0)
                final_obj.append(0)

        x = [str(n) for n in ns]
        axes[2].bar(x, final_obj, color='#3498db', label='Objects')
        ax2 = axes[2].twinx()
        ax2.plot(x, final_cov, 'ro-', label='Coverage %')
        axes[2].set_ylabel('Objects Found')
        ax2.set_ylabel('Coverage (%)')
        axes[2].set_xlabel('Num Robots')
        axes[2].legend(loc='upper left')
        ax2.legend(loc='upper right')

        plt.tight_layout()
        plot_path = os.path.join(out_dir, 'explore_benchmark.png')
        plt.savefig(plot_path, dpi=150)
        print(f'Plot saved: {plot_path}')
    except ImportError:
        print('matplotlib not available — skipping plots')

    collector.destroy_node()
    rclpy.shutdown()
    print('\nExploration benchmark complete.')


if __name__ == '__main__':
    main()
