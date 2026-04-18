#!/usr/bin/env python3
"""Batch lightweight simulator: one process manages N agents.

Instead of spawning N separate processes (each ~80 MB), this node runs
all agents in a single process using numpy vectorised kinematics. Each
agent's odom is published on /<prefix><i>/odom and cmd_vel is received
on /<prefix><i>/cmd_vel. Flocking logic is also integrated — no
separate flocking node needed.

Memory: ~100 MB base + ~0.5 MB per agent (numpy arrays, no per-agent rclpy overhead).
CPU: O(N^2) neighbor search per tick — acceptable up to ~500 agents at 10 Hz.
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = float(math.cos(yaw / 2.0))
    q.z = float(math.sin(yaw / 2.0))
    return q


class BatchSimulatorNode(Node):
    def __init__(self):
        super().__init__('batch_simulator')

        self.declare_parameter('num_agents', 100)
        self.declare_parameter('robot_prefix', 'robot_')
        self.declare_parameter('spawn_radius', 5.0)
        self.declare_parameter('sim_rate', 10.0)
        self.declare_parameter('enable_flocking', True)
        self.declare_parameter('separation_radius', 0.5)
        self.declare_parameter('alignment_radius', 2.0)
        self.declare_parameter('cohesion_radius', 3.0)
        self.declare_parameter('separation_weight', 2.0)
        self.declare_parameter('alignment_weight', 1.0)
        self.declare_parameter('cohesion_weight', 1.0)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('migration_weight', 0.3)
        self.declare_parameter('migration_angle_deg', 0.0)
        self.declare_parameter('boundary_radius', 15.0)
        self.declare_parameter('boundary_weight', 2.0)

        self._n = self.get_parameter('num_agents').value
        prefix = self.get_parameter('robot_prefix').value
        spawn_r = self.get_parameter('spawn_radius').value
        rate = self.get_parameter('sim_rate').value
        self._flocking = self.get_parameter('enable_flocking').value
        self._sep_r = self.get_parameter('separation_radius').value
        self._align_r = self.get_parameter('alignment_radius').value
        self._coh_r = self.get_parameter('cohesion_radius').value
        self._sep_w = self.get_parameter('separation_weight').value
        self._align_w = self.get_parameter('alignment_weight').value
        self._coh_w = self.get_parameter('cohesion_weight').value
        self._max_speed = self.get_parameter('max_speed').value
        self._mig_w = self.get_parameter('migration_weight').value
        mig_angle = math.radians(self.get_parameter('migration_angle_deg').value)
        self._mig_dx = math.cos(mig_angle)
        self._mig_dy = math.sin(mig_angle)
        self._bound_r = self.get_parameter('boundary_radius').value
        self._bound_w = self.get_parameter('boundary_weight').value
        self._dt = 1.0 / rate

        angles = np.linspace(0, 2 * np.pi, self._n, endpoint=False)
        self._x = spawn_r * np.cos(angles)
        self._y = spawn_r * np.sin(angles)
        self._yaw = angles + np.pi
        self._vx = np.zeros(self._n)
        self._vy = np.zeros(self._n)
        self._vyaw = np.zeros(self._n)

        self._cmd_vx = np.zeros(self._n)
        self._cmd_vy = np.zeros(self._n)
        self._cmd_vyaw = np.zeros(self._n)

        self._odom_pubs = []
        self._cmd_subs = []
        for i in range(self._n):
            ns = f'/{prefix}{i}'
            pub = self.create_publisher(Odometry, f'{ns}/odom', 10)
            self._odom_pubs.append(pub)
            sub = self.create_subscription(
                Twist, f'{ns}/cmd_vel',
                self._make_cmd_cb(i), 10)
            self._cmd_subs.append(sub)

        self._metrics_pub = self.create_publisher(
            Float64MultiArray, '/fleet/flocking_metrics', 10)

        self.create_timer(self._dt, self._step)
        self.create_timer(1.0, self._publish_metrics)

        self.get_logger().info(
            f'Batch simulator: {self._n} agents, '
            f'flocking={self._flocking}, rate={rate} Hz')

    def _make_cmd_cb(self, idx):
        def _cb(msg: Twist):
            self._cmd_vx[idx] = msg.linear.x
            self._cmd_vy[idx] = msg.linear.y
            self._cmd_vyaw[idx] = msg.angular.z
        return _cb

    def _step(self):
        if self._flocking:
            self._compute_flocking()
        else:
            self._vx = self._cmd_vx.copy()
            self._vy = self._cmd_vy.copy()
            self._vyaw = self._cmd_vyaw.copy()

        cos_y = np.cos(self._yaw)
        sin_y = np.sin(self._yaw)
        self._x += (self._vx * cos_y - self._vy * sin_y) * self._dt
        self._y += (self._vx * sin_y + self._vy * cos_y) * self._dt
        self._yaw += self._vyaw * self._dt

        now = self.get_clock().now().to_msg()
        for i in range(self._n):
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.position.x = float(self._x[i])
            odom.pose.pose.position.y = float(self._y[i])
            odom.pose.pose.orientation = _yaw_to_quat(float(self._yaw[i]))
            odom.twist.twist.linear.x = float(self._vx[i])
            odom.twist.twist.linear.y = float(self._vy[i])
            odom.twist.twist.angular.z = float(self._vyaw[i])
            self._odom_pubs[i].publish(odom)

    def _compute_flocking(self):
        vx = np.zeros(self._n)
        vy = np.zeros(self._n)

        for i in range(self._n):
            dx = self._x - self._x[i]
            dy = self._y - self._y[i]
            dist = np.sqrt(dx**2 + dy**2)
            dist[i] = 1e9

            # Separation
            sep_mask = dist < self._sep_r
            if np.any(sep_mask):
                safe_dist = np.maximum(dist[sep_mask], 0.01)
                vx[i] -= self._sep_w * np.sum(dx[sep_mask] / safe_dist) / np.sum(sep_mask)
                vy[i] -= self._sep_w * np.sum(dy[sep_mask] / safe_dist) / np.sum(sep_mask)

            # Alignment
            align_mask = dist < self._align_r
            if np.any(align_mask):
                vx[i] += self._align_w * np.mean(self._vx[align_mask])
                vy[i] += self._align_w * np.mean(self._vy[align_mask])

            # Cohesion
            coh_mask = dist < self._coh_r
            if np.any(coh_mask):
                cx = np.mean(self._x[coh_mask]) - self._x[i]
                cy = np.mean(self._y[coh_mask]) - self._y[i]
                vx[i] += self._coh_w * cx
                vy[i] += self._coh_w * cy

            # Migration — constant directional bias
            vx[i] += self._mig_w * self._mig_dx
            vy[i] += self._mig_w * self._mig_dy

            # Boundary — soft repulsion to keep flock in a region
            agent_dist = math.sqrt(self._x[i]**2 + self._y[i]**2)
            if agent_dist > self._bound_r:
                overshoot = (agent_dist - self._bound_r) / self._bound_r
                vx[i] -= self._bound_w * overshoot * self._x[i] / max(agent_dist, 0.01)
                vy[i] -= self._bound_w * overshoot * self._y[i] / max(agent_dist, 0.01)

        speeds = np.sqrt(vx**2 + vy**2)
        too_fast = speeds > self._max_speed
        if np.any(too_fast):
            scale = self._max_speed / np.maximum(speeds[too_fast], 1e-6)
            vx[too_fast] *= scale
            vy[too_fast] *= scale

        self._vx = vx
        self._vy = vy
        self._vyaw[:] = 0.0

    def _publish_metrics(self):
        speeds = np.sqrt(self._vx**2 + self._vy**2)
        avg_speed = np.mean(speeds)

        if avg_speed < 0.001:
            order = 0.0
        else:
            sum_vx = np.sum(self._vx)
            sum_vy = np.sum(self._vy)
            order = float(np.sqrt(sum_vx**2 + sum_vy**2) / (self._n * avg_speed))

        cx = np.mean(self._x)
        cy = np.mean(self._y)
        cohesion = float(np.mean(np.sqrt((self._x - cx)**2 + (self._y - cy)**2)))

        collisions = 0
        for i in range(self._n):
            dx = self._x[i+1:] - self._x[i]
            dy = self._y[i+1:] - self._y[i]
            dist = np.sqrt(dx**2 + dy**2)
            collisions += int(np.sum(dist < 0.15))

        msg = Float64MultiArray()
        msg.data = [order, cohesion, float(self._n), float(collisions), avg_speed]
        self._metrics_pub.publish(msg)

        self.get_logger().info(
            f'Flock: order={order:.3f}, cohesion={cohesion:.2f}m, '
            f'collisions={collisions}, avg_speed={avg_speed:.3f} m/s')


def main():
    rclpy.init()
    node = BatchSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
