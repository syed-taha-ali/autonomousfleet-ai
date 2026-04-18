#!/usr/bin/env python3
"""Reynolds flocking algorithm for Tier 3 lightweight agents.

Implements the three classic boid rules:
  1. Separation — repel neighbors within separation_radius
  2. Alignment — match velocity of neighbors within alignment_radius
  3. Cohesion  — steer toward center of mass within cohesion_radius

Subscribes to all agents' /robot_N/odom, computes the flocking velocity,
and publishes to /robot_N/cmd_vel. Runs one instance per agent.

Publishes /fleet/flocking_metrics at 1 Hz with order parameter and
cohesion index for benchmarking.
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class FlockingNode(Node):
    def __init__(self):
        super().__init__('flocking')

        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('num_agents', 100)
        self.declare_parameter('robot_prefix', 'robot_')
        self.declare_parameter('separation_radius', 0.5)
        self.declare_parameter('alignment_radius', 2.0)
        self.declare_parameter('cohesion_radius', 3.0)
        self.declare_parameter('separation_weight', 2.0)
        self.declare_parameter('alignment_weight', 1.0)
        self.declare_parameter('cohesion_weight', 1.0)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('update_rate', 10.0)

        self._ns = self.get_parameter('robot_namespace').value or \
            self.get_namespace().strip('/')
        num = self.get_parameter('num_agents').value
        prefix = self.get_parameter('robot_prefix').value
        self._sep_r = self.get_parameter('separation_radius').value
        self._align_r = self.get_parameter('alignment_radius').value
        self._coh_r = self.get_parameter('cohesion_radius').value
        self._sep_w = self.get_parameter('separation_weight').value
        self._align_w = self.get_parameter('alignment_weight').value
        self._coh_w = self.get_parameter('cohesion_weight').value
        self._max_speed = self.get_parameter('max_speed').value

        self._poses: dict[str, tuple[float, float]] = {}
        self._velocities: dict[str, tuple[float, float]] = {}

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        for i in range(num):
            rid = f'{prefix}{i}'
            self.create_subscription(
                Odometry, f'/{rid}/odom',
                self._make_odom_cb(rid), 10)

        rate = self.get_parameter('update_rate').value
        self.create_timer(1.0 / rate, self._update)

        self._metrics_pub = self.create_publisher(
            Float64MultiArray, '/fleet/flocking_metrics', 10)
        self.create_timer(1.0, self._publish_metrics)

        self.get_logger().info(
            f'Flocking node ready: ns={self._ns}, agents={num}')

    def _make_odom_cb(self, robot_id):
        def _cb(msg: Odometry):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            self._poses[robot_id] = (x, y)
            self._velocities[robot_id] = (vx, vy)
        return _cb

    def _update(self):
        if self._ns not in self._poses:
            return

        mx, my = self._poses[self._ns]
        sep_x, sep_y = 0.0, 0.0
        align_x, align_y = 0.0, 0.0
        coh_x, coh_y = 0.0, 0.0
        n_sep = 0
        n_align = 0
        n_coh = 0

        for rid, (ox, oy) in self._poses.items():
            if rid == self._ns:
                continue
            dx = ox - mx
            dy = oy - my
            dist = math.hypot(dx, dy)

            if dist < self._sep_r and dist > 0.01:
                sep_x -= dx / dist
                sep_y -= dy / dist
                n_sep += 1

            if dist < self._align_r:
                vx, vy = self._velocities.get(rid, (0.0, 0.0))
                align_x += vx
                align_y += vy
                n_align += 1

            if dist < self._coh_r:
                coh_x += ox
                coh_y += oy
                n_coh += 1

        vx, vy = 0.0, 0.0

        if n_sep > 0:
            vx += self._sep_w * sep_x / n_sep
            vy += self._sep_w * sep_y / n_sep

        if n_align > 0:
            vx += self._align_w * align_x / n_align
            vy += self._align_w * align_y / n_align

        if n_coh > 0:
            cx = coh_x / n_coh - mx
            cy = coh_y / n_coh - my
            vx += self._coh_w * cx
            vy += self._coh_w * cy

        speed = math.hypot(vx, vy)
        if speed > self._max_speed:
            vx = vx / speed * self._max_speed
            vy = vy / speed * self._max_speed

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        self._cmd_pub.publish(cmd)

    def _publish_metrics(self):
        if len(self._velocities) < 2:
            return

        vels = list(self._velocities.values())
        speeds = [math.hypot(vx, vy) for vx, vy in vels]
        avg_speed = sum(speeds) / len(speeds) if speeds else 0.0

        if avg_speed < 0.01:
            order = 0.0
        else:
            sum_vx = sum(v[0] for v in vels)
            sum_vy = sum(v[1] for v in vels)
            order = math.hypot(sum_vx, sum_vy) / (len(vels) * avg_speed)

        poses = list(self._poses.values())
        cx = sum(p[0] for p in poses) / len(poses)
        cy = sum(p[1] for p in poses) / len(poses)
        cohesion = sum(math.hypot(p[0] - cx, p[1] - cy) for p in poses) / len(poses)

        msg = Float64MultiArray()
        msg.data = [order, cohesion, float(len(self._poses))]
        self._metrics_pub.publish(msg)


def main():
    rclpy.init()
    node = FlockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
