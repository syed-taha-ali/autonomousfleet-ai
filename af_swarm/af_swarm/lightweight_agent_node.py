#!/usr/bin/env python3
"""Lightweight 2D kinematic agent for Tier 3 swarm simulation.

Each instance simulates one robot as a point mass with simple kinematics:
  - Subscribes to cmd_vel, integrates position at sim_rate Hz
  - Publishes odom (Odometry) with the updated pose
  - No physics, no Gazebo — pure Python + ROS 2

Designed to run 100-200 instances on a single machine (~5-15 MB each).

Can also run in batch mode: a single node managing N agents internally
(see LightweightFleetNode below), which is more efficient for 200+ agents.
"""
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


class LightweightAgentNode(Node):
    def __init__(self):
        super().__init__('lightweight_agent')

        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('sim_rate', 20.0)
        self.declare_parameter('publish_tf', True)

        self._x = self.get_parameter('init_x').value
        self._y = self.get_parameter('init_y').value
        self._yaw = self.get_parameter('init_yaw').value
        self._publish_tf = self.get_parameter('publish_tf').value

        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd, 10)

        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        rate = self.get_parameter('sim_rate').value
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._step)

        ns = self.get_namespace().strip('/')
        self.get_logger().info(
            f'Lightweight agent ready: ns={ns}, '
            f'pos=({self._x:.2f}, {self._y:.2f})')

    def _on_cmd(self, msg: Twist):
        self._vx = msg.linear.x
        self._vy = msg.linear.y
        self._vyaw = msg.angular.z

    def _step(self):
        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)
        self._x += (self._vx * cos_y - self._vy * sin_y) * self._dt
        self._y += (self._vx * sin_y + self._vy * cos_y) * self._dt
        self._yaw += self._vyaw * self._dt

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = _yaw_to_quat(self._yaw)
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._vyaw
        self._odom_pub.publish(odom)

        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation = _yaw_to_quat(self._yaw)
            self._tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = LightweightAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
