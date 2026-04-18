#!/usr/bin/env python3
"""Swarm coordinator: orchestrates fleet mission lifecycle.

Manages robot roster, issues mission start/stop/return-home commands,
tracks overall fleet status (coverage, objects found), and publishes
/fleet/status at 1 Hz.
"""
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from af_msgs.msg import FleetStatus, DetectedObject


class SwarmCoordinatorNode(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')

        self.declare_parameter('num_robots', 3)
        self.declare_parameter('robot_prefix', 'robot_')
        self.declare_parameter('objects_target', 5)
        self.declare_parameter('coverage_target', 90.0)
        self.declare_parameter('mission_timeout_s', 300.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_tolerance', 0.5)

        self._num_robots = self.get_parameter('num_robots').value
        self._prefix = self.get_parameter('robot_prefix').value
        self._objects_target = self.get_parameter('objects_target').value
        self._coverage_target = self.get_parameter('coverage_target').value
        self._timeout = self.get_parameter('mission_timeout_s').value
        self._home_x = self.get_parameter('home_x').value
        self._home_y = self.get_parameter('home_y').value
        self._home_tol = self.get_parameter('home_tolerance').value

        self._robot_ids = [f'{self._prefix}{i}' for i in range(self._num_robots)]
        self._mission_state = 'idle'
        self._start_time = None
        self._objects_found = 0
        self._coverage = 0.0
        self._robot_poses: dict[str, tuple[float, float]] = {}

        self._fleet_pub = self.create_publisher(
            FleetStatus, '/fleet/status', 10)
        cmd_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._command_pub = self.create_publisher(
            String, '/fleet/command', cmd_qos)

        self._obj_sub = self.create_subscription(
            DetectedObject, '/fleet/objects', self._on_object, 10)

        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_subs = {}
        self._maps: dict[str, OccupancyGrid] = {}
        for rid in self._robot_ids:
            self._map_subs[rid] = self.create_subscription(
                OccupancyGrid, f'/{rid}/map', self._make_map_cb(rid),
                transient)
            self.create_subscription(
                Odometry, f'/{rid}/odom',
                lambda msg, r=rid: self._on_robot_odom(msg, r), 10)

        self.create_timer(1.0, self._publish_status)
        self.create_timer(2.0, self._check_mission)

        self.get_logger().info(
            f'Swarm coordinator ready: {self._num_robots} robots, '
            f'{self._objects_target} target objects')

    def start_mission(self):
        self._mission_state = 'exploring'
        self._start_time = time.time()
        self._objects_found = 0
        self._robot_poses.clear()
        msg = String()
        msg.data = 'start_explore'
        self._command_pub.publish(msg)
        self.get_logger().info('Mission started: explore and find objects')

    def _on_object(self, msg: DetectedObject):
        if self._mission_state not in ('exploring',):
            return
        self._objects_found += 1
        self.get_logger().info(
            f'Object #{self._objects_found} found: {msg.object_class} '
            f'by {msg.detected_by} at ({msg.position.x:.2f}, {msg.position.y:.2f})')

        if self._objects_found >= self._objects_target:
            self._trigger_return_home()

    def _on_robot_odom(self, msg: Odometry, robot_id: str):
        self._robot_poses[robot_id] = (
            msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _make_map_cb(self, robot_id):
        def _cb(msg: OccupancyGrid):
            self._maps[robot_id] = msg
            self._update_coverage()
        return _cb

    def _update_coverage(self):
        if not self._maps:
            return

        ref = list(self._maps.values())[0]
        w = ref.info.width
        h = ref.info.height
        res = ref.info.resolution
        ox = ref.info.origin.position.x
        oy = ref.info.origin.position.y

        merged = np.full(w * h, -1, dtype=np.int8)

        for grid in self._maps.values():
            if grid.info.width != w or grid.info.height != h:
                continue
            data = np.array(grid.data, dtype=np.int8)
            known = data >= 0
            merged[known] = np.maximum(merged[known], data[known])

        total = len(merged)
        if total == 0:
            return
        known_count = int(np.sum(merged >= 0))
        self._coverage = (known_count / total) * 100.0

    def _trigger_return_home(self):
        self._mission_state = 'returning'
        msg = String()
        msg.data = 'return_home'
        self._command_pub.publish(msg)
        self.get_logger().info(
            f'All {self._objects_target} objects found — returning home')

    def _check_mission(self):
        if self._mission_state == 'idle':
            return

        elapsed = time.time() - self._start_time if self._start_time else 0

        if self._mission_state == 'exploring':
            if self._coverage >= self._coverage_target:
                self.get_logger().info(
                    f'Coverage target {self._coverage_target}% reached')
                self._trigger_return_home()
            elif elapsed > self._timeout:
                self.get_logger().warn('Mission timeout — returning home')
                self._trigger_return_home()

        elif self._mission_state == 'returning':
            if self._all_robots_home():
                self._mission_state = 'done'
                elapsed = time.time() - self._start_time if self._start_time else 0
                self.get_logger().info(
                    f'Mission DONE — {self._objects_found}/{self._objects_target} '
                    f'objects, {self._coverage:.1f}% coverage, '
                    f'{elapsed:.0f}s elapsed')

    def _all_robots_home(self):
        if len(self._robot_poses) < self._num_robots:
            return False
        for rid in self._robot_ids:
            if rid not in self._robot_poses:
                return False
            px, py = self._robot_poses[rid]
            dist = math.hypot(px - self._home_x, py - self._home_y)
            if dist > self._home_tol:
                return False
        return True

    def _publish_status(self):
        msg = FleetStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mission_state = self._mission_state
        msg.robot_count = self._num_robots
        msg.objects_found = self._objects_found
        msg.objects_target = self._objects_target
        msg.coverage_percent = self._coverage
        msg.elapsed_s = (
            float(time.time() - self._start_time) if self._start_time else 0.0)
        msg.active_robots = self._robot_ids
        self._fleet_pub.publish(msg)


def main():
    rclpy.init()
    node = SwarmCoordinatorNode()
    node.start_mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
