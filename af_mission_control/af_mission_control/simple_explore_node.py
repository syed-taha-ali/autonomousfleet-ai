#!/usr/bin/env python3
"""Minimal random-walk frontier explorer for single-room demos.

Reads the live /map occupancy grid, picks a random free cell near the frontier
(boundary between known-free and unknown), and sends it as a Nav2
navigate_to_pose goal. Loops until cancelled or no frontiers remain.

This is the fallback if m-explore-ros2 is unavailable. It is deliberately
simple — acceptable for a single small room; no publication claim rides on
exploration sophistication.
"""
import math
import random

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose


_FREE = 0
_UNKNOWN = -1
_MIN_FRONTIER_DIST = 0.5  # metres from robot


class SimpleExploreNode(Node):
    def __init__(self):
        super().__init__('simple_explore')
        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter('goal_tolerance_m', 0.3)
        self.declare_parameter('min_frontier_dist_m', 0.5)
        self.declare_parameter('replan_interval_s', 2.0)

        self._goal_tolerance = self.get_parameter('goal_tolerance_m').value
        self._min_dist = self.get_parameter('min_frontier_dist_m').value

        self._map_data = None
        self._map_info = None
        self._nav_active = False
        self._nav_handle = None
        self._enabled = True

        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, 5,
            callback_group=self._cb_group,
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group,
        )

        replan_s = self.get_parameter('replan_interval_s').value
        self._timer = self.create_timer(replan_s, self._plan_next, callback_group=self._cb_group)

        self.get_logger().info('Simple explore node ready')

    def _on_map(self, msg: OccupancyGrid):
        self._map_info = msg.info
        self._map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

    def _plan_next(self):
        if not self._enabled or self._nav_active:
            return
        if self._map_data is None:
            return

        frontier_cells = self._find_frontier_cells()
        if not frontier_cells:
            self.get_logger().info('No frontiers remaining — exploration complete')
            self._enabled = False
            return

        target = random.choice(frontier_cells)
        wx, wy = self._grid_to_world(target[1], target[0])

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = wx
        goal_pose.pose.position.y = wy
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Exploring to ({wx:.2f}, {wy:.2f})')
        self._send_nav_goal(goal_pose)

    def _find_frontier_cells(self):
        """Find cells that are free and adjacent to at least one unknown cell."""
        data = self._map_data
        h, w = data.shape

        free_mask = data == _FREE
        unknown_mask = data == _UNKNOWN

        # Dilate unknown by 1 pixel in 4-connectivity
        unknown_adjacent = np.zeros_like(unknown_mask)
        unknown_adjacent[1:, :] |= unknown_mask[:-1, :]
        unknown_adjacent[:-1, :] |= unknown_mask[1:, :]
        unknown_adjacent[:, 1:] |= unknown_mask[:, :-1]
        unknown_adjacent[:, :-1] |= unknown_mask[:, 1:]

        frontier = free_mask & unknown_adjacent
        coords = np.argwhere(frontier)

        if len(coords) == 0:
            return []

        # Subsample to avoid choosing from thousands of cells
        if len(coords) > 200:
            indices = np.random.choice(len(coords), 200, replace=False)
            coords = coords[indices]

        return coords.tolist()

    def _grid_to_world(self, gx: int, gy: int):
        res = self._map_info.resolution
        ox = self._map_info.origin.position.x
        oy = self._map_info.origin.position.y
        return ox + (gx + 0.5) * res, oy + (gy + 0.5) * res

    def _send_nav_goal(self, pose: PoseStamped):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_active = True
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Explore goal rejected by Nav2')
            self._nav_active = False
            return
        self._nav_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_done)

    def _on_nav_done(self, future):
        self._nav_active = False
        self._nav_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = SimpleExploreNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
