#!/usr/bin/env python3
"""Minimal random-walk frontier explorer for single-room demos.

Reads the live /map occupancy grid, picks a random free cell near the frontier
(boundary between known-free and unknown), and sends it as a Nav2
navigate_to_pose goal. Loops until cancelled or an exit condition is met.

Exit conditions (all parameter-driven, 0/false = disabled):
  max_explore_time_s      — wall-clock limit from first goal sent
  max_explore_distance_m  — cumulative odometry distance
  stop_on_detection        — stop when target_class seen on /detections
  (frontier exhaustion)    — always active; stops when no frontiers remain
"""
import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray


_FREE = 0
_UNKNOWN = -1


class SimpleExploreNode(Node):
    def __init__(self):
        super().__init__('simple_explore')
        self._cb_group = ReentrantCallbackGroup()

        # Navigation parameters
        self.declare_parameter('goal_tolerance_m', 0.3)
        self.declare_parameter('min_frontier_dist_m', 0.5)
        self.declare_parameter('replan_interval_s', 2.0)
        self.declare_parameter('goal_timeout_s', 20.0)

        # Exit condition parameters
        self.declare_parameter('max_explore_time_s', 0.0)
        self.declare_parameter('max_explore_distance_m', 0.0)
        self.declare_parameter('stop_on_detection', False)
        self.declare_parameter('target_class', 'suitcase')
        self.declare_parameter('detection_confidence_min', 0.5)

        self._goal_tolerance = self.get_parameter('goal_tolerance_m').value
        self._min_dist = self.get_parameter('min_frontier_dist_m').value
        self._goal_timeout = self.get_parameter('goal_timeout_s').value

        self._max_time = self.get_parameter('max_explore_time_s').value
        self._max_distance = self.get_parameter('max_explore_distance_m').value
        self._stop_on_detection = self.get_parameter('stop_on_detection').value
        self._target_class = self.get_parameter('target_class').value
        self._det_conf_min = self.get_parameter('detection_confidence_min').value

        self.declare_parameter('start_enabled', True)

        self._map_data = None
        self._map_info = None
        self._nav_active = False
        self._nav_handle = None
        self._goal_send_time = 0.0
        self._enabled = self.get_parameter('start_enabled').value

        # Exit condition state
        self._first_goal_time = 0.0
        self._distance_travelled = 0.0
        self._last_odom_x = None
        self._last_odom_y = None
        self._detected = False
        self._exit_reason = ''

        # Subscriptions
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, 5,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Odometry, '/odom', self._on_odom, 10,
            callback_group=self._cb_group,
        )

        if self._stop_on_detection:
            det_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self.create_subscription(
                Detection2DArray, '/detections', self._on_detections, det_qos,
                callback_group=self._cb_group,
            )

        self.create_subscription(
            Bool, '/explore/enable', self._on_enable, 10,
            callback_group=self._cb_group,
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group,
        )

        replan_s = self.get_parameter('replan_interval_s').value
        self._timer = self.create_timer(replan_s, self._plan_next, callback_group=self._cb_group)

        self._log_exit_config()

    def _log_exit_config(self):
        conditions = []
        if self._max_time > 0:
            conditions.append(f'time={self._max_time}s')
        if self._max_distance > 0:
            conditions.append(f'distance={self._max_distance}m')
        if self._stop_on_detection:
            conditions.append(f'detection={self._target_class}@{self._det_conf_min}')
        conditions.append('frontier_exhaustion')
        self.get_logger().info(f'Explorer ready, exit conditions: {", ".join(conditions)}')

    # ── Subscriptions ──────────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid):
        self._map_info = msg.info
        self._map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_odom_x is not None:
            dx = x - self._last_odom_x
            dy = y - self._last_odom_y
            self._distance_travelled += math.sqrt(dx * dx + dy * dy)
        self._last_odom_x = x
        self._last_odom_y = y

    def _on_detections(self, msg: Detection2DArray):
        if not self._enabled or self._detected:
            return
        for det in msg.detections:
            for result in det.results:
                if (result.hypothesis.class_id == self._target_class
                        and result.hypothesis.score >= self._det_conf_min):
                    self._detected = True
                    return

    def _on_enable(self, msg: Bool):
        if msg.data and not self._enabled:
            self._max_time = self.get_parameter('max_explore_time_s').value
            self._max_distance = self.get_parameter('max_explore_distance_m').value
            self._enabled = True
            self._first_goal_time = 0.0
            self._distance_travelled = 0.0
            self._detected = False
            self._exit_reason = ''
            self.get_logger().info(
                f'Explorer enabled (time={self._max_time}s, dist={self._max_distance}m)')
        elif not msg.data and self._enabled:
            self._stop('disabled via /explore/enable')

    # ── Planning loop ──────────────────────────────────────────────

    def _plan_next(self):
        if not self._enabled:
            return

        if self._check_exit_conditions():
            self._stop('exit condition met')
            return

        if self._nav_active:
            if time.monotonic() - self._goal_send_time > self._goal_timeout:
                self.get_logger().info('Goal timeout — cancelling and replanning')
                if self._nav_handle is not None:
                    self._nav_handle.cancel_goal_async()
                self._nav_active = False
                self._nav_handle = None
            else:
                return

        if self._map_data is None:
            return

        frontier_cells = self._find_frontier_cells()
        if not frontier_cells:
            self._stop('no frontiers remaining')
            return

        target = np.random.default_rng().choice(len(frontier_cells))
        cell = frontier_cells[target]
        wx, wy = self._grid_to_world(cell[1], cell[0])

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = wx
        goal_pose.pose.position.y = wy
        goal_pose.pose.orientation.w = 1.0

        if self._first_goal_time == 0.0:
            self._first_goal_time = time.monotonic()

        self.get_logger().info(f'Exploring to ({wx:.2f}, {wy:.2f})')
        self._send_nav_goal(goal_pose)

    def _check_exit_conditions(self) -> bool:
        if self._first_goal_time == 0.0:
            return False

        if self._max_time > 0:
            elapsed = time.monotonic() - self._first_goal_time
            if elapsed >= self._max_time:
                self._exit_reason = f'time_limit ({elapsed:.0f}s >= {self._max_time:.0f}s)'
                return True

        if self._max_distance > 0:
            if self._distance_travelled >= self._max_distance:
                self._exit_reason = (
                    f'distance_limit ({self._distance_travelled:.1f}m '
                    f'>= {self._max_distance:.1f}m)'
                )
                return True

        if self._stop_on_detection and self._detected:
            self._exit_reason = f'detected_{self._target_class}'
            return True

        return False

    def _stop(self, reason: str):
        self._enabled = False
        if self._nav_active and self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
            self._nav_active = False
            self._nav_handle = None
        self.get_logger().info(
            f'Exploration stopped: {reason} '
            f'(travelled {self._distance_travelled:.2f}m, '
            f'elapsed {time.monotonic() - self._first_goal_time:.0f}s)'
        )

    # ── Frontier detection ─────────────────────────────────────────

    def _find_frontier_cells(self):
        data = self._map_data
        free_mask = data == _FREE
        unknown_mask = data == _UNKNOWN

        unknown_adjacent = np.zeros_like(unknown_mask)
        unknown_adjacent[1:, :] |= unknown_mask[:-1, :]
        unknown_adjacent[:-1, :] |= unknown_mask[1:, :]
        unknown_adjacent[:, 1:] |= unknown_mask[:, :-1]
        unknown_adjacent[:, :-1] |= unknown_mask[:, 1:]

        frontier = free_mask & unknown_adjacent
        coords = np.argwhere(frontier)

        if len(coords) == 0:
            return []

        if len(coords) > 200:
            indices = np.random.choice(len(coords), 200, replace=False)
            coords = coords[indices]

        return coords.tolist()

    def _grid_to_world(self, gx: int, gy: int):
        res = self._map_info.resolution
        ox = self._map_info.origin.position.x
        oy = self._map_info.origin.position.y
        return ox + (gx + 0.5) * res, oy + (gy + 0.5) * res

    # ── Nav2 interface ─────────────────────────────────────────────

    def _send_nav_goal(self, pose: PoseStamped):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_active = True
        self._goal_send_time = time.monotonic()
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
