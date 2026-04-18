#!/usr/bin/env python3
"""Distributed frontier exploration with deconfliction.

Each robot:
1. Reads its own SLAM-generated /map
2. Extracts frontier cells (free cells adjacent to unknown)
3. Clusters frontiers into candidate regions
4. Scores each region: size * info_gain / distance
5. Penalises regions claimed by other robots (via /fleet/frontier_claims)
6. Claims the best region, publishes claim, sends Nav2 goal

When /fleet/command == 'return_home', cancels exploration and navigates
to the home position.
"""
import math
import random
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatus

from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from af_msgs.msg import FrontierClaim

_FREE = 0
_UNKNOWN = -1


class DistributedExploreNode(Node):
    def __init__(self):
        super().__init__('distributed_explore')

        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('claim_penalty', 0.3)
        self.declare_parameter('explore_rate_hz', 0.5)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('obstacle_clearance', 0.4)
        self.declare_parameter('num_robots', 3)
        self.declare_parameter('robot_prefix', 'robot_')
        self.declare_parameter('robot_repulsion_radius', 3.0)
        self.declare_parameter('blacklist_radius', 1.0)
        self.declare_parameter('max_wander_attempts', 3)

        self._ns = self.get_parameter('robot_namespace').value or \
            self.get_namespace().strip('/')
        self._home_x = self.get_parameter('home_x').value
        self._home_y = self.get_parameter('home_y').value
        self._min_frontier = self.get_parameter('min_frontier_size').value
        self._claim_penalty = self.get_parameter('claim_penalty').value
        self._goal_tolerance = self.get_parameter('goal_tolerance').value
        self._obstacle_clearance = self.get_parameter('obstacle_clearance').value
        self._repulsion_radius = self.get_parameter('robot_repulsion_radius').value
        self._blacklist_radius = self.get_parameter('blacklist_radius').value
        self._max_wander = self.get_parameter('max_wander_attempts').value

        self._map: OccupancyGrid | None = None
        self._other_claims: dict[str, FrontierClaim] = {}
        self._active = False
        self._returning = False
        self._goal_handle = None
        self._odom_x = self._home_x
        self._odom_y = self._home_y
        self._other_poses: dict[str, tuple[float, float]] = {}

        self._blacklist: list[tuple[float, float]] = []
        self._no_frontier_count = 0
        self._goal_failed = False

        transient = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_sub = self.create_subscription(
            OccupancyGrid, 'map', self._on_map, transient)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)

        num_robots = self.get_parameter('num_robots').value
        prefix = self.get_parameter('robot_prefix').value
        for i in range(num_robots):
            rid = f'{prefix}{i}'
            if rid == self._ns:
                continue
            self.create_subscription(
                Odometry, f'/{rid}/odom',
                lambda msg, r=rid: self._on_other_odom(msg, r), 10)

        cmd_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._cmd_sub = self.create_subscription(
            String, '/fleet/command', self._on_command, cmd_qos)

        self._claim_sub = self.create_subscription(
            FrontierClaim, '/fleet/frontier_claims', self._on_claim, 10)

        self._claim_pub = self.create_publisher(
            FrontierClaim, '/fleet/frontier_claims', 10)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        rate = self.get_parameter('explore_rate_hz').value
        self._explore_timer = self.create_timer(1.0 / rate, self._explore_tick)

        self.get_logger().info(
            f'Distributed explore ready: ns={self._ns}, '
            f'home=({self._home_x}, {self._home_y})')

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _on_odom(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y

    def _on_other_odom(self, msg: Odometry, robot_id: str):
        self._other_poses[robot_id] = (
            msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_command(self, msg: String):
        if msg.data == 'start_explore':
            self._active = True
            self._returning = False
            self._blacklist.clear()
            self._no_frontier_count = 0
            self.get_logger().info('Exploration activated')
        elif msg.data == 'return_home':
            self._active = False
            self._returning = True
            self._cancel_goal()
            self._send_goal(self._home_x, self._home_y)
            self.get_logger().info('Returning home')
        elif msg.data == 'stop':
            self._active = False
            self._returning = False
            self._cancel_goal()

    def _on_claim(self, msg: FrontierClaim):
        if msg.robot_id != self._ns:
            self._other_claims[msg.robot_id] = msg

    def _explore_tick(self):
        if not self._active or self._map is None:
            return

        if self._goal_handle is not None:
            return

        frontiers = self._extract_frontiers()
        if not frontiers:
            self._no_frontier_count += 1
            if self._no_frontier_count >= self._max_wander:
                self.get_logger().info('No frontiers found — trying random wander')
                goal = self._random_free_goal()
                if goal:
                    self._send_goal(goal[0], goal[1])
                else:
                    self.get_logger().info('No free space for wander — exploration complete')
                    self._active = False
                    self._returning = True
                    self._send_goal(self._home_x, self._home_y)
            return

        self._no_frontier_count = 0

        best = self._select_frontier(frontiers)
        if best is None:
            self._no_frontier_count += 1
            return

        cx, cy, score = best
        self._publish_claim(cx, cy, score)
        self._send_goal(cx, cy)

    def _extract_frontiers(self):
        grid = self._map
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        data = np.array(grid.data, dtype=np.int8).reshape((h, w))

        free_mask = data == _FREE
        unknown_mask = data == _UNKNOWN

        frontier_mask = np.zeros_like(free_mask)
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(np.roll(unknown_mask, dy, axis=0), dx, axis=1)
            frontier_mask |= (free_mask & shifted)

        fy, fx = np.where(frontier_mask)
        if len(fx) == 0:
            return []

        world_x = fx * res + ox + res / 2
        world_y = fy * res + oy + res / 2
        points = np.column_stack([world_x, world_y])

        clusters = self._cluster_frontiers(points)
        return clusters

    def _cluster_frontiers(self, points, cluster_dist=0.5):
        if len(points) == 0:
            return []
        visited = np.zeros(len(points), dtype=bool)
        clusters = []

        for i in range(len(points)):
            if visited[i]:
                continue
            dists = np.linalg.norm(points - points[i], axis=1)
            members = dists < cluster_dist
            visited |= members
            member_pts = points[members]
            if len(member_pts) >= self._min_frontier:
                cx = float(np.mean(member_pts[:, 0]))
                cy = float(np.mean(member_pts[:, 1]))
                clusters.append((cx, cy, len(member_pts)))

        return clusters

    def _near_obstacle(self, wx, wy):
        grid = self._map
        if grid is None:
            return False
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        w = grid.info.width
        h = grid.info.height
        data = np.array(grid.data, dtype=np.int8).reshape((h, w))

        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        radius_cells = int(self._obstacle_clearance / res)

        y_lo = max(0, gy - radius_cells)
        y_hi = min(h, gy + radius_cells + 1)
        x_lo = max(0, gx - radius_cells)
        x_hi = min(w, gx + radius_cells + 1)

        patch = data[y_lo:y_hi, x_lo:x_hi]
        return bool(np.any(patch > 50))

    def _is_blacklisted(self, cx, cy):
        for bx, by in self._blacklist:
            if math.hypot(cx - bx, cy - by) < self._blacklist_radius:
                return True
        return False

    def _select_frontier(self, frontiers):
        best_score = -1.0
        best = None

        for cx, cy, size in frontiers:
            dist = math.hypot(cx - self._robot_x(), cy - self._robot_y())
            if dist < 0.1:
                continue

            if self._near_obstacle(cx, cy):
                continue

            if self._is_blacklisted(cx, cy):
                continue

            score = float(size) / max(dist, 0.5)

            for claim in self._other_claims.values():
                claim_dist = math.hypot(
                    cx - claim.frontier_centroid.x,
                    cy - claim.frontier_centroid.y)
                if claim_dist < 3.0:
                    score *= self._claim_penalty

            for ox, oy in self._other_poses.values():
                robot_dist = math.hypot(cx - ox, cy - oy)
                if robot_dist < self._repulsion_radius:
                    score *= 0.1

            if score > best_score:
                best_score = score
                best = (cx, cy, score)

        return best

    def _random_free_goal(self):
        grid = self._map
        if grid is None:
            return None
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        data = np.array(grid.data, dtype=np.int8).reshape((h, w))

        free_y, free_x = np.where(data == _FREE)
        if len(free_x) == 0:
            return None

        for _ in range(20):
            idx = random.randint(0, len(free_x) - 1)
            wx = float(free_x[idx]) * res + ox + res / 2
            wy = float(free_y[idx]) * res + oy + res / 2

            dist = math.hypot(wx - self._robot_x(), wy - self._robot_y())
            if dist < 1.0 or dist > 5.0:
                continue
            if self._near_obstacle(wx, wy):
                continue
            if self._is_blacklisted(wx, wy):
                continue

            self.get_logger().info(f'Random wander goal: ({wx:.2f}, {wy:.2f})')
            return (wx, wy)

        return None

    def _robot_x(self):
        return self._odom_x

    def _robot_y(self):
        return self._odom_y

    def _publish_claim(self, cx, cy, score):
        msg = FrontierClaim()
        msg.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self._ns
        msg.frontier_centroid = Point(x=cx, y=cy, z=0.0)
        msg.utility_score = score
        self._claim_pub.publish(msg)

    def _send_goal(self, x, y):
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self._current_goal_xy = (x, y)
        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f})')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self._blacklist_current_goal()
            self._goal_handle = None
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result()
        status = result.status
        self._goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Goal aborted — blacklisting location')
            self._blacklist_current_goal()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal canceled')
        else:
            self.get_logger().warn(f'Goal ended with status {status}')
            self._blacklist_current_goal()

    def _blacklist_current_goal(self):
        if hasattr(self, '_current_goal_xy'):
            gx, gy = self._current_goal_xy
            self._blacklist.append((gx, gy))
            self.get_logger().info(
                f'Blacklisted ({gx:.2f}, {gy:.2f}), '
                f'total blacklisted: {len(self._blacklist)}')

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None


def main():
    rclpy.init()
    node = DistributedExploreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
