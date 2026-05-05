#!/usr/bin/env python3
"""ActionServer for the find-object-return-home mission.

Combines the FindObject action interface with the full mission state machine
in a single node. Accepts goals, explores via Nav2 + simple_explore_node,
detects the target via temporal voting on /detections, and navigates home.

The explore.launch.py file runs this node alongside simple_explore_node
(which sends Nav2 goals to drive frontier exploration). This node monitors
/detections and, on confirmed detection, cancels exploration by sending a
Nav2 cancel, then drives the return-home leg itself.
"""
import math
import time
import uuid
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt16
from vision_msgs.msg import Detection2DArray

from af_msgs.action import FindObject
from af_msgs.msg import MissionStatus


class State(Enum):
    IDLE = auto()
    CAPTURE_HOME = auto()
    EXPLORING = auto()
    TARGET_CONFIRMED = auto()
    RETURN_HOME = auto()
    DONE = auto()
    FAILED = auto()


class FindObjectActionNode(Node):
    def __init__(self):
        super().__init__('find_object_action')
        self._cb_group = ReentrantCallbackGroup()

        # Parameters (defaults; overridden per-goal)
        self.declare_parameter('min_battery_v', 7.2)
        self._min_battery_v = self.get_parameter('min_battery_v').value

        # State machine
        self._state = State.IDLE
        self._mission_id = ''
        self._home_pose = PoseStamped()
        self._detection_pose = PoseStamped()
        self._start_time = 0.0
        self._last_voltage = 0.0
        self._distance_travelled = 0.0
        self._error_message = ''
        self._target_class = 'suitcase'
        self._confidence_min = 0.5
        self._votes_required = 3
        self._window_size = 5
        self._max_duration_s = 300.0
        self._max_distance_m = 10.0
        self._vote_window = deque(maxlen=5)
        self._return_nav_handle = None
        self._return_complete = False
        self._timed_out = False
        self._explorer_paused = False
        self._last_odom_x = None
        self._last_odom_y = None

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Nav2 action client (for return-home leg)
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group,
        )

        # Subscriptions
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
            UInt16, '/ros_robot_controller/battery', self._on_battery, 10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Odometry, '/odom', self._on_odom, 10,
            callback_group=self._cb_group,
        )

        # Publishers
        self._status_pub = self.create_publisher(MissionStatus, '/mission/status', 10)
        self._explore_enable_pub = self.create_publisher(Bool, '/explore/enable', 10)

        # Action server
        self._action_server = ActionServer(
            self,
            FindObject,
            'find_object',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info('FindObject action server ready')

    # ── Action callbacks ───────────────────────────────────────────

    def _goal_cb(self, goal_request):
        if self._state not in (State.IDLE, State.DONE, State.FAILED):
            self.get_logger().warn('Rejecting goal — mission already active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        goal = goal_handle.request
        self._target_class = goal.target_class or 'suitcase'
        self._confidence_min = goal.confidence_min if goal.confidence_min > 0 else 0.5
        self._votes_required = goal.votes_required if goal.votes_required > 0 else 3
        self._window_size = goal.window_size if goal.window_size > 0 else 5
        self._max_duration_s = goal.max_duration_s if goal.max_duration_s > 0 else 300.0
        self._max_distance_m = goal.max_distance_m if goal.max_distance_m > 0 else 10.0
        self._vote_window = deque(maxlen=self._window_size)

        self.get_logger().info(
            f'Goal accepted: find "{self._target_class}" '
            f'(conf>={self._confidence_min}, votes={self._votes_required}/{self._window_size}, '
            f'timeout={self._max_duration_s}s)'
        )

        self._mission_id = str(uuid.uuid4())[:8]
        self._start_time = time.monotonic()
        self._distance_travelled = 0.0
        self._last_odom_x = None
        self._last_odom_y = None
        self._detection_pose = PoseStamped()
        self._error_message = ''
        self._return_complete = False
        self._return_nav_handle = None
        self._timed_out = False
        self._explorer_paused = False

        # CAPTURE_HOME
        self._set_state(State.CAPTURE_HOME)
        if not await self._wait_for_home_pose(goal_handle):
            goal_handle.abort()
            return self._make_result(False, self._error_message or 'home_capture_failed')

        # EXPLORING — driven by simple_explore_node running in parallel
        self._set_state(State.EXPLORING)
        self._set_explorer_enabled(True)
        self.get_logger().info('Exploration active — waiting for target detection')

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._set_explorer_enabled(False)
                self._set_state(State.FAILED)
                self._error_message = 'cancelled'
                goal_handle.canceled()
                return self._make_result(False, 'cancelled')

            self._publish_feedback(goal_handle)
            self._check_safety()

            if self._state == State.TARGET_CONFIRMED:
                break
            if self._state == State.FAILED:
                self._set_explorer_enabled(False)
                goal_handle.abort()
                return self._make_result(False, self._error_message)

            time.sleep(0.5)

        # RETURN_HOME
        self._set_state(State.RETURN_HOME)
        self._navigate_home()

        while rclpy.ok() and not self._return_complete:
            if goal_handle.is_cancel_requested:
                self._cancel_return_nav()
                self._set_state(State.FAILED)
                self._error_message = 'cancelled'
                goal_handle.canceled()
                return self._make_result(False, 'cancelled')

            self._publish_feedback(goal_handle)
            time.sleep(0.5)

        if self._state == State.FAILED:
            goal_handle.abort()
            return self._make_result(False, self._error_message)

        # DONE
        self._set_state(State.DONE)
        self._publish_feedback(goal_handle)
        if self._timed_out:
            goal_handle.abort()
            return self._make_result(False, 'timeout')
        goal_handle.succeed()
        return self._make_result(True, 'found')

    # ── Home capture ───────────────────────────────────────────────

    async def _wait_for_home_pose(self, goal_handle, timeout_s=30.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            try:
                t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                self._home_pose.header.frame_id = 'map'
                self._home_pose.header.stamp = t.header.stamp
                self._home_pose.pose.position.x = t.transform.translation.x
                self._home_pose.pose.position.y = t.transform.translation.y
                self._home_pose.pose.position.z = 0.0
                self._home_pose.pose.orientation = t.transform.rotation
                self.get_logger().info(
                    f'Home pose captured: ({t.transform.translation.x:.2f}, '
                    f'{t.transform.translation.y:.2f})'
                )
                return True
            except tf2_ros.TransformException:
                pass
            self._publish_feedback(goal_handle)
            time.sleep(0.5)

        self._error_message = 'tf_timeout'
        self._set_state(State.FAILED)
        return False

    # ── Detection voting ───────────────────────────────────────────

    def _on_detections(self, msg: Detection2DArray):
        if self._state != State.EXPLORING:
            return

        hit = False
        for det in msg.detections:
            for result in det.results:
                if (result.hypothesis.class_id == self._target_class
                        and result.hypothesis.score >= self._confidence_min):
                    hit = True
                    break
            if hit:
                break

        self._vote_window.append(hit)
        votes = sum(self._vote_window)

        # Pause explorer once votes reach votes_required-1 so the robot has
        # already started moving before we stop it, and single-frame false
        # positives don't freeze exploration immediately.
        if votes >= (self._votes_required - 1) and not self._explorer_paused:
            self._set_explorer_enabled(False)
            self.get_logger().info(
                f'Detection building — pausing explorer to accumulate votes '
                f'({votes}/{self._votes_required})'
            )
        # Resume explorer if votes fall below the pause threshold (object lost)
        elif self._explorer_paused and votes < (self._votes_required - 1):
            self._set_explorer_enabled(True)
            self.get_logger().info(f'Votes dropped to {votes} — resuming explorer')

        if votes >= self._votes_required:
            self.get_logger().info(
                f'Target "{self._target_class}" confirmed '
                f'({votes}/{len(self._vote_window)} votes)'
            )
            self._capture_detection_pose()
            self._set_state(State.TARGET_CONFIRMED)

    def _set_explorer_enabled(self, enabled: bool):
        self._explorer_paused = not enabled
        msg = Bool()
        msg.data = enabled
        self._explore_enable_pub.publish(msg)

    def _capture_detection_pose(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self._detection_pose.header.frame_id = 'map'
            self._detection_pose.header.stamp = t.header.stamp
            self._detection_pose.pose.position.x = t.transform.translation.x
            self._detection_pose.pose.position.y = t.transform.translation.y
            self._detection_pose.pose.position.z = 0.0
            self._detection_pose.pose.orientation = t.transform.rotation
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'Could not capture detection pose: {e}')

    # ── Safety checks ──────────────────────────────────────────────

    def _check_safety(self):
        elapsed = time.monotonic() - self._start_time

        if self._last_voltage > 0 and self._last_voltage < self._min_battery_v:
            if self._state not in (State.DONE, State.FAILED, State.RETURN_HOME):
                self.get_logger().error(
                    f'Battery critical: {self._last_voltage:.2f} V — aborting'
                )
                self._error_message = 'battery'
                self._set_state(State.FAILED)
                return

        if elapsed > self._max_duration_s and self._state == State.EXPLORING:
            self.get_logger().warn(f'Mission timeout ({self._max_duration_s}s)')
            self._error_message = 'timeout'
            self._timed_out = True
            self._set_state(State.TARGET_CONFIRMED)

    def _on_battery(self, msg: UInt16):
        self._last_voltage = msg.data / 1000.0

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_odom_x is not None and self._state not in (State.IDLE, State.DONE, State.FAILED):
            dx = x - self._last_odom_x
            dy = y - self._last_odom_y
            self._distance_travelled += math.sqrt(dx * dx + dy * dy)
        self._last_odom_x = x
        self._last_odom_y = y

    # ── Navigation ─────────────────────────────────────────────────

    def _navigate_home(self):
        self.get_logger().info('Navigating to home pose...')
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available for return-home')
            self._error_message = 'nav2_unavailable'
            self._set_state(State.FAILED)
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._home_pose
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_return_goal_response)

    def _on_return_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Return-home goal rejected')
            self._error_message = 'goal_rejected'
            self._set_state(State.FAILED)
            self._return_complete = True
            return
        self._return_nav_handle = handle
        handle.get_result_async().add_done_callback(self._on_return_done)

    def _on_return_done(self, future):
        self._return_nav_handle = None
        error_m = self._compute_home_error()
        self.get_logger().info(f'Arrived home (error: {error_m:.3f} m)')
        self._return_complete = True

    def _cancel_return_nav(self):
        if self._return_nav_handle is not None:
            self._return_nav_handle.cancel_goal_async()
            self._return_nav_handle = None

    def _compute_home_error(self) -> float:
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            dx = t.transform.translation.x - self._home_pose.pose.position.x
            dy = t.transform.translation.y - self._home_pose.pose.position.y
            return math.sqrt(dx * dx + dy * dy)
        except tf2_ros.TransformException:
            return -1.0

    # ── State / status ─────────────────────────────────────────────

    def _set_state(self, new_state: State):
        old = self._state
        self._state = new_state
        self.get_logger().info(f'State: {old.name} -> {new_state.name}')
        self._publish_status()

    def _publish_feedback(self, goal_handle):
        fb = FindObject.Feedback()
        fb.current_state = self._state.name.lower()
        fb.distance_travelled_m = self._distance_travelled
        fb.battery_voltage = self._last_voltage
        goal_handle.publish_feedback(fb)
        self._publish_status()

    def _publish_status(self):
        msg = MissionStatus()
        msg.mission_id = self._mission_id
        msg.state = self._state.name.lower()
        msg.current_action = self._describe_action()
        msg.home_pose = self._home_pose
        msg.distance_travelled_m = self._distance_travelled
        msg.elapsed_s = float(time.monotonic() - self._start_time) if self._start_time else 0.0
        msg.error_message = self._error_message
        self._status_pub.publish(msg)

    def _describe_action(self) -> str:
        return {
            State.IDLE: 'waiting',
            State.CAPTURE_HOME: 'capturing_home_pose',
            State.EXPLORING: f'exploring_for_{self._target_class}',
            State.TARGET_CONFIRMED: 'target_confirmed',
            State.RETURN_HOME: 'navigating_home',
            State.DONE: 'mission_complete',
            State.FAILED: 'mission_failed',
        }.get(self._state, 'unknown')

    def _make_result(self, success: bool, reason: str) -> FindObject.Result:
        result = FindObject.Result()
        result.success = success
        result.termination_reason = reason
        result.home_pose = self._home_pose
        result.detection_pose = self._detection_pose
        result.home_return_error_m = (
            self._compute_home_error() if success else -1.0
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FindObjectActionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
