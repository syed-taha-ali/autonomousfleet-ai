#!/usr/bin/env python3
"""Mission manager: topic-based interface for triggering find-object missions.

Subscribes to /mission/command (af_msgs/MissionCommand) and routes commands
to the find_object action server or Nav2 directly. Publishes /mission/status.

This node is the command router for Phase 5b (NLP integration). For Phase 5a,
missions are triggered directly via the /find_object action server.
"""
import json

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger

from af_msgs.action import FindObject
from af_msgs.msg import MissionCommand, MissionStatus
from af_msgs.srv import ValidateCommand


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self._cb_group = ReentrantCallbackGroup()

        self._status_pub = self.create_publisher(MissionStatus, '/mission/status', 10)

        self.create_subscription(
            MissionCommand, '/mission/command', self._on_command, 10,
            callback_group=self._cb_group,
        )

        self._validate_client = self.create_client(
            ValidateCommand, '/mission/validate',
            callback_group=self._cb_group,
        )

        self._find_object_client = ActionClient(
            self, FindObject, 'find_object',
            callback_group=self._cb_group,
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group,
        )

        self.get_logger().info('Mission manager ready (command router)')

    def _on_command(self, msg: MissionCommand):
        self.get_logger().info(
            f'Command received: {msg.command_type} '
            f'(NL: "{msg.natural_language}")'
        )

        if self._validate_client.service_is_ready():
            req = ValidateCommand.Request()
            req.command = msg
            future = self._validate_client.call_async(req)
            future.add_done_callback(
                lambda f: self._on_validated(f, msg)
            )
        else:
            self.get_logger().warn('Safety validator not available — executing anyway')
            self._dispatch(msg)

    def _on_validated(self, future, msg: MissionCommand):
        result = future.result()
        if not result.valid:
            self.get_logger().warn(f'Command rejected: {result.rejection_reason}')
            status = MissionStatus()
            status.mission_id = msg.mission_id
            status.state = 'failed'
            status.error_message = result.rejection_reason
            self._status_pub.publish(status)
            return
        self._dispatch(msg)

    def _dispatch(self, msg: MissionCommand):
        try:
            params = json.loads(msg.parameters_json) if msg.parameters_json else {}
        except json.JSONDecodeError:
            self.get_logger().error('Malformed parameters_json')
            return

        if msg.command_type == 'find_object':
            self._dispatch_find_object(params)
        elif msg.command_type == 'navigate_to':
            self._dispatch_navigate_to(params)
        elif msg.command_type == 'stop':
            self._dispatch_stop()
        elif msg.command_type == 'return_home':
            self._dispatch_find_object({'target_class': '__return_only__', 'max_duration_s': 0.1})
        else:
            self.get_logger().warn(f'Unhandled command type: {msg.command_type}')

    def _dispatch_find_object(self, params: dict):
        if not self._find_object_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FindObject action server not available')
            return

        goal = FindObject.Goal()
        goal.target_class = params.get('target_class', 'suitcase')
        goal.confidence_min = params.get('confidence_min', 0.5)
        goal.votes_required = params.get('votes_required', 3)
        goal.window_size = params.get('window_size', 5)
        goal.max_duration_s = params.get('max_duration_s', 300.0)
        goal.max_distance_m = params.get('max_distance_m', 10.0)
        goal.approach_target = params.get('approach_target', False)

        self._find_object_client.send_goal_async(goal)
        self.get_logger().info(f'FindObject goal sent: {goal.target_class}')

    def _dispatch_navigate_to(self, params: dict):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available')
            return

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(params.get('x', 0.0))
        pose.pose.position.y = float(params.get('y', 0.0))
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_client.send_goal_async(goal)
        self.get_logger().info(f'Nav goal sent: ({pose.pose.position.x}, {pose.pose.position.y})')

    def _dispatch_stop(self):
        if self._find_object_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Cancelling all goals')


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
