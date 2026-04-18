#!/usr/bin/env python3
"""Mission manager: command router for NLP and direct mission dispatch.

Subscribes to /mission/command (af_msgs/MissionCommand) and routes commands
to the find_object action server, Nav2, or other handlers. Publishes
/mission/status with outcome feedback.
"""
import json
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import Bool

from af_msgs.action import FindObject
from af_msgs.msg import MissionCommand, MissionStatus
from af_msgs.srv import ValidateCommand


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self._cb_group = ReentrantCallbackGroup()

        self._status_pub = self.create_publisher(MissionStatus, '/mission/status', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._explore_enable_pub = self.create_publisher(Bool, '/explore/enable', 10)

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

        self._waypoint_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints',
            callback_group=self._cb_group,
        )

        self._controller_set_params = self.create_client(
            SetParameters, '/controller_server/set_parameters',
            callback_group=self._cb_group,
        )

        self._explorer_set_params = self.create_client(
            SetParameters, '/simple_explore/set_parameters',
            callback_group=self._cb_group,
        )

        self._active_find_handle = None
        self._drive_timer = None
        self._drive_pub_timer = None
        self._scan_timer = None

        self._speed_linear = 0.2
        self._speed_angular = 0.5

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

        handler = {
            'find_object': self._dispatch_find_object,
            'patrol': self._dispatch_patrol,
            'drive_for': self._dispatch_drive_for,
            'stop': self._dispatch_stop,
            'set_speed': self._dispatch_set_speed,
            'scan_area': self._dispatch_scan_area,
            'explore': self._dispatch_scan_area,
        }.get(msg.command_type)

        if handler is None:
            self.get_logger().warn(f'Unhandled command type: {msg.command_type}')
            return

        handler(params, msg.mission_id)

    def _publish_status(self, mission_id: str, state: str, error: str = ''):
        status = MissionStatus()
        status.mission_id = mission_id
        status.state = state
        status.error_message = error
        self._status_pub.publish(status)

    def _dispatch_find_object(self, params: dict, mission_id: str = ''):
        if not self._find_object_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FindObject action server not available')
            return

        goal = FindObject.Goal()
        goal.target_class = params.get('target_class', 'suitcase')
        goal.confidence_min = float(params.get('confidence_min', 0.5))
        goal.votes_required = int(params.get('votes_required', 3))
        goal.window_size = int(params.get('window_size', 5))
        goal.max_duration_s = float(params.get('max_duration_s', 300.0))
        goal.max_distance_m = float(params.get('max_distance_m', 10.0))
        goal.approach_target = bool(params.get('approach_target', False))

        self._explore_enable_pub.publish(Bool(data=True))
        future = self._find_object_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_find_goal_response(f, mission_id))
        self.get_logger().info(f'FindObject goal sent: {goal.target_class}')

    def _on_find_goal_response(self, future, mission_id):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('FindObject goal rejected')
            self._publish_status(mission_id, 'failed', 'goal_rejected')
            return
        self._active_find_handle = handle
        self._publish_status(mission_id, 'exploring')
        handle.get_result_async().add_done_callback(
            lambda f: self._on_find_done(f, mission_id))

    def _on_find_done(self, future, mission_id):
        self._active_find_handle = None
        self._explore_enable_pub.publish(Bool(data=False))
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self._publish_status(mission_id, 'done')
            self.get_logger().info('FindObject mission completed — explorer disabled')
        else:
            self._publish_status(mission_id, 'failed', 'find_object_aborted')
            self.get_logger().info('FindObject mission aborted — explorer disabled')

    def _dispatch_patrol(self, params: dict, mission_id: str = ''):
        waypoints = params.get('waypoints', [])
        if not waypoints:
            self.get_logger().warn('Patrol called with no waypoints')
            return

        if not self._waypoint_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Waypoint follower not available')
            return

        poses = []
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(wp.get('x', 0.0))
            pose.pose.position.y = float(wp.get('y', 0.0))
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self._waypoint_client.send_goal_async(goal)
        self.get_logger().info(f'Patrol started: {len(poses)} waypoints')

    _DIRECTION_VECTORS = {
        'forward':        ( 1.0,  0.0),
        'backward':       (-1.0,  0.0),
        'left':           ( 0.0,  1.0),
        'right':          ( 0.0, -1.0),
        'forward_left':   ( 0.707,  0.707),
        'forward_right':  ( 0.707, -0.707),
        'backward_left':  (-0.707,  0.707),
        'backward_right': (-0.707, -0.707),
    }

    def _dispatch_drive_for(self, params: dict, mission_id: str = ''):
        direction = params.get('direction', 'forward')

        is_rotation = direction in ('rotate_left', 'rotate_right')

        if is_rotation:
            angular_speed = min(float(params.get('speed', self._speed_angular)), self._speed_angular)
            angle_deg = params.get('angle_deg')

            if angle_deg is not None:
                angle_rad = math.radians(float(angle_deg))
                duration = abs(angle_rad) / angular_speed if angular_speed > 0 else 3.0
            else:
                duration = float(params.get('duration_s', 3.0))

            twist = Twist()
            twist.angular.z = angular_speed if direction == 'rotate_left' else -angular_speed

            desc = f'angle={angle_deg}°' if angle_deg else f'duration={duration:.1f}s'
            self.get_logger().info(
                f'Rotate {direction} at {angular_speed:.2f} rad/s ({desc})')
        else:
            speed = min(float(params.get('speed', self._speed_linear)), self._speed_linear)
            duration = float(params.get('duration_s', 3.0))
            vx_unit, vy_unit = self._DIRECTION_VECTORS.get(direction, (1.0, 0.0))

            twist = Twist()
            twist.linear.x = vx_unit * speed
            twist.linear.y = vy_unit * speed

            self.get_logger().info(
                f'Drive {direction} at {speed:.2f} m/s for {duration:.1f}s '
                f'(vx={twist.linear.x:.2f}, vy={twist.linear.y:.2f})')

        self._drive_cmd = twist
        self._drive_pub_timer = self.create_timer(
            0.1, lambda: self._cmd_vel_pub.publish(self._drive_cmd))

        def stop_driving():
            if self._drive_pub_timer is not None:
                self._drive_pub_timer.cancel()
                self._drive_pub_timer = None
            if self._drive_timer is not None:
                self._drive_timer.cancel()
                self._drive_timer = None
            self._cmd_vel_pub.publish(Twist())
            self.get_logger().info('Drive complete — stopped')
            self._publish_status(mission_id, 'done')

        if self._drive_timer is not None:
            self._drive_timer.cancel()
        self._drive_timer = self.create_timer(duration, stop_driving,
                                               callback_group=self._cb_group)

    def _dispatch_stop(self, params: dict = None, mission_id: str = ''):
        self._cmd_vel_pub.publish(Twist())

        if self._drive_pub_timer is not None:
            self._drive_pub_timer.cancel()
            self._drive_pub_timer = None
        if self._drive_timer is not None:
            self._drive_timer.cancel()
            self._drive_timer = None
        if self._scan_timer is not None:
            self._scan_timer.cancel()
            self._scan_timer = None

        if self._active_find_handle is not None:
            self._active_find_handle.cancel_goal_async()
            self._active_find_handle = None

        self._explore_enable_pub.publish(Bool(data=False))
        self._cancel_all_nav2_goals()
        self._publish_status(mission_id, 'idle')
        self.get_logger().info('STOP: cancelled all goals, zeroed cmd_vel')

    def _cancel_all_nav2_goals(self):
        for action_name in ('navigate_to_pose', 'follow_waypoints'):
            cancel_srv = self.create_client(
                CancelGoal,
                f'/{action_name}/_action/cancel_goal',
                callback_group=self._cb_group,
            )
            if cancel_srv.wait_for_service(timeout_sec=1.0):
                req = CancelGoal.Request()
                cancel_srv.call_async(req)
            cancel_srv.destroy()

    def _dispatch_set_speed(self, params: dict, mission_id: str = ''):
        new_linear = params.get('max_linear')
        new_angular = params.get('max_angular')

        if new_linear is not None:
            self._speed_linear = min(float(new_linear), 0.3)
        if new_angular is not None:
            self._speed_angular = min(float(new_angular), 1.0)

        self.get_logger().info(
            f'Speed updated: linear={self._speed_linear:.2f} m/s, '
            f'angular={self._speed_angular:.2f} rad/s')

        self._reconfigure_nav2_speed()
        self._publish_status(mission_id, 'done')

    def _reconfigure_nav2_speed(self):
        if not self._controller_set_params.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('controller_server set_parameters not available')
            return

        params = []
        for name, val in [
            ('FollowPath.max_vel_x', self._speed_linear),
            ('FollowPath.max_vel_y', self._speed_linear),
            ('FollowPath.max_vel_theta', self._speed_angular),
        ]:
            p = Parameter()
            p.name = name
            p.value = ParameterValue()
            p.value.type = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = val
            params.append(p)

        req = SetParameters.Request()
        req.parameters = params
        future = self._controller_set_params.call_async(req)
        future.add_done_callback(self._on_speed_reconfigure_done)

    def _on_speed_reconfigure_done(self, future):
        try:
            result = future.result()
            ok = all(r.successful for r in result.results)
            if ok:
                self.get_logger().info('Nav2 speed reconfigured')
            else:
                reasons = [r.reason for r in result.results if not r.successful]
                self.get_logger().warn(f'Nav2 speed reconfigure partial failure: {reasons}')
        except Exception as e:
            self.get_logger().warn(f'Nav2 speed reconfigure failed: {e}')

    def _dispatch_scan_area(self, params: dict, mission_id: str = ''):
        mode = params.get('mode')
        if mode is None:
            if 'max_distance_m' in params and params['max_distance_m']:
                mode = 'distance'
            elif 'max_time_s' in params and params['max_time_s']:
                mode = 'timed'
            else:
                mode = 'timed'

        if mode == 'timed':
            max_time = float(params.get('max_time_s', 120.0))
            max_distance = 0.0
        elif mode == 'distance':
            max_time = 0.0
            max_distance = float(params.get('max_distance_m', 5.0))
        else:
            max_time = 0.0
            max_distance = 0.0

        self._configure_explorer(max_time=0.0, max_distance=max_distance)
        self._explore_enable_pub.publish(Bool(data=True))

        if max_time > 0:
            def on_scan_timeout():
                if self._scan_timer is not None:
                    self._scan_timer.cancel()
                    self._scan_timer = None
                self._explore_enable_pub.publish(Bool(data=False))
                self._cancel_all_nav2_goals()
                self._cmd_vel_pub.publish(Twist())
                self._publish_status(mission_id, 'done')
                self.get_logger().info(f'Scan area complete (timeout {max_time}s)')

            if self._scan_timer is not None:
                self._scan_timer.cancel()
            self._scan_timer = self.create_timer(
                max_time, on_scan_timeout, callback_group=self._cb_group)

        self.get_logger().info(
            f'Scan area: mode={mode}, time={max_time}s, distance={max_distance}m')

    def _configure_explorer(self, max_time: float, max_distance: float):
        if not self._explorer_set_params.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Explorer set_parameters not available')
            return

        params = []
        for name, val in [
            ('max_explore_time_s', max_time),
            ('max_explore_distance_m', max_distance),
        ]:
            p = Parameter()
            p.name = name
            p.value = ParameterValue()
            p.value.type = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = val
            params.append(p)

        req = SetParameters.Request()
        req.parameters = params
        try:
            self._explorer_set_params.call(req)
        except Exception as e:
            self.get_logger().warn(f'Explorer param set failed: {e}')


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
