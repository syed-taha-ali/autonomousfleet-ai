#!/usr/bin/env python3
"""Safety validator: gate all mission commands through battery, speed, and type checks.

Serves /mission/validate (af_msgs/ValidateCommand). Caches the latest battery
voltage from the hardware watchdog and rejects commands when the 2S LiPo has
sagged below the safe threshold.
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16

from af_msgs.msg import MissionCommand
from af_msgs.srv import ValidateCommand


_ALLOWED_TYPES = frozenset([
    'find_object',
    'patrol',
    'drive_for',
    'stop',
    'set_speed',
    'scan_area',
    'explore',
])


class SafetyValidatorNode(Node):
    def __init__(self):
        super().__init__('safety_validator')

        self.declare_parameter('min_battery_v', 7.2)
        self.declare_parameter('max_linear', 0.3)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('max_distance_m', 10.0)

        self._min_battery_v = self.get_parameter('min_battery_v').value
        self._max_linear = self.get_parameter('max_linear').value
        self._max_angular = self.get_parameter('max_angular').value
        self._max_distance_m = self.get_parameter('max_distance_m').value

        self._last_voltage = None

        self.create_subscription(
            UInt16,
            '/ros_robot_controller/battery',
            self._on_battery,
            10,
        )

        self.create_service(
            ValidateCommand,
            '/mission/validate',
            self._validate_cb,
        )

        self.get_logger().info(
            f'Safety validator ready (min_battery={self._min_battery_v} V, '
            f'max_linear={self._max_linear}, max_angular={self._max_angular})'
        )

    def _on_battery(self, msg: UInt16):
        self._last_voltage = msg.data / 1000.0

    def _validate_cb(self, request, response):
        cmd: MissionCommand = request.command
        response.valid = True
        response.rejection_reason = ''

        if cmd.command_type not in _ALLOWED_TYPES:
            response.valid = False
            response.rejection_reason = f'Unknown command type: {cmd.command_type}'
            return response

        if self._last_voltage is not None and self._last_voltage < self._min_battery_v:
            if cmd.command_type != 'stop':
                response.valid = False
                response.rejection_reason = (
                    f'Battery too low: {self._last_voltage:.2f} V < {self._min_battery_v} V'
                )
                return response

        try:
            params = json.loads(cmd.parameters_json) if cmd.parameters_json else {}
        except json.JSONDecodeError:
            response.valid = False
            response.rejection_reason = 'Malformed parameters_json'
            return response

        if cmd.command_type == 'set_speed':
            lin = params.get('max_linear', 0.0)
            ang = params.get('max_angular', 0.0)
            if lin > self._max_linear:
                response.valid = False
                response.rejection_reason = (
                    f'max_linear {lin} exceeds limit {self._max_linear}'
                )
                return response
            if ang > self._max_angular:
                response.valid = False
                response.rejection_reason = (
                    f'max_angular {ang} exceeds limit {self._max_angular}'
                )
                return response

        if cmd.command_type == 'find_object':
            dist = params.get('max_distance_m', 10.0)
            if dist > self._max_distance_m:
                response.valid = False
                response.rejection_reason = (
                    f'max_distance_m {dist} exceeds limit {self._max_distance_m}'
                )
                return response

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SafetyValidatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
