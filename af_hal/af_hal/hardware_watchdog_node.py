#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hardware watchdog: battery, controller heartbeat, diagnostic aggregation.

- Subscribes to ``/ros_robot_controller/battery`` (std_msgs/UInt16 in mV;
  e.g. 8040 -> 8.04 V on a 2S pack).
- Publishes ``/diagnostics`` (``diagnostic_msgs/DiagnosticArray``) at 1 Hz.
- Heartbeat: if no battery message is received for ``heartbeat_timeout``
  seconds, reports an ERROR so downstream consumers (e.g. mission_manager)
  can refuse to start new missions.
- Battery thresholds (2S Li-ion defaults; override via parameters for 3S):
    * WARN  when voltage < ``warn_voltage``  (default 6.8 V)
    * ERROR when voltage < ``error_voltage`` (default 6.4 V)
  Trigger a single log warning/error on each transition.
"""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import UInt16


class HardwareWatchdog(Node):
    def __init__(self):
        super().__init__('hardware_watchdog')

        self.declare_parameter('battery_topic', '/ros_robot_controller/battery')
        self.declare_parameter('warn_voltage', 6.8)
        self.declare_parameter('error_voltage', 6.4)
        self.declare_parameter('heartbeat_timeout', 3.0)
        self.declare_parameter('publish_rate', 1.0)

        self.warn_voltage = float(self.get_parameter('warn_voltage').value)
        self.error_voltage = float(self.get_parameter('error_voltage').value)
        self.heartbeat_timeout = float(self.get_parameter('heartbeat_timeout').value)

        self.last_voltage = None  # volts
        self.last_battery_time = None
        self.last_reported_level = DiagnosticStatus.OK

        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(
            UInt16,
            self.get_parameter('battery_topic').value,
            self._on_battery,
            10,
        )

        period = 1.0 / float(self.get_parameter('publish_rate').value)
        self.create_timer(period, self._publish_diagnostics)

        self.get_logger().info('hardware_watchdog started')

    def _on_battery(self, msg: UInt16):
        # RRC Lite firmware reports millivolts (e.g. 8040 -> 8.04 V on a 2S pack).
        self.last_voltage = msg.data / 1000.0
        self.last_battery_time = time.monotonic()

    def _classify(self):
        now = time.monotonic()
        if self.last_battery_time is None or (now - self.last_battery_time) > self.heartbeat_timeout:
            return DiagnosticStatus.ERROR, 'no battery heartbeat from STM32'
        if self.last_voltage is None:
            return DiagnosticStatus.WARN, 'battery voltage unknown'
        if self.last_voltage < self.error_voltage:
            return DiagnosticStatus.ERROR, f'battery critical ({self.last_voltage:.2f} V)'
        if self.last_voltage < self.warn_voltage:
            return DiagnosticStatus.WARN, f'battery low ({self.last_voltage:.2f} V)'
        return DiagnosticStatus.OK, f'battery {self.last_voltage:.2f} V'

    def _publish_diagnostics(self):
        level, message = self._classify()
        status = DiagnosticStatus()
        status.name = 'af_hal: battery/heartbeat'
        status.hardware_id = 'stm32_rrc_lite'
        status.level = level
        status.message = message
        if self.last_voltage is not None:
            status.values.append(KeyValue(key='voltage_v', value=f'{self.last_voltage:.2f}'))
        if self.last_battery_time is not None:
            age = time.monotonic() - self.last_battery_time
            status.values.append(KeyValue(key='battery_age_s', value=f'{age:.2f}'))

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.diag_pub.publish(array)

        if level != self.last_reported_level:
            if level == DiagnosticStatus.ERROR:
                self.get_logger().error(message)
            elif level == DiagnosticStatus.WARN:
                self.get_logger().warn(message)
            else:
                self.get_logger().info(message)
            self.last_reported_level = level


def main():
    rclpy.init()
    node = HardwareWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
