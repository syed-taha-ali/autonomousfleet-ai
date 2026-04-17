#!/usr/bin/env python3
"""Subscribes to key topics, measures publication rates and latency, publishes
aggregated results on /diagnostics at 1 Hz.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan, CompressedImage


_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

_TRANSIENT_LOCAL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class _TopicStats:
    __slots__ = ('count', 'first_ts', 'last_ts', 'latencies')

    def __init__(self):
        self.count = 0
        self.first_ts = 0.0
        self.last_ts = 0.0
        self.latencies: list[float] = []

    def tick(self, latency: float | None = None):
        now = time.monotonic()
        if self.first_ts == 0.0:
            self.first_ts = now
        self.last_ts = now
        self.count += 1
        if latency is not None:
            self.latencies.append(latency)
            if len(self.latencies) > 200:
                self.latencies = self.latencies[-100:]

    def hz(self) -> float:
        if self.count < 2:
            return 0.0
        dt = self.last_ts - self.first_ts
        return (self.count - 1) / dt if dt > 0 else 0.0

    def avg_latency_ms(self) -> float:
        if not self.latencies:
            return 0.0
        return sum(self.latencies) / len(self.latencies) * 1000.0

    def reset(self):
        self.count = 0
        self.first_ts = 0.0
        self.last_ts = 0.0
        self.latencies.clear()


class PerfMonitorNode(Node):
    def __init__(self):
        super().__init__('perf_monitor')
        self.declare_parameter('publish_rate_hz', 1.0)
        rate = self.get_parameter('publish_rate_hz').value

        self._stats: dict[str, _TopicStats] = {}
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self._subscribe('/scan', LaserScan, _BEST_EFFORT_QOS, stamped=True)
        self._subscribe('/odom', Odometry, 10, stamped=True)
        self._subscribe('/imu', Imu, _BEST_EFFORT_QOS, stamped=True)
        self._subscribe('/cmd_vel', Twist, 10)
        self._subscribe('/map', OccupancyGrid, _TRANSIENT_LOCAL_QOS)
        self._subscribe('/camera/demo/compressed', CompressedImage, _BEST_EFFORT_QOS, stamped=True)

        self.create_timer(1.0 / rate, self._publish_diagnostics)
        self.get_logger().info(
            f'Performance monitor started — tracking {len(self._stats)} topics at {rate} Hz')

    def _subscribe(self, topic, msg_type, qos, stamped=False):
        self._stats[topic] = _TopicStats()
        if stamped:
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._on_stamped(msg, t), qos)
        else:
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._on_unstamped(t), qos)

    def _on_stamped(self, msg, topic: str):
        now = self.get_clock().now()
        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            latency = (now - stamp).nanoseconds / 1e9
            if latency < 0 or latency > 30.0:
                latency = None
        except Exception:
            latency = None
        self._stats[topic].tick(latency)

    def _on_unstamped(self, topic: str):
        self._stats[topic].tick()

    def _publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        for topic, stats in self._stats.items():
            hz = stats.hz()
            status = DiagnosticStatus()
            status.name = f'perf_monitor: {topic}'
            status.hardware_id = 'af_monitoring'
            status.level = DiagnosticStatus.OK
            status.message = f'{hz:.1f} Hz'
            status.values = [
                KeyValue(key='rate_hz', value=f'{hz:.2f}'),
                KeyValue(key='msg_count', value=str(stats.count)),
            ]

            lat = stats.avg_latency_ms()
            if lat > 0:
                status.values.append(
                    KeyValue(key='avg_latency_ms', value=f'{lat:.1f}'))

            if stats.count == 0:
                status.level = DiagnosticStatus.WARN
                status.message = 'no messages received'

            arr.status.append(status)
            stats.reset()

        self._diag_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PerfMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
