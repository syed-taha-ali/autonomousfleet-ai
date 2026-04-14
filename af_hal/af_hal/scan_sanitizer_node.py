#!/usr/bin/env python3
"""Resample MS200 LaserScan messages onto a canonical fixed-size grid.

The HiWonder/Oradar MS200 driver emits /scan_raw where len(ranges),
angle_min, angle_max and angle_increment jitter between messages. Karto
(inside slam_toolbox) latches the LaserRangeFinder geometry from the first
scan it sees and rejects every subsequent scan with a different reading
count ("LaserRangeScan contains N range readings, expected M").

This node publishes /scan with a canonical fixed grid:
  angle_min = 0.0
  angle_increment = 2*pi / bins
  angle_max = (bins - 1) * angle_increment
  len(ranges) = bins   (default 450)

Each input reading is bucketed to the nearest output angle bin; the lowest
range value in each bucket wins (conservative -- treats obstacles as near).
Empty bins are filled with +inf so Karto treats them as "no return".
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

TWO_PI = 2.0 * math.pi


class ScanSanitizer(Node):
    def __init__(self):
        super().__init__('scan_sanitizer')
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('bins', 450)
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.bins = int(self.get_parameter('bins').value)
        self.bin_width = TWO_PI / self.bins

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(LaserScan, out_topic, qos)
        self.sub = self.create_subscription(LaserScan, in_topic, self._cb, qos)
        self.get_logger().info(
            f'scan_sanitizer: {in_topic} -> {out_topic} (bins={self.bins})'
        )

    def _cb(self, msg: LaserScan):
        if not msg.ranges:
            return

        out_ranges = [math.inf] * self.bins
        out_intensities = [0.0] * self.bins
        has_intensity = len(msg.intensities) == len(msg.ranges)

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            angle = angle % TWO_PI
            j = int(angle / self.bin_width + 0.5) % self.bins
            if r < out_ranges[j]:
                out_ranges[j] = r
                if has_intensity:
                    out_intensities[j] = msg.intensities[i]

        out = LaserScan()
        out.header = msg.header
        out.angle_min = 0.0
        out.angle_increment = self.bin_width
        out.angle_max = (self.bins - 1) * self.bin_width
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = out_ranges
        out.intensities = out_intensities if has_intensity else []
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
