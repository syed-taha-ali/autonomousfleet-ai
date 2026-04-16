#!/usr/bin/env python3
"""Decompress a CompressedImage stream into a raw Image stream.

Runs on the Dev PC. The Pi streams `/camera/image_raw/compressed` over WiFi
(JPEG, ~15 FPS, ~2-4 Mbps) and this node turns it back into
`sensor_msgs/Image` (`bgr8`) so the YOLO detector and RViz can consume it.

cv_bridge's imgmsg decoder handles YUYV/MJPEG sources produced by usb_cam
with the image_transport `compressed` plugin.
"""
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class ImageDecompressorNode(Node):
    def __init__(self):
        super().__init__('image_decompressor_node')

        self.declare_parameter('input_topic', '/camera/image_raw/compressed')
        self.declare_parameter('output_topic', '/camera/image_raw')
        self.declare_parameter('output_encoding', 'bgr8')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self._encoding = self.get_parameter('output_encoding').value

        # usb_cam's image_transport compressed publisher is RELIABLE in
        # Humble and emits ~67 KB JPEG frames that fragment into ~45 UDP
        # packets each over WiFi. The robust combination — verified against
        # `ros2 topic hz` and a direct Python probe — is BEST_EFFORT with a
        # generous KEEP_LAST buffer. A tighter buffer (depth<=2) causes DDS
        # to spend most of its time discarding in-flight fragments rather
        # than reassembling a complete frame, producing long 0-frame
        # stretches. Any single late fragment is cheap to drop; the next
        # complete frame is already in flight.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, out_topic, pub_qos)
        self._sub = self.create_subscription(
            CompressedImage, in_topic, self._cb, sub_qos
        )

        self._frames = 0
        self._drops = 0
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f'image_decompressor: {in_topic} -> {out_topic} ({self._encoding})'
        )

    def _cb(self, msg: CompressedImage):
        # Skip cv_bridge's compressed decoder — in Humble it sometimes returns
        # a single-channel (8UC1) buffer for colour JPEGs and then rejects
        # re-encoding as bgr8. cv2.imdecode with IMREAD_COLOR always yields
        # BGR 3-channel output which we then wrap as an Image msg.
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        cv_img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if cv_img is None:
            self._drops += 1
            self.get_logger().warn('cv2.imdecode returned None')
            return

        out = self._bridge.cv2_to_imgmsg(cv_img, encoding=self._encoding)
        out.header = msg.header
        self._pub.publish(out)
        self._frames += 1

    def _log_stats(self):
        fps = self._frames / 5.0
        self.get_logger().info(
            f'decompressed {self._frames} frames ({fps:.1f} FPS), drops={self._drops}'
        )
        self._frames = 0


def main():
    rclpy.init()
    node = ImageDecompressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
