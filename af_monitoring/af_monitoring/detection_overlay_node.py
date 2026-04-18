#!/usr/bin/env python3
"""Dev-PC-side node that composites YOLO bounding boxes onto the camera feed.

Subscribes to /camera/image_raw/compressed and /detections, draws bounding
boxes from the latest detections onto each camera frame, and publishes the
annotated image on /detection_image/compressed. Runs on the Dev PC so the
Pi's CPU is unaffected.
"""
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray


_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)


class DetectionOverlayNode(Node):
    def __init__(self):
        super().__init__('detection_overlay')
        self.declare_parameter('max_det_age_s', 1.0)
        self._max_age = self.get_parameter('max_det_age_s').value

        self._latest_dets: Detection2DArray | None = None
        self._last_det_time = None

        self.create_subscription(
            Detection2DArray, '/detections', self._on_dets, _BEST_EFFORT)
        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self._on_image, _BEST_EFFORT)

        self._pub = self.create_publisher(
            CompressedImage, '/detection_image/compressed', 10)

        self.get_logger().info('Detection overlay node ready')

    def _on_dets(self, msg: Detection2DArray):
        self._latest_dets = msg
        self._last_det_time = self.get_clock().now()

    def _on_image(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return

        if self._latest_dets is not None and self._last_det_time is not None:
            age = (self.get_clock().now() - self._last_det_time).nanoseconds / 1e9
            if age < self._max_age:
                self._draw(frame, self._latest_dets)

        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = bytes(cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])[1])
        self._pub.publish(out)

    def _draw(self, frame, dets: Detection2DArray):
        for det in dets.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            label = f'{hyp.class_id} {hyp.score:.2f}'
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)

            color = (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(
                frame, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
            cv2.putText(
                frame, label, (x1, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionOverlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
