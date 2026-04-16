#!/usr/bin/env python3
"""Real-time YOLO object detector for the Dev-PC perception pipeline.

Subscribes to `/camera/image_raw`, runs Ultralytics YOLO on CUDA, and
publishes `vision_msgs/Detection2DArray` on `/detections` plus an optional
annotated `/detection_image` for RViz debug.

Model selection policy
----------------------
1. If the `model_path` parameter points at an existing file, load it.
2. Otherwise load the Ultralytics zoo model named by `model_name`
   (default `yolov5su.pt`), which Ultralytics fetches on first use and
   caches under `~/.config/Ultralytics/`.

This keeps the package runnable on a fresh clone: `ros2 launch
af_perception perception.launch.py` Just Works with COCO classes, and the
operator can drop a custom `best.pt` into `af_perception/models/` once
training lands.

Runtime cost
------------
YOLOv5s on an RTX 3060 at 640x640: ~7 ms inference, ~30-60 FPS end-to-end.
On CPU (fallback for laptops), the same model lands at 3-5 FPS, which is
enough to smoke-test the plumbing but not enough for live navigation.
"""
import os
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)
from cv_bridge import CvBridge


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('model_name', 'yolov5su.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('annotated_topic', '/detection_image')
        self.declare_parameter('publish_annotated', True)
        # An empty yaml list like `classes: []` is ambiguous to rclpy's type
        # inference and leaves the parameter "uninitialized". Use an explicit
        # INTEGER_ARRAY type so empty == "no filter" and any [0, 39, ...] list
        # just works. Any value < 0 in the list is treated as the "all classes"
        # sentinel too — handy on the launch command line.
        self.declare_parameter('classes', Parameter.Type.INTEGER_ARRAY)

        self._conf = float(self.get_parameter('confidence_threshold').value)
        self._iou = float(self.get_parameter('iou_threshold').value)
        self._imgsz = int(self.get_parameter('image_size').value)
        self._device = str(self.get_parameter('device').value)
        classes_param = self.get_parameter_or('classes', Parameter('classes', value=[])).value
        classes_param = [int(c) for c in (classes_param or []) if int(c) >= 0]
        self._class_filter = classes_param or None
        self._publish_annotated = bool(self.get_parameter('publish_annotated').value)

        self._model = self._load_model()
        self._names = getattr(self._model, 'names', {}) or {}

        self._bridge = CvBridge()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self._det_pub = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter('detections_topic').value),
            10,
        )
        self._ann_pub = None
        if self._publish_annotated:
            self._ann_pub = self.create_publisher(
                Image,
                str(self.get_parameter('annotated_topic').value),
                qos,
            )

        self._sub = self.create_subscription(
            Image,
            str(self.get_parameter('input_topic').value),
            self._cb,
            qos,
        )

        self._frames = 0
        self._inference_s = 0.0
        self.create_timer(5.0, self._log_stats)

    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:  # noqa: BLE001
            self.get_logger().fatal(
                'ultralytics not installed — run `pip install ultralytics` '
                'on the Dev PC'
            )
            raise SystemExit(1) from exc

        path = str(self.get_parameter('model_path').value).strip()
        if path and os.path.isfile(path):
            self.get_logger().info(f'loading custom weights: {path}')
            model = YOLO(path)
        else:
            name = str(self.get_parameter('model_name').value)
            if path:
                self.get_logger().warn(
                    f'model_path "{path}" not found — falling back to zoo "{name}"'
                )
            else:
                self.get_logger().info(f'loading zoo model: {name}')
            model = YOLO(name)

        try:
            model.to(self._device)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'device "{self._device}" unavailable ({exc}); using CPU'
            )
            self._device = 'cpu'
            model.to('cpu')
        return model

    def _cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return

        t0 = time.perf_counter()
        results = self._model.predict(
            source=frame,
            imgsz=self._imgsz,
            conf=self._conf,
            iou=self._iou,
            device=self._device,
            classes=self._class_filter,
            verbose=False,
        )
        self._inference_s += time.perf_counter() - t0
        self._frames += 1

        if not results:
            return
        result = results[0]

        det_array = Detection2DArray()
        det_array.header = msg.header

        boxes = result.boxes
        if boxes is None or boxes.shape[0] == 0:
            self._det_pub.publish(det_array)
            self._publish_annotated_image(msg.header, frame, result)
            return

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        cls = boxes.cls.cpu().numpy().astype(int)

        for (x1, y1, x2, y2), conf, cls_id in zip(xyxy, confs, cls):
            det = Detection2D()
            det.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(self._names.get(int(cls_id), int(cls_id)))
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)
            bbox = BoundingBox2D()
            bbox.center.position.x = float((x1 + x2) * 0.5)
            bbox.center.position.y = float((y1 + y2) * 0.5)
            bbox.center.theta = 0.0
            bbox.size_x = float(x2 - x1)
            bbox.size_y = float(y2 - y1)
            det.bbox = bbox
            det_array.detections.append(det)

        self._det_pub.publish(det_array)
        self._publish_annotated_image(msg.header, frame, result)

    def _publish_annotated_image(self, header, frame, result):
        if self._ann_pub is None:
            return
        try:
            annotated = result.plot()
        except Exception:  # noqa: BLE001
            annotated = frame
        if annotated.dtype != np.uint8:
            annotated = annotated.astype(np.uint8)
        ann_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        ann_msg.header = header
        self._ann_pub.publish(ann_msg)

    def _log_stats(self):
        if self._frames == 0:
            self.get_logger().info('yolo: no frames in the last 5 s')
            return
        fps = self._frames / 5.0
        avg_ms = (self._inference_s / self._frames) * 1000.0
        self.get_logger().info(
            f'yolo: {fps:.1f} FPS, avg inference {avg_ms:.1f} ms '
            f'({self._device})'
        )
        self._frames = 0
        self._inference_s = 0.0


def main():
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
