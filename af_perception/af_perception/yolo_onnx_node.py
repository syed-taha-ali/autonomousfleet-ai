#!/usr/bin/env python3
"""Lightweight YOLOv8n detector for on-robot inference (Phase 4.1).

This node is the on-Pi counterpart to the Dev-PC `yolo_detector_node`. It
runs ONNX Runtime on the Pi 5's Cortex-A76 (CPU, NEON SIMD) and consumes
frames from a local `/camera/image_raw` topic so raw camera frames never
cross the WiFi link — only the small `/detections` array does.

Why ONNX Runtime instead of NCNN
--------------------------------
NCNN int8 is the nominally "fastest" option on ARM, but the int8
calibration dance + the Python binding's manual YOLOv8 post-processing
adds a lot of surface area for a model that already lands comfortably
above the ≥3 Hz target at fp32. ONNX Runtime ships precompiled wheels
for aarch64 (`pip install onnxruntime`), YOLOv8 ONNX export is a
one-liner on the Dev PC, and the post-processing is deterministic.
Swap to NCNN later only if the Pi CPU budget gets tight — the public
API of this node (topics, msg types, parameters) is identical to the
Dev-PC detector, so the downstream Nav2 pipeline does not care which
backend is underneath.

Model
-----
YOLOv8n exported at 320x320 on the Dev PC:
    yolo export model=yolov8n.pt format=onnx imgsz=320 opset=17 simplify=True
Drop the resulting `yolov8n_320.onnx` into `af_perception/models/`.
If `model_path` is absolute and exists, it takes precedence; otherwise
the node looks up `<package_share>/models/<model_name>`.

Performance envelope (Pi 5, single thread, 320x320)
---------------------------------------------------
  ~3–5 Hz end-to-end (preprocess + inference + post-process)
  ~50–70 MB resident
  <15% of one core — headroom for Nav2 and SLAM to co-exist.
"""
import os
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)
from cv_bridge import CvBridge


COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
    'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
    'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
    'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
    'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
    'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
    'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
    'scissors', 'teddy bear', 'hair drier', 'toothbrush',
]


class YoloOnnxNode(Node):
    def __init__(self):
        super().__init__('yolo_onnx_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('model_name', 'yolov8n_320.onnx')
        self.declare_parameter('input_size', 320)
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('num_threads', 2)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        # Integer class-index filter. Any negative value or an empty list
        # means "all COCO classes". Kept as INTEGER_ARRAY so `classes: []`
        # in YAML doesn't trip rclpy's type inference.
        self.declare_parameter('classes', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('process_every_n', 1)

        self._imgsz = int(self.get_parameter('input_size').value)
        self._conf = float(self.get_parameter('confidence_threshold').value)
        self._iou = float(self.get_parameter('iou_threshold').value)
        num_threads = int(self.get_parameter('num_threads').value)
        raw_filter = self.get_parameter_or(
            'classes', Parameter('classes', value=[])
        ).value or []
        raw_filter = [int(c) for c in raw_filter if int(c) >= 0]
        self._class_filter = set(raw_filter) if raw_filter else None
        self._process_every_n = max(1, int(self.get_parameter('process_every_n').value))
        self._frame_counter = 0

        self._session, self._input_name, self._output_name = self._load_model(num_threads)

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
        self._sub = self.create_subscription(
            Image,
            str(self.get_parameter('input_topic').value),
            self._cb,
            qos,
        )

        self._frames = 0
        self._inference_s = 0.0
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f'yolo_onnx_node ready — imgsz={self._imgsz}, '
            f'conf={self._conf}, threads={num_threads}, '
            f'process_every_n={self._process_every_n}'
        )

    def _resolve_model_path(self):
        explicit = str(self.get_parameter('model_path').value).strip()
        if explicit and os.path.isfile(explicit):
            return explicit
        name = str(self.get_parameter('model_name').value).strip()
        try:
            share = get_package_share_directory('af_perception')
            candidate = os.path.join(share, 'models', name)
            if os.path.isfile(candidate):
                return candidate
        except Exception:  # noqa: BLE001
            pass
        if explicit:
            self.get_logger().warn(
                f'model_path "{explicit}" not found on disk'
            )
        return None

    def _load_model(self, num_threads: int):
        try:
            import onnxruntime as ort
        except ImportError as exc:  # noqa: BLE001
            self.get_logger().fatal(
                'onnxruntime not installed on the Pi. In the container:\n'
                '    pip install --user onnxruntime'
            )
            raise SystemExit(1) from exc

        model_path = self._resolve_model_path()
        if model_path is None:
            self.get_logger().fatal(
                'YOLO ONNX model not found. Run af_perception/scripts/export_onnx.sh '
                'on the Dev PC and rsync the resulting yolov8n_320.onnx into '
                'af_perception/models/ (or pass model_path:=/abs/path.onnx).'
            )
            raise SystemExit(1)

        sess_opts = ort.SessionOptions()
        sess_opts.intra_op_num_threads = max(1, num_threads)
        sess_opts.inter_op_num_threads = 1
        sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        self.get_logger().info(f'loading ONNX model: {model_path}')
        session = ort.InferenceSession(
            model_path,
            sess_options=sess_opts,
            providers=['CPUExecutionProvider'],
        )
        input_name = session.get_inputs()[0].name
        output_name = session.get_outputs()[0].name
        input_shape = session.get_inputs()[0].shape
        self.get_logger().info(
            f'ONNX input "{input_name}" shape={input_shape} '
            f'-> output "{output_name}"'
        )
        return session, input_name, output_name

    def _preprocess(self, frame: np.ndarray):
        """Letterbox-resize `frame` to (imgsz, imgsz) and return NCHW float32.

        Returns the padded image plus the inverse transform
        `(scale, pad_x, pad_y)` so post-processing can map detections back
        to the original image coordinates.
        """
        h, w = frame.shape[:2]
        scale = self._imgsz / max(h, w)
        new_w, new_h = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        canvas = np.full((self._imgsz, self._imgsz, 3), 114, dtype=np.uint8)
        pad_x = (self._imgsz - new_w) // 2
        pad_y = (self._imgsz - new_h) // 2
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized

        # BGR -> RGB, HWC -> CHW, [0,255] -> [0,1]
        blob = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[None, ...]
        return np.ascontiguousarray(blob), scale, pad_x, pad_y

    def _postprocess(self, raw_output: np.ndarray, scale: float,
                     pad_x: int, pad_y: int, orig_h: int, orig_w: int):
        """Decode YOLOv8 ONNX output into (xyxy, conf, class_id) lists.

        `raw_output` has shape (1, 4+nc, num_anchors). For YOLOv8 the first
        four rows are xywh in the letterboxed input-image coordinate system
        and the remaining `nc` rows are per-class confidences already
        sigmoid-activated.
        """
        preds = raw_output[0].T  # (num_anchors, 4 + nc)
        if preds.shape[0] == 0:
            return [], [], []

        class_scores = preds[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[np.arange(len(class_ids)), class_ids]

        mask = confidences >= self._conf
        if self._class_filter is not None:
            mask &= np.isin(class_ids, list(self._class_filter))
        if not np.any(mask):
            return [], [], []

        preds = preds[mask]
        confidences = confidences[mask]
        class_ids = class_ids[mask]

        # xywh (centre) in letterbox space -> xyxy in original image space.
        cx, cy, bw, bh = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]
        x1 = (cx - bw * 0.5 - pad_x) / scale
        y1 = (cy - bh * 0.5 - pad_y) / scale
        x2 = (cx + bw * 0.5 - pad_x) / scale
        y2 = (cy + bh * 0.5 - pad_y) / scale
        x1 = np.clip(x1, 0, orig_w - 1)
        y1 = np.clip(y1, 0, orig_h - 1)
        x2 = np.clip(x2, 0, orig_w - 1)
        y2 = np.clip(y2, 0, orig_h - 1)

        # cv2.dnn.NMSBoxes expects (x, y, w, h)
        nms_boxes = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1)
        keep = cv2.dnn.NMSBoxes(
            nms_boxes.tolist(),
            confidences.tolist(),
            self._conf,
            self._iou,
        )
        if len(keep) == 0:
            return [], [], []
        keep = np.asarray(keep).flatten()

        xyxy = np.stack([x1[keep], y1[keep], x2[keep], y2[keep]], axis=1)
        return xyxy, confidences[keep], class_ids[keep]

    def _cb(self, msg: Image):
        self._frame_counter += 1
        if self._frame_counter % self._process_every_n != 0:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return

        orig_h, orig_w = frame.shape[:2]
        blob, scale, pad_x, pad_y = self._preprocess(frame)

        t0 = time.perf_counter()
        raw = self._session.run([self._output_name], {self._input_name: blob})[0]
        self._inference_s += time.perf_counter() - t0
        self._frames += 1

        xyxy, confs, class_ids = self._postprocess(
            raw, scale, pad_x, pad_y, orig_h, orig_w
        )

        det_array = Detection2DArray()
        det_array.header = msg.header
        for (x1, y1, x2, y2), conf, cls_id in zip(xyxy, confs, class_ids):
            det = Detection2D()
            det.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = (
                COCO_CLASSES[int(cls_id)]
                if 0 <= int(cls_id) < len(COCO_CLASSES)
                else str(int(cls_id))
            )
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

    def _log_stats(self):
        if self._frames == 0:
            self.get_logger().info('yolo_onnx: no frames in the last 5 s')
            return
        fps = self._frames / 5.0
        avg_ms = (self._inference_s / self._frames) * 1000.0
        self.get_logger().info(
            f'yolo_onnx: {fps:.1f} FPS, avg inference {avg_ms:.1f} ms'
        )
        self._frames = 0
        self._inference_s = 0.0


def main():
    rclpy.init()
    node = YoloOnnxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
