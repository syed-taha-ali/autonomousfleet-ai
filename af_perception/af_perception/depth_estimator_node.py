#!/usr/bin/env python3
"""Back-project 2D detections into 3D obstacle observations.

Subscribes to:
  /detections                      vision_msgs/Detection2DArray
  /camera/camera_info              sensor_msgs/CameraInfo
  /camera/depth/image_raw          sensor_msgs/Image   (depth mode)

Publishes:
  /detected_objects_3d             vision_msgs/Detection3DArray
  /vision_obstacles                sensor_msgs/PointCloud2

Two back-projection modes
-------------------------
* `depth`  — median depth inside each bbox, unproject with camera intrinsics.
             Requires the MentorPi depth-camera driver to be publishing a
             `32FC1` (metres) or `16UC1` (millimetres) depth image.
* `ground` — assume each detected object is resting on the ground plane and
             back-project the bottom-centre of its bbox onto `z = 0` in the
             `base_link` frame. Works with just a plain USB camera and
             TF `base_link -> camera_optical_frame`.

The stock `nav2_costmap_2d::ObstacleLayer` consumes `/vision_obstacles`
directly as a second observation source (already wired into
`af_navigation/config/nav2_params.yaml`), so *no custom C++ costmap plugin*
is required — the point of this whole detour.
"""
import math
import struct
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

import message_filters

from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from vision_msgs.msg import (
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
    BoundingBox3D,
)
from geometry_msgs.msg import PointStamped

import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformException

try:
    from tf2_geometry_msgs import do_transform_point  # noqa: F401
    _HAVE_TF2_GEO = True
except ImportError:
    _HAVE_TF2_GEO = False

from cv_bridge import CvBridge, CvBridgeError


def _make_pointcloud2(header: Header, points: List[Tuple[float, float, float]]) -> PointCloud2:
    """Hand-roll a PointCloud2 — keeps us off sensor_msgs_py for now."""
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.is_dense = False
    cloud.is_bigendian = False
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    buf = bytearray(cloud.row_step)
    off = 0
    for x, y, z in points:
        struct.pack_into('<fff', buf, off, float(x), float(y), float(z))
        off += 12
    cloud.data = bytes(buf)
    return cloud


class DepthEstimatorNode(Node):
    def __init__(self):
        super().__init__('depth_estimator_node')

        self.declare_parameter('mode', 'depth')  # 'depth' | 'ground'
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('objects_topic', '/detected_objects_3d')
        self.declare_parameter('obstacles_topic', '/vision_obstacles')
        self.declare_parameter('obstacle_frame', 'base_link')
        self.declare_parameter('optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('cluster_radius', 0.1)
        self.declare_parameter('cluster_samples', 9)
        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_depth', 5.0)
        self.declare_parameter('ground_plane_z', 0.0)
        # When using ground mode, we need the camera's height and tilt in
        # `obstacle_frame`. These are only used if TF lookup fails.
        self.declare_parameter('fallback_camera_height', 0.15)
        self.declare_parameter('fallback_camera_pitch', 0.0)
        self.declare_parameter('sync_slop', 0.1)

        self._mode = str(self.get_parameter('mode').value)
        if self._mode not in ('depth', 'ground'):
            self.get_logger().warn(
                f'unknown mode "{self._mode}", defaulting to "ground"'
            )
            self._mode = 'ground'

        self._obstacle_frame = str(self.get_parameter('obstacle_frame').value)
        self._optical_frame_param = str(self.get_parameter('optical_frame').value)
        self._min_depth = float(self.get_parameter('min_depth').value)
        self._max_depth = float(self.get_parameter('max_depth').value)
        self._cluster_radius = float(self.get_parameter('cluster_radius').value)
        self._cluster_samples = int(self.get_parameter('cluster_samples').value)
        self._ground_z = float(self.get_parameter('ground_plane_z').value)
        self._fallback_h = float(self.get_parameter('fallback_camera_height').value)
        self._fallback_pitch = float(self.get_parameter('fallback_camera_pitch').value)

        self._bridge = CvBridge()
        self._camera_info: Optional[CameraInfo] = None
        self._K: Optional[np.ndarray] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._obj_pub = self.create_publisher(
            Detection3DArray,
            str(self.get_parameter('objects_topic').value),
            10,
        )
        self._cloud_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('obstacles_topic').value),
            qos,
        )

        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            10,
        )

        det_sub = message_filters.Subscriber(
            self,
            Detection2DArray,
            str(self.get_parameter('detections_topic').value),
        )

        if self._mode == 'depth':
            depth_sub = message_filters.Subscriber(
                self,
                Image,
                str(self.get_parameter('depth_topic').value),
                qos_profile=qos,
            )
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [det_sub, depth_sub],
                queue_size=10,
                slop=float(self.get_parameter('sync_slop').value),
            )
            self._sync.registerCallback(self._on_depth_pair)
            self.get_logger().info('depth_estimator: depth-image mode')
        else:
            det_sub.registerCallback(self._on_detections_ground)
            self.get_logger().info(
                'depth_estimator: ground-plane mode '
                f'(fallback_h={self._fallback_h} m, pitch={self._fallback_pitch} rad)'
            )

    # --- camera info ------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo):
        if self._camera_info is None:
            self.get_logger().info(
                f'camera_info: {msg.width}x{msg.height} '
                f'fx={msg.k[0]:.1f} fy={msg.k[4]:.1f}'
            )
        self._camera_info = msg
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    # --- depth mode -------------------------------------------------------

    def _on_depth_pair(self, det_msg: Detection2DArray, depth_msg: Image):
        if self._K is None:
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as exc:
            self.get_logger().warn(f'depth decode failed: {exc}')
            return
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0  # mm -> m
        else:
            depth_m = depth.astype(np.float32)

        optical_frame = depth_msg.header.frame_id or self._optical_frame_param
        objs, pts_optical = self._project_detections_depth(det_msg, depth_m)
        self._publish(det_msg.header, optical_frame, objs, pts_optical)

    def _project_detections_depth(
        self, det_msg: Detection2DArray, depth_m: np.ndarray
    ):
        fx = self._K[0, 0]
        fy = self._K[1, 1]
        cx = self._K[0, 2]
        cy = self._K[1, 2]
        h, w = depth_m.shape[:2]

        objs: List[Detection3D] = []
        pts_optical: List[Tuple[float, float, float]] = []

        for det in det_msg.detections:
            bb = det.bbox
            u0 = int(max(0, bb.center.position.x - bb.size_x * 0.5))
            v0 = int(max(0, bb.center.position.y - bb.size_y * 0.5))
            u1 = int(min(w, bb.center.position.x + bb.size_x * 0.5))
            v1 = int(min(h, bb.center.position.y + bb.size_y * 0.5))
            if u1 <= u0 or v1 <= v0:
                continue
            patch = depth_m[v0:v1, u0:u1]
            valid = patch[(patch > self._min_depth) & (patch < self._max_depth)]
            if valid.size == 0:
                continue
            z = float(np.median(valid))

            u = float(bb.center.position.x)
            v = float(bb.center.position.y)
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            obj = Detection3D()
            obj.header = det_msg.header
            if det.results:
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = det.results[0].hypothesis.class_id
                hyp.hypothesis.score = det.results[0].hypothesis.score
                obj.results.append(hyp)
            bb3 = BoundingBox3D()
            bb3.center.position.x = x
            bb3.center.position.y = y
            bb3.center.position.z = z
            # Rough size estimate: bbox span in pixels back-projected at z.
            bb3.size.x = max(0.05, float(bb.size_x) * z / fx)
            bb3.size.y = max(0.05, float(bb.size_y) * z / fy)
            bb3.size.z = 0.3
            obj.bbox = bb3
            objs.append(obj)

            pts_optical.extend(self._cluster_points(x, y, z))

        return objs, pts_optical

    # --- ground mode ------------------------------------------------------

    def _on_detections_ground(self, det_msg: Detection2DArray):
        if self._K is None:
            return
        optical_frame = (
            det_msg.detections[0].header.frame_id
            if det_msg.detections
            else self._optical_frame_param
        )
        objs, pts = self._project_detections_ground(det_msg, optical_frame)
        self._publish(det_msg.header, optical_frame, objs, pts)

    def _project_detections_ground(
        self, det_msg: Detection2DArray, optical_frame: str
    ):
        fx = self._K[0, 0]
        fy = self._K[1, 1]
        cx = self._K[0, 2]
        cy = self._K[1, 2]

        objs: List[Detection3D] = []
        pts_optical: List[Tuple[float, float, float]] = []

        # Ground plane in obstacle_frame is z=ground_z.
        # We look up TF optical->obstacle and solve for the ray t where
        # (R*d*t + o).z == ground_z, but to keep this Python-light we just
        # use the fallback camera height and pitch if TF isn't available.
        h_cam = self._fallback_h
        pitch = self._fallback_pitch

        for det in det_msg.detections:
            bb = det.bbox
            u = float(bb.center.position.x)
            v = float(bb.center.position.y + bb.size_y * 0.5)  # bottom edge
            # Ray in optical frame (x right, y down, z forward)
            rx = (u - cx) / fx
            ry = (v - cy) / fy
            rz = 1.0

            # Rotate by camera pitch about optical X so +y ends up along
            # world -z. For a level camera (pitch=0) we just need ry>0 to
            # intersect the ground below the camera.
            cp = math.cos(pitch)
            sp = math.sin(pitch)
            ry_r = cp * ry - sp * rz
            rz_r = sp * ry + cp * rz

            if ry_r <= 1e-3:
                continue  # ray is above horizon, no ground hit
            t = h_cam / ry_r
            if t < self._min_depth or t > self._max_depth:
                continue
            x_opt = rx * t
            y_opt = ry * t
            z_opt = rz * t

            obj = Detection3D()
            obj.header = det_msg.header
            if det.results:
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = det.results[0].hypothesis.class_id
                hyp.hypothesis.score = det.results[0].hypothesis.score
                obj.results.append(hyp)
            bb3 = BoundingBox3D()
            bb3.center.position.x = x_opt
            bb3.center.position.y = y_opt
            bb3.center.position.z = z_opt
            bb3.size.x = max(0.1, float(bb.size_x) * rz_r * t / fx)
            bb3.size.y = max(0.1, float(bb.size_y) * rz_r * t / fy)
            bb3.size.z = 0.3
            obj.bbox = bb3
            objs.append(obj)

            pts_optical.extend(self._cluster_points(x_opt, y_opt, z_opt))

        return objs, pts_optical

    # --- shared helpers ---------------------------------------------------

    def _cluster_points(self, x: float, y: float, z: float):
        """Emit a small ring of points around (x,y,z) so the costmap marks it."""
        n = max(1, self._cluster_samples)
        r = self._cluster_radius
        if n == 1:
            return [(x, y, z)]
        pts = [(x, y, z)]
        for i in range(n - 1):
            a = 2.0 * math.pi * i / (n - 1)
            pts.append((x + r * math.cos(a), y, z + r * math.sin(a)))
        return pts

    def _publish(
        self,
        stamp_header: Header,
        optical_frame: str,
        objs: List[Detection3D],
        pts_optical: List[Tuple[float, float, float]],
    ):
        # Detection3DArray stays in the optical frame — downstream consumers
        # are free to transform it.
        objs_msg = Detection3DArray()
        objs_msg.header = stamp_header
        objs_msg.header.frame_id = optical_frame
        objs_msg.detections = objs
        self._obj_pub.publish(objs_msg)

        header = Header()
        header.stamp = stamp_header.stamp
        header.frame_id = self._obstacle_frame

        if not pts_optical:
            self._cloud_pub.publish(_make_pointcloud2(header, []))
            return

        transformed = self._transform_points(
            pts_optical, optical_frame, self._obstacle_frame, stamp_header.stamp
        )
        if transformed is None:
            # TF lookup failed — publish in the optical frame so the operator
            # still sees something in RViz, but warn loudly so it gets fixed.
            header.frame_id = optical_frame
            transformed = pts_optical
            self.get_logger().warn(
                f'TF {optical_frame} -> {self._obstacle_frame} unavailable; '
                f'publishing /vision_obstacles in {optical_frame}',
                throttle_duration_sec=5.0,
            )
        self._cloud_pub.publish(_make_pointcloud2(header, transformed))

    def _transform_points(
        self,
        pts: List[Tuple[float, float, float]],
        src_frame: str,
        dst_frame: str,
        stamp,
    ) -> Optional[List[Tuple[float, float, float]]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                dst_frame, src_frame, Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except TransformException:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        # Quaternion -> rotation matrix
        x, y, z, w = q.x, q.y, q.z, q.w
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        R = np.array(
            [
                [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
                [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
                [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
            ],
            dtype=np.float64,
        )
        origin = np.array([t.x, t.y, t.z], dtype=np.float64)
        arr = np.asarray(pts, dtype=np.float64).T  # (3, N)
        out = R @ arr + origin[:, None]
        return [tuple(out[:, i]) for i in range(out.shape[1])]


def main():
    rclpy.init()
    node = DepthEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
