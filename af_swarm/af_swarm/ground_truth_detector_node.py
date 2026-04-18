#!/usr/bin/env python3
"""Ground-truth object detector for Gazebo simulation.

Instead of running YOLO per robot (too expensive for 5 Nav2 instances),
this node checks each robot's pose against known object positions in the
world and simulates a detection when the object is within camera FOV and
range. Publishes DetectedObject on /<namespace>/detected_object.

Object positions are loaded from a YAML parameter list matching the
Gazebo world's target_obj_* poses.
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from af_msgs.msg import DetectedObject


class GroundTruthDetectorNode(Node):
    def __init__(self):
        super().__init__('ground_truth_detector')

        self.declare_parameter('detection_range', 2.0)
        self.declare_parameter('fov_deg', 60.0)
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('object_classes', ['obj_0', 'obj_1', 'obj_2', 'obj_3', 'obj_4'])
        self.declare_parameter('object_x', [3.0, -3.0, -2.0, 1.0, 2.5])
        self.declare_parameter('object_y', [2.0, -2.0, 2.2, -2.2, 0.5])

        self._range = self.get_parameter('detection_range').value
        self._fov = math.radians(self.get_parameter('fov_deg').value)
        self._ns = self.get_parameter('robot_namespace').value

        classes = self.get_parameter('object_classes').value
        xs = self.get_parameter('object_x').value
        ys = self.get_parameter('object_y').value
        self._objects = list(zip(classes, xs, ys))

        self._detected = set()
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

        self._det_pub = self.create_publisher(
            DetectedObject, 'detected_object', 10)

        self._odom_sub = self.create_subscription(
            Odometry, 'odom', self._on_odom, 10)

        self.create_timer(0.5, self._check_detections)

        self.get_logger().info(
            f'Ground truth detector ready: {len(self._objects)} objects, '
            f'range={self._range}m, fov={math.degrees(self._fov):.0f}deg')

    def _on_odom(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    def _check_detections(self):
        for cls, ox, oy in self._objects:
            if cls in self._detected:
                continue

            dx = ox - self._robot_x
            dy = oy - self._robot_y
            dist = math.hypot(dx, dy)
            if dist > self._range:
                continue

            angle_to_obj = math.atan2(dy, dx)
            angle_diff = abs(self._normalize_angle(
                angle_to_obj - self._robot_yaw))
            if angle_diff > self._fov / 2.0:
                continue

            self._detected.add(cls)

            msg = DetectedObject()
            msg.stamp = self.get_clock().now().to_msg()
            msg.object_class = cls
            msg.position = Point(x=ox, y=oy, z=0.0)
            msg.detected_by = self._ns or self.get_namespace().strip('/')
            msg.confidence = max(0.5, 1.0 - dist / self._range)
            self._det_pub.publish(msg)

            self.get_logger().info(
                f'Detected {cls} at ({ox:.1f}, {oy:.1f}), '
                f'dist={dist:.2f}m, conf={msg.confidence:.2f}')

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main():
    rclpy.init()
    node = GroundTruthDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
