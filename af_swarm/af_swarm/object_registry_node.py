#!/usr/bin/env python3
"""Shared object registry: aggregates detections from all robots.

Subscribes to per-robot detection topics, deduplicates by proximity,
and publishes the canonical list to /fleet/objects. Also provides a
/fleet/objects_list latched topic with the full list on demand.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from af_msgs.msg import DetectedObject
from std_msgs.msg import String


class ObjectRegistryNode(Node):
    def __init__(self):
        super().__init__('object_registry')

        self.declare_parameter('num_robots', 3)
        self.declare_parameter('robot_prefix', 'robot_')
        self.declare_parameter('dedup_radius_m', 0.5)

        num = self.get_parameter('num_robots').value
        prefix = self.get_parameter('robot_prefix').value
        self._dedup_radius = self.get_parameter('dedup_radius_m').value

        self._objects: list[DetectedObject] = []

        self._fleet_pub = self.create_publisher(
            DetectedObject, '/fleet/objects', 10)

        transient = QoSProfile(
            depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._list_pub = self.create_publisher(
            String, '/fleet/objects_list', transient)

        for i in range(num):
            ns = f'{prefix}{i}'
            self.create_subscription(
                DetectedObject, f'/{ns}/detected_object',
                self._on_detection, 10)

        self.get_logger().info(
            f'Object registry ready: listening to {num} robots, '
            f'dedup radius {self._dedup_radius} m')

    def _is_duplicate(self, msg: DetectedObject) -> bool:
        for obj in self._objects:
            dx = obj.position.x - msg.position.x
            dy = obj.position.y - msg.position.y
            if math.hypot(dx, dy) < self._dedup_radius:
                return True
        return False

    def _on_detection(self, msg: DetectedObject):
        if self._is_duplicate(msg):
            self.get_logger().debug(
                f'Duplicate detection ignored: {msg.object_class} '
                f'at ({msg.position.x:.2f}, {msg.position.y:.2f})')
            return

        self._objects.append(msg)
        self._fleet_pub.publish(msg)

        list_msg = String()
        entries = [
            f'{o.object_class}@({o.position.x:.2f},{o.position.y:.2f}) '
            f'by {o.detected_by}' for o in self._objects
        ]
        list_msg.data = '; '.join(entries)
        self._list_pub.publish(list_msg)

        self.get_logger().info(
            f'New object #{len(self._objects)}: {msg.object_class} '
            f'at ({msg.position.x:.2f}, {msg.position.y:.2f}) '
            f'by {msg.detected_by}')


def main():
    rclpy.init()
    node = ObjectRegistryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
