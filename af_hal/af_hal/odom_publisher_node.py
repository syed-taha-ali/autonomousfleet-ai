#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mecanum velocity command + dead-reckoning odometry node.

Differences vs. HiWonder inspo:
- Mecanum-only (no Ackermann branch).
- Single ``/cmd_vel`` subscription (the inspo had both ``/cmd_vel`` and
  ``/controller/cmd_vel``, which caused dual-publisher conflicts). Teleop
  nodes should publish directly to ``/cmd_vel``.
- No ``MACHINE_TYPE`` environment variable -- hard-coded for MentorPi_Mecanum.
- Integrates lateral velocity (``linear_y``) into the pose, which the inspo
  dropped; mecanum odometry must include it.
- Hardcoded workspace paths removed; all config comes from node parameters.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, Pose2D, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

from af_hal.mecanum import MecanumChassis
from af_msgs.msg import MotorsState

ODOM_POSE_COVARIANCE = [
    1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e3,
]

ODOM_POSE_COVARIANCE_STOP = [
    1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e-9, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e-9,
]

ODOM_TWIST_COVARIANCE = ODOM_POSE_COVARIANCE
ODOM_TWIST_COVARIANCE_STOP = ODOM_POSE_COVARIANCE_STOP


def rpy_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation


class OdomPublisher(Node):
    def __init__(self, name='odom_publisher'):
        super().__init__(name)

        self.declare_parameter('pub_odom_topic', True)
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('linear_correction_factor', 1.00)
        self.declare_parameter('angular_correction_factor', 1.00)
        self.declare_parameter('wheelbase', 0.1368)
        self.declare_parameter('track_width', 0.1410)
        self.declare_parameter('wheel_diameter', 0.065)
        self.declare_parameter('max_linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 1.0)

        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.linear_factor = float(self.get_parameter('linear_correction_factor').value)
        self.angular_factor = float(self.get_parameter('angular_correction_factor').value)
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)

        self.chassis = MecanumChassis(
            wheelbase=float(self.get_parameter('wheelbase').value),
            track_width=float(self.get_parameter('track_width').value),
            wheel_diameter=float(self.get_parameter('wheel_diameter').value),
        )

        # Integrated pose state
        self.x = 0.0
        self.y = 0.0
        self.pose_yaw = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.last_time = None

        self.clock = self.get_clock()

        # Publishers: NOTE topic names are relative and get remapped or
        # namespaced in launch. Default names match the HAL plan:
        #   /odom_raw, /ros_robot_controller/set_motor
        self.motor_pub = self.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 1)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 1)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'set_pose', 1)

        # Subscribers: single cmd_vel input (consolidated from inspo).
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        self.create_subscription(Pose2D, 'set_odom', self.set_odom, 1)

        self.create_service(Trigger, 'controller/load_calibrate_param', self.load_calibrate_param)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        if self.get_parameter('pub_odom_topic').value:
            threading.Thread(target=self._odom_loop, daemon=True).start()

        self.get_logger().info(
            f'odom_publisher started (lin={self.linear_factor}, ang={self.angular_factor})'
        )

    # ------------------------------------------------------------------
    # Services

    def get_node_state(self, request, response):
        response.success = True
        return response

    def load_calibrate_param(self, request, response):
        self.linear_factor = float(self.get_parameter('linear_correction_factor').value)
        self.angular_factor = float(self.get_parameter('angular_correction_factor').value)
        self.get_logger().info('calibration params reloaded')
        response.success = True
        return response

    # ------------------------------------------------------------------
    # Commands

    def cmd_vel_callback(self, msg: Twist):
        vx = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        vy = max(-self.max_lin, min(self.max_lin, msg.linear.y))
        wz = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        self.linear_x = vx
        self.linear_y = vy
        self.angular_z = wz
        self.motor_pub.publish(self.chassis.set_velocity(vx, vy, wz))

    def set_odom(self, msg: Pose2D):
        self.x = msg.x
        self.y = msg.y
        self.pose_yaw = msg.theta
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.odom_frame_id
        pose.header.stamp = self.clock.now().to_msg()
        pose.pose.pose.position.x = self.x
        pose.pose.pose.position.y = self.y
        pose.pose.pose.orientation = rpy_to_quaternion(0.0, 0.0, self.pose_yaw)
        pose.pose.covariance = ODOM_POSE_COVARIANCE
        self.pose_pub.publish(pose)

    # ------------------------------------------------------------------
    # Odometry integration

    def _odom_loop(self):
        while rclpy.ok():
            now = time.time()
            dt = 0.0 if self.last_time is None else (now - self.last_time)
            self.last_time = now

            # Body-frame velocity -> world-frame delta (mecanum is holonomic).
            cos_y = math.cos(self.pose_yaw)
            sin_y = math.sin(self.pose_yaw)
            dx = (self.linear_x * cos_y - self.linear_y * sin_y) * dt
            dy = (self.linear_x * sin_y + self.linear_y * cos_y) * dt
            dyaw = self.angular_z * dt

            self.x += dx
            self.y += dy
            self.pose_yaw += dyaw

            odom = Odometry()
            odom.header.stamp = self.clock.now().to_msg()
            odom.header.frame_id = self.odom_frame_id
            odom.child_frame_id = self.base_frame_id
            odom.pose.pose.position.x = self.linear_factor * self.x
            odom.pose.pose.position.y = self.linear_factor * self.y
            odom.pose.pose.orientation = rpy_to_quaternion(0.0, 0.0, self.pose_yaw)
            odom.twist.twist.linear.x = self.linear_x
            odom.twist.twist.linear.y = self.linear_y
            odom.twist.twist.angular.z = self.angular_z * self.angular_factor

            stopped = (self.linear_x == 0.0 and self.linear_y == 0.0 and self.angular_z == 0.0)
            odom.pose.covariance = ODOM_POSE_COVARIANCE_STOP if stopped else ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE_STOP if stopped else ODOM_TWIST_COVARIANCE

            self.odom_pub.publish(odom)
            time.sleep(0.02)


def main():
    rclpy.init()
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
