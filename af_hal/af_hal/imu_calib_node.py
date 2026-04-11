#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Static IMU bias calibration.

At startup the node waits until it has collected ``calibration_samples``
stationary readings, averages them to estimate gyro and accel biases, then
begins publishing ``/imu_corrected`` with the bias removed. The gravity
component on the Z axis is preserved (we subtract the mean of X/Y accel, and
subtract only the non-gravity part of Z so that the calibrated sample still
reflects ~9.81 m/s^2 downward).

This replaces the inspo project's C++ ``imu_calib`` package: the math is
trivial and keeps the stack Python-only per the plan's language policy.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

GRAVITY = 9.80665


class ImuCalibNode(Node):
    def __init__(self):
        super().__init__('imu_calib')

        self.declare_parameter('input_topic', '/ros_robot_controller/imu_raw')
        self.declare_parameter('output_topic', '/imu_corrected')
        self.declare_parameter('calibration_samples', 200)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.target_samples = int(self.get_parameter('calibration_samples').value)

        self.samples = 0
        self.gx_sum = 0.0
        self.gy_sum = 0.0
        self.gz_sum = 0.0
        self.ax_sum = 0.0
        self.ay_sum = 0.0
        self.az_sum = 0.0

        self.gx_bias = 0.0
        self.gy_bias = 0.0
        self.gz_bias = 0.0
        self.ax_bias = 0.0
        self.ay_bias = 0.0
        self.az_bias = 0.0
        self.calibrated = False

        self.pub = self.create_publisher(Imu, out_topic, 10)
        self.sub = self.create_subscription(Imu, in_topic, self._on_imu, 50)

        self.get_logger().info(
            f'imu_calib waiting for {self.target_samples} stationary samples on {in_topic}'
        )

    def _on_imu(self, msg: Imu):
        if not self.calibrated:
            self.gx_sum += msg.angular_velocity.x
            self.gy_sum += msg.angular_velocity.y
            self.gz_sum += msg.angular_velocity.z
            self.ax_sum += msg.linear_acceleration.x
            self.ay_sum += msg.linear_acceleration.y
            self.az_sum += msg.linear_acceleration.z
            self.samples += 1
            if self.samples >= self.target_samples:
                n = float(self.samples)
                self.gx_bias = self.gx_sum / n
                self.gy_bias = self.gy_sum / n
                self.gz_bias = self.gz_sum / n
                self.ax_bias = self.ax_sum / n
                self.ay_bias = self.ay_sum / n
                # Preserve gravity on Z: bias is whatever's left after removing
                # ~9.81 m/s^2. If the robot isn't level this absorbs that tilt.
                self.az_bias = (self.az_sum / n) - GRAVITY
                self.calibrated = True
                self.get_logger().info(
                    'imu_calib done: '
                    f'gyro_bias=({self.gx_bias:.4f},{self.gy_bias:.4f},{self.gz_bias:.4f}) '
                    f'accel_bias=({self.ax_bias:.4f},{self.ay_bias:.4f},{self.az_bias:.4f})'
                )
            return

        corrected = Imu()
        corrected.header = msg.header
        corrected.orientation = msg.orientation
        corrected.orientation_covariance = msg.orientation_covariance
        corrected.angular_velocity.x = msg.angular_velocity.x - self.gx_bias
        corrected.angular_velocity.y = msg.angular_velocity.y - self.gy_bias
        corrected.angular_velocity.z = msg.angular_velocity.z - self.gz_bias
        corrected.angular_velocity_covariance = msg.angular_velocity_covariance
        corrected.linear_acceleration.x = msg.linear_acceleration.x - self.ax_bias
        corrected.linear_acceleration.y = msg.linear_acceleration.y - self.ay_bias
        corrected.linear_acceleration.z = msg.linear_acceleration.z - self.az_bias
        corrected.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(corrected)


def main():
    rclpy.init()
    node = ImuCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
