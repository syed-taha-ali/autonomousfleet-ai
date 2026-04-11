#!/usr/bin/env python3
# coding=utf-8
"""Mecanum wheel inverse kinematics (ported from HiWonder inspo)."""
import math

from af_msgs.msg import MotorsState, MotorState


class MecanumChassis:
    """ABAB mecanum layout used on the MentorPi M1.

    Wheel layout (top-down view, +x forward, +y left):

        motor1 (FL) |      | motor3 (FR)
                    |      |
        motor2 (RL) |      | motor4 (RR)
    """

    def __init__(self, wheelbase=0.1368, track_width=0.1410, wheel_diameter=0.065):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_convert(self, speed):
        """Convert linear wheel speed (m/s) to revolutions per second."""
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self, linear_x, linear_y, angular_z):
        """Compute per-motor rps for a body-frame twist (vx, vy, wz)."""
        half_sum = (self.wheelbase + self.track_width) / 2.0
        motor1 = linear_x - linear_y - angular_z * half_sum
        motor2 = linear_x + linear_y - angular_z * half_sum
        motor3 = linear_x + linear_y + angular_z * half_sum
        motor4 = linear_x - linear_y + angular_z * half_sum
        # Sign convention matches the HiWonder board mounting.
        rps = [self.speed_convert(v) for v in (-motor1, -motor2, motor3, motor4)]
        msg = MotorsState()
        msg.data = [MotorState(id=i + 1, rps=float(v)) for i, v in enumerate(rps)]
        return msg
