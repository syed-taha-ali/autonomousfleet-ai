#!/usr/bin/env python3
"""Keyboard teleop for the robot (publishes directly to /cmd_vel)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[('/cmd_vel', '/cmd_vel')],
        ),
    ])
