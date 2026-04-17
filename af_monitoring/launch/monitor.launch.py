#!/usr/bin/env python3
"""Launch the performance monitor node.

Usage:
  ros2 launch af_monitoring monitor.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='af_monitoring',
            executable='perf_monitor_node',
            name='perf_monitor',
            output='screen',
            parameters=[{'publish_rate_hz': 1.0}],
        ),
    ])
