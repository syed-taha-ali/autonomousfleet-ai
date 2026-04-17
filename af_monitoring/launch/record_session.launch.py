#!/usr/bin/env python3
"""Record a rosbag of key topics for offline analysis.

Usage:
  ros2 launch af_monitoring record_session.launch.py
  ros2 launch af_monitoring record_session.launch.py bag_dir:=/tmp/bags
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


_TOPICS = [
    '/scan',
    '/odom',
    '/imu',
    '/cmd_vel',
    '/map',
    '/tf',
    '/tf_static',
    '/detections',
    '/mission/status',
    '/mission/command',
    '/diagnostics',
    '/camera/demo/compressed',
]


def _launch_setup(context):
    bag_dir = LaunchConfiguration('bag_dir').perform(context)
    os.makedirs(bag_dir, exist_ok=True)

    cmd = ['ros2', 'bag', 'record', '-o', bag_dir + '/session', '--max-bag-duration', '300']
    cmd.extend(_TOPICS)

    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    default_dir = os.path.join(os.getcwd(), 'rosbags')
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_dir', default_value=default_dir,
            description='Directory for rosbag output.'),
        OpaqueFunction(function=_launch_setup),
    ])
