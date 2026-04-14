#!/usr/bin/env python3
"""MS200 LiDAR driver + scan sanitiser.

The HiWonder factory `oradar_lidar` driver publishes partial scan chunks on
`/scan_raw`. Its angle_max / angle_increment / ranges.size are off-by-one,
which slam_toolbox (Karto) rejects with "LaserRangeScan contains N range
readings, expected N+1". `af_hal/scan_sanitizer_node` republishes onto
`/scan` with angle_max recomputed from len(ranges) so downstream Nav2 /
slam_toolbox consumers accept every scan.

Requires the factory workspace to be sourced (it owns `oradar_lidar`):
source /home/ubuntu/ros2_ws/install/setup.bash before launching, or overlay
it on top of af_bringup's workspace.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_frame = LaunchConfiguration('lidar_frame', default='laser_frame')

    ms200 = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='MS200',
        output='screen',
        parameters=[{
            'device_model': 'MS200',
            'frame_id': lidar_frame,
            'scan_topic': 'MS200/scan',
            'port_name': '/dev/ldlidar',
            'baudrate': 230400,
            'angle_min': 0.0,
            'angle_max': 360.0,
            'range_min': 0.05,
            'range_max': 12.0,
            'clockwise': False,
            'motor_speed': 15,
        }],
        remappings=[('/MS200/scan', '/scan_raw')],
    )

    scan_sanitizer = Node(
        package='af_hal',
        executable='scan_sanitizer_node',
        name='scan_sanitizer',
        output='screen',
        parameters=[{
            'input_topic': '/scan_raw',
            'output_topic': '/scan',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame'),
        ms200,
        scan_sanitizer,
    ])
