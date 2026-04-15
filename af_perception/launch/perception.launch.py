#!/usr/bin/env python3
"""Launch the full Dev-PC perception pipeline.

Runs three nodes on the Dev PC:
  1. image_decompressor_node — JPEG CompressedImage -> Image (bgr8)
  2. yolo_detector_node      — YOLO on CUDA, Image -> Detection2DArray
  3. depth_estimator_node    — Detection2DArray -> Detection3DArray
                                + /vision_obstacles PointCloud2

The Pi must already be publishing /camera/image_raw/compressed and
/camera/camera_info over the shared DDS domain (af_bringup/robot.launch.py
handles this).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('af_perception'), 'config', 'perception.yaml'
    ])

    # Lets the operator quickly swap in a custom-trained weights file without
    # editing the yaml: `ros2 launch ... model_path:=/abs/path/best.pt`.
    model_path = LaunchConfiguration('model_path')
    mode = LaunchConfiguration('mode')
    device = LaunchConfiguration('device')
    publish_annotated = LaunchConfiguration('publish_annotated')

    decompressor = Node(
        package='af_perception',
        executable='image_decompressor_node',
        name='image_decompressor_node',
        output='screen',
        parameters=[config],
    )

    detector = Node(
        package='af_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[config, {
            'model_path': model_path,
            'device': device,
            'publish_annotated': publish_annotated,
        }],
    )

    depth = Node(
        package='af_perception',
        executable='depth_estimator_node',
        name='depth_estimator_node',
        output='screen',
        parameters=[config, {'mode': mode}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('mode', default_value='ground',
                              description='ground | depth'),
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('publish_annotated', default_value='true'),
        decompressor,
        detector,
        depth,
    ])
