#!/usr/bin/env python3
"""On-robot (Phase 4.1) perception pipeline.

Runs two nodes inside the MentorPi Docker container on the Pi:
  1. yolo_onnx_node        — ONNX Runtime YOLOv8n, `/camera/image_raw`
                              (Pi-local) -> `/detections`
  2. depth_estimator_node  — `/detections` -> `/detected_objects_3d` and
                              (optional) `/vision_obstacles` PointCloud2

This deliberately has no image_decompressor_node: the Pi already has the
raw BGR frames from usb_cam, so round-tripping through JPEG compression
and decompression just burns CPU for nothing.

Usage:
  ros2 launch af_perception perception_pi.launch.py
  ros2 launch af_perception perception_pi.launch.py \\
      model_path:=/abs/path/to/best_320.onnx confidence_threshold:=0.4
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('af_perception'), 'config', 'perception_pi.yaml'
    ])

    model_path = LaunchConfiguration('model_path')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    input_size = LaunchConfiguration('input_size')

    detector = Node(
        package='af_perception',
        executable='yolo_onnx_node',
        name='yolo_onnx_node',
        output='screen',
        parameters=[config, {
            'model_path': model_path,
            'confidence_threshold': confidence_threshold,
            'input_size': input_size,
        }],
    )

    depth = Node(
        package='af_perception',
        executable='depth_estimator_node',
        name='depth_estimator_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('confidence_threshold', default_value='0.35'),
        DeclareLaunchArgument('input_size', default_value='320'),
        detector,
        depth,
    ])
