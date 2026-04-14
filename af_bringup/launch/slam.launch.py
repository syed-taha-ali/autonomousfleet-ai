#!/usr/bin/env python3
"""Full robot bringup + SLAM mapping or AMCL localisation.

Usage:
  # Mapping mode (default):
  ros2 launch af_bringup slam.launch.py

  # Localisation mode with a saved map:
  ros2 launch af_bringup slam.launch.py mode:=localization map:=/path/to/map.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar').perform(context).lower() in ('true', '1', 'yes')
    entities = []

    entities.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('af_bringup'), 'launch', 'robot.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    ))

    if start_lidar:
        entities.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('af_bringup'), 'launch', 'lidar.launch.py'])),
        ))

    if mode == 'mapping':
        entities.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('af_slam'), 'launch',
                'slam_mapping.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ))
    elif mode == 'localization':
        map_file = LaunchConfiguration('map').perform(context)
        entities.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('af_slam'), 'launch',
                'slam_localization.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file,
            }.items(),
        ))

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mode', default_value='mapping',
                              description='SLAM mode: "mapping" or "localization"'),
        DeclareLaunchArgument('start_lidar', default_value='true',
                              description='Start MS200 LiDAR driver + /scan_raw -> /scan relay'),
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML (required for localization mode)'),
        OpaqueFunction(function=_launch_setup),
    ])
