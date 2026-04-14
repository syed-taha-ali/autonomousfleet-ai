#!/usr/bin/env python3
"""Full robot bringup + AMCL localisation + Nav2 navigation.

One-shot launcher for autonomous navigation in a pre-mapped room. Composes:
  - af_bringup/robot.launch.py        (HAL + description + EKF + camera)
  - af_bringup/lidar.launch.py        (MS200 + scan sanitiser)          [optional]
  - af_slam/slam_localization.launch  (map_server + AMCL on saved map)
  - af_navigation/navigation.launch   (Nav2 stack)

Usage:
  ros2 launch af_bringup nav2.launch.py map:=/path/to/map.yaml

The map argument is required — Nav2 needs an occupancy grid to plan on. If you
want to navigate while building a map instead, use `slam.launch.py
mode:=mapping` and then launch `af_navigation navigation.launch.py` on its own.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar').perform(context).lower() in ('true', '1', 'yes')
    start_robot = LaunchConfiguration('start_robot').perform(context).lower() in ('true', '1', 'yes')
    map_file = LaunchConfiguration('map').perform(context)
    entities = []

    if start_robot:
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

    entities.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('af_slam'), 'launch',
            'slam_localization.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
        }.items(),
    ))

    entities.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('af_navigation'), 'launch',
            'navigation.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    ))

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_robot', default_value='true',
                              description='Start HAL + description + EKF (robot.launch.py).'),
        DeclareLaunchArgument('start_lidar', default_value='true',
                              description='Start MS200 LiDAR driver + scan_raw relay.'),
        DeclareLaunchArgument('map', description='Full path to the map YAML file.'),
        OpaqueFunction(function=_launch_setup),
    ])
