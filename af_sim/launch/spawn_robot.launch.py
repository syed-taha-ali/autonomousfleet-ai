#!/usr/bin/env python3
"""Spawn a single namespaced MentorPi robot into a running Gazebo world.

Usage:
  ros2 launch af_sim spawn_robot.launch.py namespace:=robot_0 x:=0.0 y:=0.0
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def _launch_setup(context):
    ns = LaunchConfiguration('namespace').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    desc_dir = get_package_share_directory('af_description')
    xacro_file = os.path.join(desc_dir, 'urdf', 'mentorpi_sim.xacro')

    import xacro
    robot_desc = xacro.process_file(
        xacro_file, mappings={'robot_namespace': ns}).toxml()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', ns,
            '-topic', f'/{ns}/robot_description',
            '-x', x,
            '-y', y,
            '-z', '0.05',
            '-Y', yaw,
        ],
        output='screen',
    )

    return [robot_state_pub, spawn_entity]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_0'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_launch_setup),
    ])
