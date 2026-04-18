#!/usr/bin/env python3
"""Launch Gazebo world + N namespaced MentorPi robots.

Usage:
  ros2 launch af_sim simulation.launch.py                      # 3 robots
  ros2 launch af_sim simulation.launch.py num_robots:=5
  ros2 launch af_sim simulation.launch.py world:=search_arena
"""
import os
import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def _setup_gazebo_env():
    """Set GAZEBO_MODEL_PATH so meshes resolve without network lookups."""
    desc_share = get_package_share_directory('af_description')
    model_parent = os.path.dirname(desc_share)
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    paths = [p for p in [model_parent, '/usr/share/gazebo-11/models', existing] if p]
    os.environ['GAZEBO_MODEL_PATH'] = ':'.join(paths)
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''


def _launch_setup(context):
    _setup_gazebo_env()
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    world_name = LaunchConfiguration('world').perform(context)

    sim_dir = get_package_share_directory('af_sim')
    world_file = os.path.join(sim_dir, 'worlds', f'{world_name}.world')

    actions = []

    headless = LaunchConfiguration('headless').perform(context)
    gazebo_cmd = 'gzserver' if headless.lower() == 'true' else 'gazebo'

    gazebo_server = ExecuteProcess(
        cmd=[
            gazebo_cmd, '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
    )
    actions.append(gazebo_server)

    spawn_launch = os.path.join(sim_dir, 'launch', 'spawn_robot.launch.py')

    home_x, home_y = 0.0, 0.0
    spawn_radius = 0.5

    for i in range(num_robots):
        angle = 2.0 * math.pi * i / max(num_robots, 1)
        x = home_x + spawn_radius * math.cos(angle)
        y = home_y + spawn_radius * math.sin(angle)
        yaw = angle + math.pi

        spawn = TimerAction(
            period=float(3.0 + i * 2.0),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(spawn_launch),
                    launch_arguments={
                        'namespace': f'robot_{i}',
                        'x': str(round(x, 3)),
                        'y': str(round(y, 3)),
                        'yaw': str(round(yaw, 3)),
                    }.items(),
                ),
            ],
        )
        actions.append(spawn)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='3'),
        DeclareLaunchArgument('world', default_value='search_arena'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
