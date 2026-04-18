#!/usr/bin/env python3
"""Tier 3 demo: Reynolds flocking with N lightweight agents.

Uses a single batch_simulator_node that manages all N agents in one
process with numpy-vectorised kinematics and integrated flocking.
Memory: ~100 MB + ~0.5 MB/agent (not ~80 MB/agent like separate nodes).

Usage:
  ros2 launch af_swarm flocking_demo.launch.py                    # 100 agents
  ros2 launch af_swarm flocking_demo.launch.py num_agents:=200
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_agents', default_value='100'),
        DeclareLaunchArgument('spawn_radius', default_value='5.0'),
        DeclareLaunchArgument('sim_rate', default_value='10.0'),

        Node(
            package='af_swarm',
            executable='batch_simulator_node',
            name='batch_simulator',
            parameters=[{
                'num_agents': LaunchConfiguration('num_agents'),
                'robot_prefix': 'robot_',
                'spawn_radius': LaunchConfiguration('spawn_radius'),
                'sim_rate': LaunchConfiguration('sim_rate'),
                'enable_flocking': True,
                'separation_radius': 0.3,
                'alignment_radius': 3.0,
                'cohesion_radius': 4.0,
                'separation_weight': 1.5,
                'alignment_weight': 2.0,
                'cohesion_weight': 0.5,
                'max_speed': 0.3,
                'migration_weight': 0.8,
                'migration_angle_deg': 0.0,
                'boundary_radius': 50.0,
                'boundary_weight': 2.0,
            }],
            output='screen',
        ),
    ])
