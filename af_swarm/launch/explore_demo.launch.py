#!/usr/bin/env python3
"""Tier 1 demo: distributed exploration on MVSim.

Launches the swarm coordinator, object registry, and per-robot
ground-truth detector + distributed explorer on top of a running
simulation_mvsim.launch.py.

Usage:
  # Terminal 1: start MVSim + robots + SLAM + Nav2
  ros2 launch af_sim simulation_mvsim.launch.py num_robots:=3

  # Terminal 2: start swarm layer
  ros2 launch af_swarm explore_demo.launch.py num_robots:=3
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_OBJECT_CLASSES = ['obj_0', 'obj_1', 'obj_2', 'obj_3', 'obj_4']
_OBJECT_X = [4.0, -4.0, 0.0, 0.0, -1.5]
_OBJECT_Y = [0.0, 0.0, 3.0, -3.0, -1.0]


def _launch_setup(context):
    num = int(LaunchConfiguration('num_robots').perform(context))
    prefix = 'robot_'
    actions = []

    fleet_nodes = [
        Node(
            package='af_swarm',
            executable='swarm_coordinator_node',
            name='swarm_coordinator',
            parameters=[{
                'use_sim_time': False,
                'num_robots': num,
                'robot_prefix': prefix,
                'objects_target': len(_OBJECT_CLASSES),
                'home_x': 0.0,
                'home_y': 0.0,
            }],
            output='screen',
        ),
        Node(
            package='af_swarm',
            executable='object_registry_node',
            name='object_registry',
            parameters=[{
                'use_sim_time': False,
                'num_robots': num,
                'robot_prefix': prefix,
                'dedup_radius_m': 0.5,
            }],
            output='screen',
        ),
    ]
    actions.append(TimerAction(period=5.0, actions=fleet_nodes))

    spawn_radius = 1.0
    for i in range(num):
        ns = f'{prefix}{i}'
        angle = 2.0 * math.pi * i / max(num, 1)
        hx = round(spawn_radius * math.cos(angle), 3)
        hy = round(spawn_radius * math.sin(angle), 3)

        robot_nodes = [
            Node(
                package='af_swarm',
                executable='ground_truth_detector_node',
                name='ground_truth_detector',
                namespace=ns,
                parameters=[{
                    'use_sim_time': False,
                    'robot_namespace': ns,
                    'object_classes': _OBJECT_CLASSES,
                    'object_x': _OBJECT_X,
                    'object_y': _OBJECT_Y,
                    'detection_range': 2.0,
                    'fov_deg': 60.0,
                }],
                output='screen',
            ),
            Node(
                package='af_swarm',
                executable='distributed_explore_node',
                name='distributed_explore',
                namespace=ns,
                parameters=[{
                    'use_sim_time': False,
                    'robot_namespace': ns,
                    'home_x': hx,
                    'home_y': hy,
                    'min_frontier_size': 5,
                    'claim_penalty': 0.1,
                    'explore_rate_hz': 0.5,
                    'num_robots': num,
                    'robot_prefix': prefix,
                    'robot_repulsion_radius': 3.0,
                }],
                output='screen',
            ),
        ]
        actions.append(TimerAction(
            period=10.0 + i * 2.0,
            actions=robot_nodes,
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='3'),
        OpaqueFunction(function=_launch_setup),
    ])
