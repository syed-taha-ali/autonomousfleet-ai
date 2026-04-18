#!/usr/bin/env python3
"""Launch MVSim world + N robots each with SLAM + Nav2.

MVSim runs wall-clock time (no /clock, no use_sim_time).
Vehicles are injected into the world XML at launch time.
TF is published per-vehicle on /<vehicle>/tf, so all nodes
remap /tf → tf to read from their namespaced TF topic.

Usage:
  ros2 launch af_sim simulation_mvsim.launch.py
  ros2 launch af_sim simulation_mvsim.launch.py num_robots:=5
"""
import math
import os
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


_NAV2_LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]

_TF_REMAPPINGS = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

_LIDAR_DEFS = '/opt/ros/humble/share/mvsim/definitions/lidar2d.sensor.xml'

_VEHICLE_TEMPLATE = """\
    <vehicle name="{name}" class="small_robot">
        <init_pose>{x:.3f} {y:.3f} {yaw_deg:.1f}</init_pose>
        <init_vel>0 0 0</init_vel>
        <include file="{lidar_defs}"
            sensor_x="0.25" sensor_y="0" sensor_z="0.30" sensor_yaw="0"
            sensor_name="scan"
            fov_degrees="360"
            sensor_nrays="450"
            max_range="12.0"
            sensor_period_sec="0.1"
            sensor_std_noise="0.01"
            raytrace_3d="false"
            sensor_publish="true"
            viz_visibleLines="false"
        />
    </vehicle>
"""


def _generate_world_xml(template_path, num_robots, spawn_radius=0.6):
    """Read the arena template and inject vehicle definitions."""
    with open(template_path, 'r') as f:
        template = f.read()

    vehicles = []
    for i in range(num_robots):
        angle = 2.0 * math.pi * i / max(num_robots, 1)
        x = spawn_radius * math.cos(angle)
        y = spawn_radius * math.sin(angle)
        yaw_deg = math.degrees(angle + math.pi)
        vehicles.append(_VEHICLE_TEMPLATE.format(
            name=f'robot_{i}', x=x, y=y, yaw_deg=yaw_deg,
            lidar_defs=_LIDAR_DEFS,
        ))

    world_xml = template.replace(
        '    <!-- VEHICLES_PLACEHOLDER: replaced by launch file -->',
        '\n'.join(vehicles),
    )

    fd, path = tempfile.mkstemp(suffix='.world.xml', prefix='mvsim_arena_')
    with os.fdopen(fd, 'w') as f:
        f.write(world_xml)
    return path


def _slam_node(ns, slam_configured):
    """Return the SLAM node for one robot."""
    return Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=ns,
        parameters=[slam_configured, {'use_sim_time': False}],
        remappings=[('/map', 'map'), ('/map_metadata', 'map_metadata')] + _TF_REMAPPINGS,
        output='screen',
    )


def _nav2_nodes(ns, nav2_configured):
    """Return Nav2 nodes for one robot."""
    nodes = []

    for node_name, pkg, exe in [
        ('controller_server', 'nav2_controller', 'controller_server'),
        ('smoother_server', 'nav2_smoother', 'smoother_server'),
        ('planner_server', 'nav2_planner', 'planner_server'),
        ('behavior_server', 'nav2_behaviors', 'behavior_server'),
        ('bt_navigator', 'nav2_bt_navigator', 'bt_navigator'),
        ('waypoint_follower', 'nav2_waypoint_follower', 'waypoint_follower'),
        ('velocity_smoother', 'nav2_velocity_smoother', 'velocity_smoother'),
    ]:
        remappings = list(_TF_REMAPPINGS)
        if node_name in ('controller_server', 'behavior_server'):
            remappings.append(('cmd_vel', 'cmd_vel_nav'))
        if node_name == 'velocity_smoother':
            remappings = list(_TF_REMAPPINGS) + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ]

        nodes.append(Node(
            package=pkg,
            executable=exe,
            name=node_name,
            namespace=ns,
            parameters=[nav2_configured],
            remappings=remappings,
            output='log',
        ))

    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=ns,
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': _NAV2_LIFECYCLE_NODES,
        }],
        remappings=_TF_REMAPPINGS,
        output='log',
    ))

    return nodes


def _launch_setup(context):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'

    sim_dir = get_package_share_directory('af_sim')
    template_path = os.path.join(sim_dir, 'worlds', 'search_arena_mvsim.world.xml')
    nav2_params = os.path.join(sim_dir, 'config', 'nav2_mvsim_params.yaml')
    slam_params = os.path.join(sim_dir, 'config', 'slam_mvsim_params.yaml')

    world_file = _generate_world_xml(template_path, num_robots)

    actions = []

    actions.append(Node(
        package='mvsim',
        executable='mvsim_node',
        name='mvsim',
        output='screen',
        parameters=[{
            'world_file': world_file,
            'headless': headless,
            'do_fake_localization': False,
            'publish_tf_odom2baselink': True,
            'force_publish_vehicle_namespace': True,
        }],
    ))

    for i in range(num_robots):
        ns = f'robot_{i}'

        nav2_configured = RewrittenYaml(
            source_file=nav2_params,
            root_key=ns,
            param_rewrites={},
            convert_types=True,
        )
        slam_configured = RewrittenYaml(
            source_file=slam_params,
            root_key=ns,
            param_rewrites={},
            convert_types=True,
        )

        slam_delay = 5.0 + i * 2.0
        nav2_delay = slam_delay + 8.0

        actions.append(TimerAction(
            period=slam_delay,
            actions=[_slam_node(ns, slam_configured)],
        ))
        actions.append(TimerAction(
            period=nav2_delay,
            actions=_nav2_nodes(ns, nav2_configured),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='3'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
