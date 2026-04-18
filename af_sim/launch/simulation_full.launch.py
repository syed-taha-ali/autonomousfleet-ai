#!/usr/bin/env python3
"""Launch Gazebo world + N robots each with SLAM + Nav2.

This is the Tier 1 simulation launch: full Gazebo physics with LiDAR,
slam_toolbox mapping, and the complete Nav2 stack per robot.

Usage:
  ros2 launch af_sim simulation_full.launch.py
  ros2 launch af_sim simulation_full.launch.py num_robots:=5
"""
import os
import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def _setup_gazebo_env():
    """Set Gazebo environment so models and plugins resolve correctly."""
    desc_share = get_package_share_directory('af_description')
    model_parent = os.path.dirname(desc_share)
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    paths = [p for p in [model_parent, '/usr/share/gazebo-11/models', existing] if p]
    os.environ['GAZEBO_MODEL_PATH'] = ':'.join(paths)
    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''

    gz_plugin_dir = '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'
    for var in ('GAZEBO_PLUGIN_PATH', 'LD_LIBRARY_PATH'):
        cur = os.environ.get(var, '')
        if gz_plugin_dir not in cur:
            os.environ[var] = f'{gz_plugin_dir}:{cur}' if cur else gz_plugin_dir


_NAV2_LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]

# Remap global /tf → namespace-relative tf to avoid multi-robot TF conflicts.
# Only needed when num_robots > 1; populated in _launch_setup.
_TF_REMAPPINGS = []


def _spawn_nodes(ns, x, y, yaw, robot_desc):
    """Return RSP + spawn_entity nodes for one robot."""
    return (
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
            }],
            remappings=_TF_REMAPPINGS,
            output='log',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', ns,
                '-topic', f'/{ns}/robot_description',
                '-robot_namespace', ns,
                '-x', str(x), '-y', str(y), '-z', '0.05', '-Y', str(yaw),
                '-unpause',
            ],
            output='log',
        ),
    )


def _nav_nodes(ns, nav2_configured, slam_configured):
    """Return SLAM + Nav2 nodes for one robot."""
    sim_time = {'use_sim_time': True}
    nodes = []

    nodes.append(Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=ns,
        parameters=[slam_configured, sim_time],
        remappings=[('scan', 'scan')] + _TF_REMAPPINGS,
        output='screen',
    ))

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
            **sim_time,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': _NAV2_LIFECYCLE_NODES,
        }],
        output='log',
    ))

    return nodes


def _launch_setup(context):
    _setup_gazebo_env()
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    world_name = LaunchConfiguration('world').perform(context)

    sim_dir = get_package_share_directory('af_sim')
    desc_dir = get_package_share_directory('af_description')
    world_file = os.path.join(sim_dir, 'worlds', f'{world_name}.world')
    nav2_params = os.path.join(sim_dir, 'config', 'nav2_sim_params.yaml')
    slam_params = os.path.join(sim_dir, 'config', 'slam_sim_params.yaml')

    xacro_file = os.path.join(desc_dir, 'urdf', 'mentorpi_sim.xacro')
    import xacro

    actions = []

    headless = LaunchConfiguration('headless').perform(context)
    gazebo_cmd = 'gzserver' if headless.lower() == 'true' else 'gazebo'

    actions.append(ExecuteProcess(
        cmd=[
            gazebo_cmd, '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
    ))

    home_x, home_y = 0.0, 0.0
    spawn_radius = 0.6

    last_spawn = None

    for i in range(num_robots):
        angle = 2.0 * math.pi * i / max(num_robots, 1)
        x = round(home_x + spawn_radius * math.cos(angle), 3)
        y = round(home_y + spawn_radius * math.sin(angle), 3)
        yaw = round(angle + math.pi, 3)
        ns = f'robot_{i}'

        robot_desc = xacro.process_file(
            xacro_file, mappings={
                'robot_namespace': ns,
                'enable_camera': 'false',
            }).toxml()

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

        rsp_node, spawn_node = _spawn_nodes(ns, x, y, yaw, robot_desc)
        nav_actions = _nav_nodes(ns, nav2_configured, slam_configured)

        if last_spawn is None:
            actions.append(rsp_node)
            actions.append(spawn_node)
            # Nav2 starts after first spawn_entity exits
            actions.append(RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_node,
                    on_exit=nav_actions,
                )
            ))
        else:
            # Chain: wait for previous spawn to exit, then spawn this robot
            actions.append(RegisterEventHandler(
                OnProcessExit(
                    target_action=last_spawn,
                    on_exit=[rsp_node, spawn_node],
                )
            ))
            # Then wait for THIS spawn to exit before starting its Nav2
            actions.append(RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_node,
                    on_exit=nav_actions,
                )
            ))

        last_spawn = spawn_node

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='3'),
        DeclareLaunchArgument('world', default_value='search_arena'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=_launch_setup),
    ])
