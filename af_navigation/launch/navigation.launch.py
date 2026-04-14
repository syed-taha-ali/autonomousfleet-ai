#!/usr/bin/env python3
"""Bring up the Nav2 navigation stack for AutonomousFleet AI.

Launches controller_server, smoother_server, planner_server, behavior_server,
bt_navigator, waypoint_follower, velocity_smoother, and the navigation
lifecycle manager (which auto-configures and auto-activates all of them).

Expects map_server + AMCL to already be running (see
`af_slam/launch/slam_localization.launch.py`) or slam_toolbox in mapping mode.
The navigation lifecycle manager deliberately does not own those nodes so the
localisation stack can be brought up and taken down independently.

Usage:
  ros2 launch af_navigation navigation.launch.py
  ros2 launch af_navigation navigation.launch.py \\
      params_file:=/custom/nav2_params.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    bt_through_xml = LaunchConfiguration('default_nav_through_poses_bt_xml')
    log_level = LaunchConfiguration('log_level')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true.')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack.')

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('af_navigation'), 'config', 'nav2_params.yaml']),
        description='Full path to the Nav2 parameter file.')

    declare_bt_xml = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=PathJoinSubstitution([
            FindPackageShare('af_navigation'), 'behavior_trees',
            'af_navigate_to_pose.xml']),
        description='Full path to the NavigateToPose behaviour tree XML.')

    declare_bt_through_xml = DeclareLaunchArgument(
        'default_nav_through_poses_bt_xml',
        default_value=PathJoinSubstitution([
            FindPackageShare('af_navigation'), 'behavior_trees',
            'af_navigate_through_poses.xml']),
        description='Full path to the NavigateThroughPoses behaviour tree XML.')

    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Log level for all nav2 nodes.')

    # Common parameter overlays. The YAML holds tuned values; these additions
    # carry runtime-only paths (package-share expansions) and the sim-time
    # toggle without duplicating the whole file.
    common_overlay = {'use_sim_time': use_sim_time}
    bt_overlay = {
        'use_sim_time': use_sim_time,
        'default_nav_to_pose_bt_xml': bt_xml,
        'default_nav_through_poses_bt_xml': bt_through_xml,
    }

    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file, common_overlay],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server',
        name='smoother_server', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file, common_overlay],
    )

    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file, common_overlay],
    )

    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file, common_overlay],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file, bt_overlay],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file, common_overlay],
    )

    # velocity_smoother sits between Nav2 (publishing /cmd_vel_nav) and the
    # robot (subscribing /cmd_vel). Sinks the controller output into a rate-
    # limited stream so the motor driver never sees step changes.
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[params_file, common_overlay],
        remappings=[('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'bond_timeout': 4.0,
            'node_names': _LIFECYCLE_NODES,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params,
        declare_bt_xml,
        declare_bt_through_xml,
        declare_log_level,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
    ])
