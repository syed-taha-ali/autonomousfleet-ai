#!/usr/bin/env python3
"""Launch slam_toolbox in async online mapping mode.

Starts a LifecycleNode, auto-configures, and auto-activates it so that
mapping begins as soon as /scan and TF (odom -> base_footprint) are available.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('af_slam'), 'config',
            'mapper_params_online_async.yaml']),
        description='Full path to the slam_toolbox parameter file')

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='slam_toolbox configured — activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params,
        slam_toolbox_node,
        configure_event,
        activate_event,
    ])
