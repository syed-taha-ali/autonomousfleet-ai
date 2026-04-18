#!/usr/bin/env python3
"""Launch RViz2 with a mode-specific configuration.

Usage:
  ros2 launch af_monitoring rviz.launch.py mode:=navigation
  ros2 launch af_monitoring rviz.launch.py mode:=slam
  ros2 launch af_monitoring rviz.launch.py mode:=perception
  ros2 launch af_monitoring rviz.launch.py mode:=full
  ros2 launch af_monitoring rviz.launch.py mode:=swarm
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_VALID_MODES = ('navigation', 'slam', 'perception', 'full', 'swarm')


def _launch_setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in _VALID_MODES:
        raise ValueError(
            f'Invalid mode "{mode}". Choose from: {", ".join(_VALID_MODES)}')

    rviz_config = PathJoinSubstitution([
        FindPackageShare('af_monitoring'), 'config', f'{mode}.rviz'])

    return [Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='navigation',
            description=f'RViz config to load: {", ".join(_VALID_MODES)}'),
        OpaqueFunction(function=_launch_setup),
    ])
