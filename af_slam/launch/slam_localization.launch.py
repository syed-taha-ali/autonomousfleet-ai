#!/usr/bin/env python3
"""Launch map_server + AMCL for localisation within a saved map.

Requires a map file — pass via the 'map' argument:
  ros2 launch af_slam slam_localization.launch.py map:=/path/to/map.yaml

Both map_server and amcl are lifecycle-managed; nav2_lifecycle_manager
auto-configures and activates them in order.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    amcl_params_file = LaunchConfiguration('amcl_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    declare_map = DeclareLaunchArgument(
        'map',
        description='Full path to the map YAML file (e.g. /path/to/map.yaml)')

    declare_amcl_params = DeclareLaunchArgument(
        'amcl_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('af_slam'), 'config', 'amcl_params.yaml']),
        description='Full path to the AMCL / map_server parameter file')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            amcl_params_file,
            {'use_sim_time': use_sim_time,
             'yaml_filename': map_file},
        ],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 4.0,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_amcl_params,
        map_server_node,
        amcl_node,
        lifecycle_manager,
    ])
