#!/usr/bin/env python3
"""Exploration demo: find an object in an unknown room and return home.

Composes (staggered to avoid CPU-startup saturation on the Pi 5):
  t=0s   HAL + description + EKF + camera                  (robot.launch.py, perception off)
  t=0s   MS200 + scan sanitiser                            (lidar.launch.py)
  t=5s   slam_toolbox async mapping                        (af_slam/slam_mapping.launch.py)
  t=12s  Nav2 stack                                        (af_navigation/navigation.launch.py)
  t=15s  mission_manager + find_object_action + safety_validator
  t=22s  on-Pi YOLOv8n + depth_estimator                   (af_perception/perception_pi.launch.py)
  t=25s  simple_explore_node                               (frontier explorer)

The staggered bringup gives SLAM a CPU-quiet window to register the laser
sensor and publish an initial /map before Nav2, perception, and the explorer
all contend for the same cores. Phase 5a validation showed the prior
"launch-everything-at-t=0" approach saturated the Pi 5 and left slam_toolbox
starved of CPU (queue-full scan drops, no /map emission).

Usage (inside MentorPi Docker container):
  ros2 launch af_bringup explore.launch.py

  # Then from Dev PC, trigger the mission:
  ros2 action send_goal /find_object af_msgs/action/FindObject \\
      '{target_class: "suitcase", confidence_min: 0.5}'
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar').perform(context).lower() in ('true', '1', 'yes')
    start_robot = LaunchConfiguration('start_robot').perform(context).lower() in ('true', '1', 'yes')
    enable_perception = LaunchConfiguration('enable_perception').perform(context).lower() in ('true', '1', 'yes')
    enable_explore = LaunchConfiguration('enable_explore').perform(context).lower() in ('true', '1', 'yes')
    target_class = LaunchConfiguration('target_class')

    entities = []

    # t=0  HAL + camera + EKF + description (perception is explicitly OFF here;
    # it comes up later in the stagger so SLAM can boot without YOLO contention).
    if start_robot:
        entities.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('af_bringup'), 'launch', 'robot.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_perception': 'false',
                'enable_demo_stream': 'false',
            }.items(),
        ))

    # t=0  LiDAR
    if start_lidar:
        entities.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('af_bringup'), 'launch', 'lidar.launch.py'])),
        ))

    # t=5  SLAM in mapping mode
    entities.append(TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('af_slam'), 'launch', 'slam_mapping.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )],
    ))

    # t=12  Nav2 (consuming live /map from slam_toolbox, not map_server)
    entities.append(TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('af_navigation'), 'launch', 'navigation.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )],
    ))

    # t=15  Mission control nodes
    mission_manager = Node(
        package='af_mission_control',
        executable='mission_manager_node',
        name='mission_manager',
        output='screen',
        parameters=[{
            'target_class': target_class,
        }],
    )

    find_object_action = Node(
        package='af_mission_control',
        executable='find_object_action_node',
        name='find_object_action',
        output='screen',
    )

    safety_validator = Node(
        package='af_mission_control',
        executable='safety_validator_node',
        name='safety_validator',
        output='screen',
    )

    entities.append(TimerAction(
        period=15.0,
        actions=[mission_manager, find_object_action, safety_validator],
    ))

    # t=22  On-Pi perception (YOLOv8n + depth_estimator)
    if enable_perception:
        entities.append(TimerAction(
            period=22.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('af_perception'), 'launch', 'perception_pi.launch.py'])),
            )],
        ))

    # t=25  Frontier exploration (random-walk fallback)
    # Always launched; listens to /explore/enable for dynamic start/stop.
    # start_enabled=true means autonomous exploration from boot;
    # start_enabled=false means explorer waits for mission_manager to enable it.
    simple_explore = Node(
        package='af_mission_control',
        executable='simple_explore_node',
        name='simple_explore',
        output='screen',
        parameters=[{
            'start_enabled': enable_explore,
            'max_explore_time_s': LaunchConfiguration('explore_time_s'),
            'max_explore_distance_m': LaunchConfiguration('explore_distance_m'),
            'stop_on_detection': LaunchConfiguration('explore_stop_on_detection'),
            'target_class': target_class,
            'detection_confidence_min': 0.5,
        }],
    )
    entities.append(TimerAction(period=25.0, actions=[simple_explore]))

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_robot', default_value='true',
                              description='Start HAL + description + EKF (robot.launch.py).'),
        DeclareLaunchArgument('start_lidar', default_value='true',
                              description='Start MS200 LiDAR driver + scan sanitiser.'),
        DeclareLaunchArgument('enable_perception', default_value='true',
                              description='Start on-Pi YOLO perception pipeline (staggered to t=22 s).'),
        DeclareLaunchArgument('enable_explore', default_value='false',
                              description='Start the frontier explorer disabled; mission_manager enables it on demand.'),
        DeclareLaunchArgument('target_class', default_value='suitcase',
                              description='COCO class name to search for.'),
        DeclareLaunchArgument('explore_time_s', default_value='0.0',
                              description='Explorer time limit in seconds (0 = no limit).'),
        DeclareLaunchArgument('explore_distance_m', default_value='0.0',
                              description='Explorer distance limit in metres (0 = no limit).'),
        DeclareLaunchArgument('explore_stop_on_detection', default_value='false',
                              description='Stop explorer when target_class detected on /detections.'),
        OpaqueFunction(function=_launch_setup),
    ])
