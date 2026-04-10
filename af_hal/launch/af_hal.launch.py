#!/usr/bin/env python3
"""
Master launch for the AutonomousFleet AI hardware abstraction layer.

Starts:
  - ros_robot_controller_node (STM32 bridge)
  - odom_publisher_node (mecanum IK + dead-reckoning)
  - imu_calib_node (static bias removal)
  - hardware_watchdog_node (battery + heartbeat diagnostics)

This launch does NOT start:
  * MS200 LiDAR driver (HiWonder factory driver runs outside our workspace)
  * imu_complementary_filter (wired in af_bringup's robot.launch.py)
  * robot_localization EKF (wired in af_bringup)
  * robot_state_publisher (wired in af_bringup with the URDF)
  * usb_cam (wired in af_bringup)
Those are composed by af_bringup/launch/robot.launch.py on top of this one.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rrc = Node(
        package='af_hal',
        executable='ros_robot_controller_node',
        name='ros_robot_controller',
        output='screen',
        parameters=[{'imu_frame': 'imu_link'}],
    )

    odom = Node(
        package='af_hal',
        executable='odom_publisher_node',
        name='odom_publisher',
        output='screen',
        remappings=[
            ('ros_robot_controller/set_motor', '/ros_robot_controller/set_motor'),
            ('odom_raw', '/odom_raw'),
            ('cmd_vel', '/cmd_vel'),
            ('set_pose', '/set_pose'),
        ],
    )

    imu_calib = Node(
        package='af_hal',
        executable='imu_calib_node',
        name='imu_calib',
        output='screen',
        parameters=[{
            'input_topic': '/ros_robot_controller/imu_raw',
            'output_topic': '/imu_corrected',
            'calibration_samples': 200,
        }],
    )

    watchdog = Node(
        package='af_hal',
        executable='hardware_watchdog_node',
        name='hardware_watchdog',
        output='screen',
    )

    return LaunchDescription([rrc, odom, imu_calib, watchdog])
