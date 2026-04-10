#!/usr/bin/env python3
"""
Top-level bringup for the real MentorPi M1 robot.

Composes:
  - af_description/description.launch.py  (robot_state_publisher + URDF)
  - af_hal/af_hal.launch.py                (STM32 bridge, odom, imu_calib, watchdog)
  - imu_complementary_filter              (/imu_corrected -> /imu)
  - rf2o_laser_odometry                   (/scan -> /odom_rf2o)
  - robot_localization ekf_filter_node    (fuses odom_raw + odom_rf2o + imu -> /odom)
  - usb_cam                               (compressed stream for Dev PC perception)

NOT included (run separately):
  * HiWonder factory MS200 LiDAR driver (runs in vendor Docker container)
  * Nav2, slam_toolbox, amcl (Phase 2/3)
  * af_perception pipeline (Dev PC, Phase 4)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ekf_config = PathJoinSubstitution([FindPackageShare('af_bringup'), 'config', 'ekf.yaml'])
    imu_filter_config = PathJoinSubstitution([FindPackageShare('af_bringup'), 'config', 'imu_filter.yaml'])

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('af_description'), 'launch', 'description.launch.py'
        ])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    hal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('af_hal'), 'launch', 'af_hal.launch.py'
        ])),
    )

    imu_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[imu_filter_config],
        remappings=[
            ('/imu/data_raw', '/imu_corrected'),
            ('/imu/data', '/imu'),
        ],
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0,
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odom')],
    )

    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'framerate': 15.0,
            'pixel_format': 'yuyv2rgb',
            'camera_frame_id': 'camera_link',
            'camera_name': 'mentorpi_cam',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        description_launch,
        hal_launch,
        imu_filter,
        rf2o,
        ekf,
        usb_cam,
    ])
