#!/usr/bin/env python3
"""
Top-level bringup for the real MentorPi M1 robot.

Composes:
  - af_description/description.launch.py  (robot_state_publisher + URDF)
  - af_hal/af_hal.launch.py                (STM32 bridge, odom, imu_calib, watchdog)
  - imu_complementary_filter              (/imu_corrected -> /imu)
  - robot_localization ekf_filter_node    (fuses odom_raw + imu -> /odom)
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

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odom')],
    )

    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_frame', 'laser_frame'],
    )

    camera_info_url = PathJoinSubstitution([
        FindPackageShare('af_bringup'), 'config', 'camera_info.yaml'
    ])
    # image_transport auto-publishes /camera/image_raw/compressed and
    # /camera/image_raw/compressedDepth companions when the matching plugins
    # are installed (ros-humble-compressed-image-transport, already pulled in
    # by usb_cam). The Dev-PC perception stack subscribes to the compressed
    # topic to keep WiFi bandwidth at 2-4 Mbps.
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
            'pixel_format': 'yuyv',
            # `frame_id` is the correct param in usb_cam 0.6+; the URDF
            # already publishes a `depth_cam` frame rigidly attached to
            # base_link, so we reuse it for the USB camera too (the depth
            # and RGB sensors are colocated on the MentorPi front plate).
            'frame_id': 'depth_cam',
            'camera_name': 'mentorpi_cam',
            'camera_info_url': ['file://', camera_info_url],
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('image_raw/compressed', '/camera/image_raw/compressed'),
            ('image_raw/compressedDepth', '/camera/image_raw/compressedDepth'),
            ('image_raw/theora', '/camera/image_raw/theora'),
            ('camera_info', '/camera/camera_info'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        description_launch,
        hal_launch,
        imu_filter,
        ekf,
        lidar_frame_bridge,
        usb_cam,
    ])
