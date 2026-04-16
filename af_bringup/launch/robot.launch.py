#!/usr/bin/env python3
"""
Top-level bringup for the real MentorPi M1 robot.

Composes:
  - af_description/description.launch.py  (robot_state_publisher + URDF)
  - af_hal/af_hal.launch.py                (STM32 bridge, odom, imu_calib, watchdog)
  - imu_complementary_filter              (/imu_corrected -> /imu)
  - robot_localization ekf_filter_node    (fuses odom_raw + imu -> /odom)
  - usb_cam                               (raw + compressed stream)
  - camera demo throttle                  (/camera/demo/compressed @ N Hz, RViz)
  - af_perception/perception_pi           (on-robot YOLO, optional)

NOT included (run separately):
  * HiWonder factory MS200 LiDAR driver (runs via lidar.launch.py)
  * Nav2, slam_toolbox, amcl (Phase 2/3)

Launch arguments
----------------
  use_sim_time:=false
  enable_perception:=false
      When true, launches the on-Pi YOLOv8n + depth_estimator pipeline
      (Phase 4.1). Defaults to off so the Phase 1-3 bringup stays lean.
  enable_demo_stream:=true
      When true, re-publishes /camera/image_raw/compressed at
      `demo_stream_hz` (default 4 Hz) on /camera/demo/compressed so RViz
      on the Dev PC can render a visual without saturating WiFi.
  demo_stream_hz:=4.0
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_perception = LaunchConfiguration('enable_perception')
    enable_demo_stream = LaunchConfiguration('enable_demo_stream').perform(context)
    demo_stream_hz = LaunchConfiguration('demo_stream_hz').perform(context)

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
    # by usb_cam). Phase 4.1 keeps usb_cam local on the Pi — raw frames are
    # consumed in-process by yolo_onnx_node and the compressed topic only
    # flows over WiFi via the throttled demo stream below.
    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'framerate': 10.0,
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

    entities = [
        description_launch,
        hal_launch,
        imu_filter,
        ekf,
        lidar_frame_bridge,
        usb_cam,
    ]

    # Throttled demo stream. topic_tools `throttle messages` will republish
    # at most `demo_stream_hz` per second — enough for a live RViz preview
    # on the Dev PC without burning the WiFi link. Skipped entirely when
    # `enable_demo_stream:=false`.
    if enable_demo_stream.lower() in ('true', '1', 'yes'):
        demo_throttle = Node(
            package='topic_tools',
            executable='throttle',
            name='camera_demo_throttle',
            arguments=[
                'messages',
                '/camera/image_raw/compressed',
                demo_stream_hz,
                '/camera/demo/compressed',
            ],
            output='screen',
        )
        entities.append(demo_throttle)

    perception_pi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('af_perception'), 'launch', 'perception_pi.launch.py'
        ])),
        condition=IfCondition(enable_perception),
    )
    entities.append(perception_pi_launch)

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'enable_perception', default_value='false',
            description='Launch the on-Pi YOLOv8n + depth_estimator pipeline (Phase 4.1).'),
        DeclareLaunchArgument(
            'enable_demo_stream', default_value='true',
            description='Re-publish compressed camera at demo_stream_hz on /camera/demo/compressed.'),
        DeclareLaunchArgument(
            'demo_stream_hz', default_value='4.0',
            description='Throttle rate for the demo camera stream (Hz).'),
        OpaqueFunction(function=_launch_setup),
    ])
