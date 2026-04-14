from glob import glob

from setuptools import find_packages, setup

package_name = 'af_hal'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Syed Taha Ali',
    maintainer_email='nbstaha@gmail.com',
    description='Hardware abstraction layer for MentorPi M1 (STM32 bridge, mecanum odom, IMU calib, watchdog).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_robot_controller_node = af_hal.ros_robot_controller_node:main',
            'odom_publisher_node = af_hal.odom_publisher_node:main',
            'imu_calib_node = af_hal.imu_calib_node:main',
            'hardware_watchdog_node = af_hal.hardware_watchdog_node:main',
            'scan_sanitizer_node = af_hal.scan_sanitizer_node:main',
        ],
    },
)
