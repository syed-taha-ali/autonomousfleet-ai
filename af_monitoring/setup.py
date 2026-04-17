from glob import glob

from setuptools import find_packages, setup

package_name = 'af_monitoring'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Syed Taha Ali',
    maintainer_email='nbstaha@gmail.com',
    description='Monitoring, logging and visualisation for AutonomousFleet AI.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perf_monitor_node = af_monitoring.perf_monitor_node:main',
            'detection_overlay_node = af_monitoring.detection_overlay_node:main',
        ],
    },
)
