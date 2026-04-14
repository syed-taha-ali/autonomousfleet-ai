from glob import glob

from setuptools import setup

package_name = 'af_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Syed Taha Ali',
    maintainer_email='nbstaha@gmail.com',
    description='SLAM and localisation configuration for AutonomousFleet AI.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
