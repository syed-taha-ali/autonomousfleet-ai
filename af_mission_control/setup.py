from glob import glob

from setuptools import setup

package_name = 'af_mission_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
    description='Mission state machine, find-object action server, and safety validator.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager_node = af_mission_control.mission_manager_node:main',
            'find_object_action_node = af_mission_control.find_object_action_node:main',
            'safety_validator_node = af_mission_control.safety_validator_node:main',
            'simple_explore_node = af_mission_control.simple_explore_node:main',
        ],
    },
)
