from glob import glob

from setuptools import find_packages, setup

package_name = 'af_swarm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Syed Taha Ali',
    maintainer_email='nbstaha@gmail.com',
    description='Multi-robot swarm algorithms for AutonomousFleet AI.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_coordinator_node = af_swarm.swarm_coordinator_node:main',
            'object_registry_node = af_swarm.object_registry_node:main',
            'distributed_explore_node = af_swarm.distributed_explore_node:main',
            'flocking_node = af_swarm.flocking_node:main',
            'lightweight_agent_node = af_swarm.lightweight_agent_node:main',
            'ground_truth_detector_node = af_swarm.ground_truth_detector_node:main',
            'batch_simulator_node = af_swarm.batch_simulator_node:main',
        ],
    },
)
