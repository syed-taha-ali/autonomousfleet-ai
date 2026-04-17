from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'af_nlp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Syed Taha Ali',
    maintainer_email='nbstaha@gmail.com',
    description='Natural-language mission interface for AutonomousFleet AI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nlp_command_node = af_nlp.nlp_command_node:main',
            'cli = af_nlp.cli:main',
        ],
    },
)
