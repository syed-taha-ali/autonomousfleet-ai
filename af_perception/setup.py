from glob import glob

from setuptools import find_packages, setup

package_name = 'af_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob('models/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Syed Taha Ali',
    maintainer_email='nbstaha@gmail.com',
    description='Distributed YOLO + depth-backed perception pipeline that feeds Nav2 via PointCloud2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_decompressor_node = af_perception.image_decompressor_node:main',
            'yolo_detector_node = af_perception.yolo_detector_node:main',
            'depth_estimator_node = af_perception.depth_estimator_node:main',
        ],
    },
)
