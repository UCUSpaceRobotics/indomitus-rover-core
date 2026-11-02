import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_cameras'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'calibration'),
            glob(os.path.join('config', 'calibration', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yurifi',
    maintainer_email='fito.pn@ucu.edu.ua',
    description='Rover cameras: GStreamer capture, MJPEG over HTTP to the GS, Image topics onboard.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            f'camera_node = {package_name}.camera_node:main',
            f'dataset_node = {package_name}.dataset_node:main',
        ],
    },
)
