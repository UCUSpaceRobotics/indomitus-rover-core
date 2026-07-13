import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_peripherals'

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

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yurifi',
    maintainer_email='fito.pn@ucu.edu.ua',
    description='ROS2 nodes for rover peripheral hardware via CAN bus',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rover_container_node = rover_peripherals.rover_container_node:main',
            'rover_lighting_node = rover_peripherals.rover_lighting_node:main',
            'rover_power_node = rover_peripherals.rover_power_node:main',
        ],
    },
)
