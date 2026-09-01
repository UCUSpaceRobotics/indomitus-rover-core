import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_peripherals'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UCUSpaceRobotics',
    maintainer_email='indomitus@ucu.edu.ua',
    description='ROS2 nodes for the arm end-effector tool via CAN bus',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'end_effector_can_node = arm_peripherals.end_effector_can_node:main',
        ],
    },
)
