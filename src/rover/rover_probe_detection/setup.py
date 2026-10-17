import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_probe_detection'

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
    description='ERC probe detection in the rover cameras and multi-camera triangulation.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            f'probe_detector_node = {package_name}.probe_detector_node:main',
            f'probe_triangulator_node = {package_name}.probe_triangulator_node:main',
            f'fake_detections_node = {package_name}.fake_detections_node:main',
        ],
    },
)
