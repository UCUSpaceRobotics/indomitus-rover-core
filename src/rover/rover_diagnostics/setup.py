import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_diagnostics'

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
    maintainer='yurifi',
    maintainer_email='fito.pn@ucu.edu.ua',
    description='Fault event logging for the rover',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fault_logger_node = rover_diagnostics.fault_logger_node:main',
        ],
    },
)
