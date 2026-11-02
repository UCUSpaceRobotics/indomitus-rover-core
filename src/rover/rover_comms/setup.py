import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_comms'

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
    # Not a console_script: it is a standalone diagnostic, wanted precisely
    # when the node will not start, on a rover where the installed workspace
    # may be all there is. Installed so `ros2 run rover_comms uart_loopback.py`
    # works without the source tree.
    scripts=['tools/uart_loopback.py'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuriifito',
    maintainer_email='fito.pn@ucu.edu.ua',
    description='Rover end of the LoRa fallback link',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lora_rover_node = rover_comms.lora_rover_node:main',
            'gs_link_lamp_node = rover_comms.gs_link_lamp_node:main',
        ],
    },
)
