from setuptools import find_packages, setup

package_name = 'arm_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'poses.json']),
        ('share/' + package_name + '/launch', [
            'launch/gamepad.launch.py',
            'launch/gamepad_joy.launch.py',
            'launch/gamepad_servo.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UCUSpaceRobotics',
    maintainer_email='indomitus@ucu.edu.ua',
    description='Direct operator control (keyboard + gamepad teleop) for the Indomitus arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop_node = arm_teleop.keyboard_teleop_node:main',
            'gamepad_teleop_node = arm_teleop.gamepad_teleop_node:main',
            'arm_motion_lock_server = arm_teleop.arm_motion_lock_server:main',
        ],
    },
)
