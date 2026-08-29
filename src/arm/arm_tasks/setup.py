from setuptools import find_packages, setup

package_name = 'arm_tasks'

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
    description='Core control tasks and MoveIt scripts for the Indomitus arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teach_poses = arm_tasks.teach_poses:main',
            'keyboard_servo_node = arm_tasks.keyboard_servo_node:main',
            'gamepad_servo_node = arm_tasks.keyboard_servo_node:main_gamepad',
            'collision_link_reporter = arm_tasks.collision_link_reporter:main',
            'panel_align_node = arm_tasks.panel_align_node:main',
        ],
    },
)