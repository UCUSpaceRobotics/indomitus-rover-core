from setuptools import find_packages, setup

package_name = 'rover_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yurifi',
    maintainer_email='fito.pn@ucu.edu.ua',
    description='Teleometry package.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joystick_interpreter_node = rover_control.joystick_interpreter_node:main',
        ],
    },
)
