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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UCUSpaceRobotics',
    maintainer_email='indomitus@ucu.edu.ua',
    description='Runtime diagnostics and safety watchdogs for the rover (steering/velocity jump detection, current monitoring, PlotJuggler visualization)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'swerve_jump_watchdog = rover_diagnostics.swerve_jump_watchdog:main'
        ],
    },
)
