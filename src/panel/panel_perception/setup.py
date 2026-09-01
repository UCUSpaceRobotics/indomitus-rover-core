from setuptools import find_packages, setup

package_name = 'panel_perception'

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
    description="Fuses aruco_opencv per-marker detections of the switch panel's 3 ArUco tags into a single panel pose",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'panel_pose_fuser_node = panel_perception.panel_pose_fuser_node:main',
        ],
    },
)
