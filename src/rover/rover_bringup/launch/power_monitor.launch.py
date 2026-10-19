from launch import LaunchDescription
from rover_bringup.launch_utils import include_launch

def generate_launch_description():

    return LaunchDescription([
        include_launch('rover_peripherals', 'power_monitor_node.launch.py'),
        include_launch('rover_viz', 'power_monitor_viz.launch.py'),
    ])