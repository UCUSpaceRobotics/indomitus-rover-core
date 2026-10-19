from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from rover_bringup.launch_utils import include_launch

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'rover_namespace',
            default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value='rover'),
            description='ROS namespace all rover nodes/topics are pushed under (arm excluded). '
                        'Must match the namespace of the rover graph being monitored - '
                        'plotjuggler/power_monitor.xml hardcodes the /rover/ prefix.',
        ),
        GroupAction([
            PushRosNamespace(LaunchConfiguration('rover_namespace')),
            include_launch('rover_peripherals', 'power_monitor_node.launch.py'),
        ]),
        # plotjuggler just subscribes using the exact topic strings baked
        # into power_monitor.xml - it isn't itself part of the rover graph.
        include_launch('rover_viz', 'power_monitor_gui.launch.py'),
    ])