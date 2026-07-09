from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # 1. Create a variable that will read the value from the terminal
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # 2. Declare the argument for ROS 2 Launch (so it knows about its existence)
    declare_use_fake_hardware_cmd = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Whether to use fake hardware or real CAN bus"
    )

    # 3. Pass the variable into the builder via the mappings parameter
    moveit_config = (
        MoveItConfigsBuilder("indomitus_arm", package_name="arm_moveit_config")
        .robot_description(mappings={"use_fake_hardware": use_fake_hardware})
        .to_moveit_configs()
    )

    # Generate the base demo launch description
    demo_launch = generate_demo_launch(moveit_config)

    # 4. Create a new LaunchDescription and add the argument declaration
    ld = LaunchDescription()
    ld.add_action(declare_use_fake_hardware_cmd)
    
    # Add all entities from the generated demo launch
    for action in demo_launch.entities:
        ld.add_action(action)

    return ld
