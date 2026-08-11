alias cb="colcon build"
alias cbs="colcon build --symlink-install"
alias sws="if [ -f install/setup.zsh ]; then source install/setup.zsh && echo 'Workspace sourced!'; else echo 'No install/setup.zsh found in this directory.'; fi"

alias tl="ros2 topic list"
alias nl="ros2 node list"
alias te="ros2 topic echo"

alias launch_rover="ros2 launch rover_bringup rover.launch.py"
alias launch_joy="ros2 launch rover_teleop joy.launch.py"
alias launch_navigation="ros2 launch rover_teleop navigation.launch.py"
alias launch_nav="ros2 launch rover_teleop navigation.launch.py"


kill_node() {
    if [ -z "$1" ]; then
        echo "Usage: ros2kill <node_or_executable_name>"
        echo "Example: ros2kill minimal_publisher"
        return 1
    fi

    local target="$1"
    
    # 1. Try graceful termination (SIGINT - simulates Ctrl+C)
    echo "Attempting to gracefully stop '$target'..."
    pkill -SIGINT -f "$target"
    
    # Wait to allow node to clean up its DDS entities
    sleep 2
    
    # 2. Check if it's still running, and forcefully kill if necessary (SIGKILL)
    if pgrep -f "$target" > /dev/null; then
        echo "Node '$target' is still hanging. Forcing shutdown..."
        pkill -SIGKILL -f "$target"
        echo "Killed."
    else
        echo "Successfully shut down '$target'."
    fi
}