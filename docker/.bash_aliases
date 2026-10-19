# After making changes in this file update docs/software/aliases.md

alias cb="colcon build --symlink-install"
alias cbr="colcon build --symlink-install --packages-select-regex "^rover_""
alias cba="colcon build --symlink-install --packages-select-regex "^arm_""
alias cbp="colcon build --symlink-install --packages-select-regex "^panel_""
alias sws="if [ -f install/setup.bash ]; then source install/setup.bash && echo 'Workspace sourced!'; else echo 'No install/setup.bash found in this directory.'; fi"
alias sb="if [ -f install/setup.bash ]; then source install/setup.bash && echo 'Workspace sourced!'; else echo 'No install/setup.bash found in this directory.'; fi"

alias tl="ros2 topic list"
alias nl="ros2 node list"
alias te="ros2 topic echo"

alias launch_rover="ros2 launch rover_bringup rover.launch.py"
alias launch_arm="ros2 launch arm_bringup arm.launch.py"
alias launch_gamepad="ros2 launch rover_teleop gamepad.launch.py"
alias launch_navigation="ros2 launch rover_bringup navigation.launch.py"
alias launch_nav="ros2 launch rover_bringup navigation.launch.py"


# Prints PIDs of live (non-zombie) processes whose command line matches the ERE in $1
_kill_node_find() {
    local pid stat ancestors=" " p="$BASHPID"
    # Never match this shell or its ancestors (e.g. `bash -c "kill_node foo"`)
    while [ -n "$p" ] && [ "$p" -gt 1 ]; do
        ancestors+="$p "
        p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
    done
    for pid in $(pgrep -f -- "$1"); do
        [[ "$ancestors" == *" $pid "* ]] && continue
        stat=$(ps -o stat= -p "$pid" 2>/dev/null)
        case "$stat" in
            ""|Z*) ;;  # already exited or zombie (dead, just not reaped yet)
            *) echo "$pid" ;;
        esac
    done
}

kill_node() {
    if [ -z "$1" ]; then
        echo "Usage: kill_node <node_or_executable_name>"
        echo "Example: kill_node minimal_publisher"
        return 1
    fi

    local target="$1"

    # Match the name literally, as the executable (argv[0] or the script path of a
    # Python node) or as a remapped node name (__node:=<name>), not as any substring
    local escaped pattern
    escaped=$(printf '%s' "$target" | sed 's/[][\.*^$+?(){}|/]/\\&/g')
    pattern="(^|/)${escaped}( |$)|__node:=${escaped}( |$)"

    local pids
    pids=$(_kill_node_find "$pattern")
    if [ -z "$pids" ]; then
        echo "No running process matches '$target'."
        return 1
    fi

    echo "Found processes matching '$target':"
    ps -o pid=,args= -p $(echo $pids | tr ' ' ',')

    # Escalate: SIGINT (Ctrl+C, lets the node clean up its DDS entities),
    # then SIGTERM, then SIGKILL. Each stage waits up to the given number of seconds.
    local stage sig timeout i
    for stage in INT:5 TERM:3 KILL:2; do
        sig="${stage%%:*}"
        timeout="${stage##*:}"

        echo "Sending SIG$sig..."
        kill -s "$sig" $pids 2>/dev/null

        # Poll every 0.1 s instead of sleeping a fixed time
        for ((i = 0; i < timeout * 10; i++)); do
            pids=$(_kill_node_find "$pattern")
            [ -z "$pids" ] && break
            sleep 0.1
        done
        [ -z "$pids" ] && break
        echo "Still running after SIG$sig: $(echo $pids)"
    done

    # Final check: rescan from scratch so respawned instances are caught too
    pids=$(_kill_node_find "$pattern")
    if [ -n "$pids" ]; then
        echo "ERROR: '$target' is still running:"
        ps -o pid,stat,user,args -p $(echo $pids | tr ' ' ',')
        echo "Hint: state 'D' means stuck in uninterruptible I/O; a new PID means it was respawned (e.g. by ros2 launch respawn=True)."
        return 1
    fi

    echo "Verified: no process matching '$target' is running."
}
