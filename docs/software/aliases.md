# Environment Aliases

To speed up development and standardize common workflows, our development container comes pre-configured with several custom shortcuts.

## Available Aliases and Commands

| Alias | Command / Action | Description |
| --- | --- | --- |
| **`cb`** | `colcon build` | Builds the colcon workspace. |
| **`cbs`** | `colcon build --symlink-install` | Builds the workspace using symlinks for faster iterative development. |
| **`sws`** | `source install/setup.zsh` | Sources the local workspace. Prints a confirmation or a warning if the file is missing. |
| **`tl`** | `ros2 topic list` | Lists all active ROS 2 topics. |
| **`nl`** | `ros2 node list` | Lists all active ROS 2 nodes. |
| **`te`** | `ros2 topic echo` | Echoes data published to a specific ROS 2 topic. |
| **`launch_rover`** | `ros2 launch rover_bringup rover.launch.py` | Launches the main rover bringup file. |
| **`launch_joy`** | `ros2 launch rover_teleop joy.launch.py` | Launches the joystick teleop nodes. |
| **`launch_navigation`**, **`launch_nav`** | `ros2 launch rover_teleop navigation.launch.py` | Launches the navigation stack. Both aliases perform the exact same action. |
| **`kill_node`** | *(Custom Bash Function)* | Safely shuts down a target node by attempting a graceful `SIGINT`, then falling back to a forced `SIGKILL` after 2 seconds if it hangs. Usage: `kill_node <node_name>`. |

---

## Adding Your Own Aliases

You are encouraged to add new aliases if they simplify common team workflows. To deploy a new alias to the environment, you must update two files:

1. [**`docker/.bash_aliases`**](../../docker/.bash_aliases): Add your new alias or function definition here. This file is copied into the Docker container during the build process.
2. [**`docs/software/aliases`**](./aliases): Update this documentation table so the rest of the team knows the shortcut exists.

**⚠️ Preventing Collisions:**
Before adding a new alias, you must ensure that your chosen shortcut does not conflict with existing aliases, standard Linux utilities, or built-in ROS 2 CLI tools.

To verify a shortcut is safe to use, run the `type` command in your terminal before adding it:

```bash
type your_proposed_alias
```

*If the terminal returns "not found," the alias is safe to use.*