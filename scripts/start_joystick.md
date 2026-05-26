# Local Joystick Startup Script (`start_joystick.sh`)

The `start_joystick.sh` script starts the local Docker Compose stack, enters the running container, builds the joystick stack, and launches it interactively so you can stop it naturally with Ctrl+C.

## Adding Execution Rights

Before running the script for the first time, you must grant it execution permissions. Open a terminal at the root of your repository and run:

```bash
chmod +x scripts/start_joystick.sh
```

## Running the Script

Thanks to automatic path resolution, you can run this script from **any folder on your computer**. The script will automatically locate the repository root.

By default, the script uses the `docker-compose.yml` file in the repository root, starts the `indomitus_rover_dev` service, builds the joystick-related ROS packages inside the container, and launches `joy.launch.py`.

The joystick device path is passed through to the launch file as `joy_dev`, with `/dev/input/js0` as the default.

To run the script using the default configuration:

```bash
./scripts/start_joystick.sh
```

Once launched, the node runs in the foreground. Press `Ctrl+C` to stop it cleanly.

## Configuration Flags

The script is pre-configured with default values matching the standard repository layout. You can override any of these defaults using the following flags.

* `-c, --compose FILE` : Path to the Compose file. (Default: `docker-compose.yml`)
* `-s, --service NAME` : Docker Compose service name. (Default: `indomitus_rover_dev`)
* `-w, --workdir DIR` : Workspace directory inside the container. (Default: `/work`)
* `-l, --launch FILE` : Bringup launch file to run. (Default: `joy.launch.py`)
* `-j, --joystick DEV` : Joystick device path passed to `joy.launch.py`. (Default: `/dev/input/js0`)
* `-h, --help` : Display the help message and exit.