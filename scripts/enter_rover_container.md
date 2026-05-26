# Rover Container Entry Script (`enter_rover_container.sh`)

The `enter_rover_container.sh` script connects to the Jetson Nano over the hotspot, starts the production Docker container if it is not already running on the Jetson, and opens an interactive shell inside the container for manual commands and debugging.

## Adding Execution Rights

Before running the script for the first time, grant it execution permissions. From the repository root run:

```bash
chmod +x scripts/enter_rover_container.sh
```

## Running the Script

You can run this script from any folder; it will locate the repository root automatically.

Ensure your laptop is connected to the Jetson hotspot before running it. The script will attempt to connect to the hotspot automatically when possible, then wait for SSH access to the Jetson and start the production container if needed.

To run the script with defaults:

```bash
./scripts/enter_rover_container.sh
```

Once connected, the script opens an interactive terminal inside the container. From there you can launch the rover manually, for example:

```bash
ros2 launch indomitus_rover_bringup rover.launch.py
```

## Configuration Flags

The script is pre-configured with default values matching the standard repository layout. You can override any of these defaults using the following flags.

* `-i, --ip IP` : The Jetson Nano IP address over the hotspot. (Default: 10.42.0.1)
* `-u, --user USER` : The Jetson Nano SSH username. (Default: ros)
* `-d, --dir DIR` : Remote deployment directory on the Jetson. (Default: `/home/ros/Indomitus/indomitus-rover-core/`)
* `-n, --name NAME` : The Docker container name to enter. (Default: `indomitus_rover_prod`)
* `-c, --compose FILE` : The path to the Compose file, relative to the repository root. (Default: `docker/docker-compose.prod.yaml`)
* `-w, --ssid SSID` : Wi-Fi SSID of the Jetson hotspot to automatically connect to. (Default: `JetsonRosIndomitus`)
* `-p, --pass PASS` : Wi-Fi password for the Jetson hotspot. (Default: `jetson1234`)
* `-h, --help` : Display the help message and exit.