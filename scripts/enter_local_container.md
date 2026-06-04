# Local Container Entry Script (`enter_local_container.sh`)

The `enter_local_container.sh` script ensures the production Docker container is running locally, optionally builds joystick-related ROS packages inside the container, and opens an interactive shell with a ready-to-run launch command.

## Adding Execution Rights

Before running the script for the first time, grant it execution permissions. From the repository root run:

```bash
chmod +x scripts/enter_local_container.sh
```

This container expects the Jetson-side `src/` directory to be mounted directly over `/opt/ws/src` inside the Docker container. That works only when `deploy_to_jetson.sh` full deploy has already copied `src/` to the Jetson, because `docker/docker-compose.prod.yaml` uses a `../src:/opt/ws/src` bind mount.

## Running the Script

You can run this script from any folder; it will locate the repository root automatically.

By default the script expects the production compose layout and container names. It will start or create the container if necessary, optionally build selected ROS packages inside the container (using `--build`), then open an interactive shell. Inside the container you can run the joystick launch with the printed command.

To run the script with defaults:

```bash
./scripts/enter_local_container.sh
```

## Defaults & Behaviour

- **Container name:** `rover_prod`
- **Compose file:** `docker/docker-compose.prod.yaml` (path relative to repo root)
- **ROS distro:** `humble`
- **Workspace dir (inside container):** `/opt/ws`
- **Packages built by default:** `rover_control`, `rover_bringup`
- **Launch package/file:** `rover_bringup` / `joy.launch.py`

## Configuration Flags

The script accepts the following flags to override defaults:

* `-n, --name NAME` : Docker container name. (Default: `rover_prod`)
* `-c, --compose FILE` : Path to Compose file, relative to repo root. (Default: `docker/docker-compose.prod.yaml`)
* `-r, --ros-distro DIST` : ROS 2 distribution name. (Default: `humble`)
* `-w, --workspace DIR` : ROS 2 workspace path inside the container. (Default: `/opt/ws`)
* `--build` : Build joystick packages inside the container before opening the terminal.
* `-h, --help` : Display the help message and exit.