# Container Entry Script (`enter_container.sh`)

The `enter_container.sh` script provides an interface to ensure the required Docker container is running and opens an interactive shell. It supports both local development and remote execution on the rover computer.

## Running the Script

You can run this script from any folder; it will locate the repository root automatically. The script requires a subcommand to specify your target environment: `local` or `rover`.

It will automatically start or create the container if necessary, and then open an interactive shell. If using the `rover` command, it will also automatically attempt to connect to the Jetson's Wi-Fi hotspot and establish an SSH connection.

Enter the local development container:

```bash
./scripts/enter_container.sh local
```


Enter the remote production container on the rover computer:

```bash
./scripts/enter_container.sh rover
```

## Configuration Flags & Defaults

The script accepts the following flags to override defaults depending on the subcommand used.

| Flag | Applies To | Description | Default |
| --- | --- | --- | --- |
| **`-n, --name`** | `local`, `rover` | Docker container name | `rover_dev` (local), `rover_prod` (rover) |
| **`-c, --compose`** | `local`, `rover` | Path to Compose file | `docker-compose.yaml` (local), `docker-compose.prod.yaml` (rover) |
| **`-r, --ros-distro`** | `local` | ROS 2 distribution name | `humble` |
| **`-w, --workspace`** | `local` | ROS 2 workspace path inside container | `/opt/ws` |
| **`-u, --user`** | `rover` | Jetson SSH username | `indomitus-rover` |
| **`-i, --ip`** | `rover` | Jetson IP address | `10.42.0.1` |
| **`-d, --dir`** | `rover` | Remote deployment directory | `/home/indomitus-rover/indomitus-rover-core/` |
| **`-w, --ssid`** | `rover` | Wi-Fi SSID of the Jetson hotspot | `IndomitusRover` |
| **`-p, --pass`** | `rover` | Wi-Fi password for the hotspot | `12345678` |
| **`-h, --help`** | `local`, `rover` | Display the help message and exit | N/A |
