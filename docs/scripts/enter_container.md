# Container Entry Script (`enter_container.sh`)

The `enter_container.sh` script ensures the target Docker container is running and opens an interactive shell. It supports both local development and remote execution on the rover computer.

## Running the Script

You can run this script from any folder; it will locate the repository root automatically. The script requires a subcommand to specify your target environment: `local` or `rover`.

It will automatically start or create the container if necessary, and then open an interactive shell. If using the `rover` command, it will attempt to connect to the Jetson hotspot and establish SSH unless Ethernet mode is enabled.

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
| **`--name`** | `local`, `rover` | Docker container name | `rover_dev` (local), `rover_prod` (rover) |
| **`--compose`** | `local`, `rover` | Compose file path | `docker-compose.yaml` (local), `docker-compose.prod.yaml` (rover) |
| **`--ros-distro`** | `local` | ROS 2 distribution name | `humble` |
| **`--workspace`** | `local`, `rover` | ROS 2 workspace path inside container | `/opt/ws` |
| **`--user`** | `rover` | Jetson SSH username | `jetson` |
| **`--ip`** | `rover` | Jetson target IP/hostname | `10.42.0.1` |
| **`--eth`** | `rover` | Use Ethernet target and skip hotspot auto-connect | `off` |
| **`--dir`** | `rover` | Remote deployment directory | `/home/jetson/indomitus-rover-core/` |
| **`--ssid`** | `rover` | Wi-Fi SSID of the Jetson hotspot | `IndomitusRover` |
| **`--pass`** | `rover` | Wi-Fi password for the hotspot | `12345678` |
| **`-h, --help`** | `local`, `rover` | Display help and exit | N/A |

## Notes

- In rover mode, `--eth` sets the target to `nano-4gb-jp451.local` and disables Wi-Fi auto-connect logic.
- In rover mode, `--ip` always sets the final target address based on argument order (the last assignment wins).
- Both local and rover shells source ROS and workspace setup files before opening an interactive shell.
