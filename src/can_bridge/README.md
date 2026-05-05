# can_bridge

## Docker requirements

The docker compose file must include:

- `network_mode: host` so the container can reach the host CAN interface.
- `cap_add: NET_ADMIN` so the CAN link can be configured and brought up/down.
- `cap_add: NET_RAW` so the nodes can use raw CAN sockets.

Snippet for the `docker-compose.yml` 
```
network_mode: host
cap_add:
  - NET_ADMIN
  - NET_RAW 
```

## What this node setup does

This package launches two lifecycle nodes from `ros2_socketcan`:

- `socket_can_sender` sends ROS-originated CAN frames to the Linux SocketCAN interface.
- `socket_can_receiver` reads frames from SocketCAN and publishes them into ROS.

Both nodes are auto-configured and auto-activated by the launch file.

## How to launch

The launch script `can_bridge.launch.py` starts both SocketCAN bridge lifecycle nodes (`socket_can_sender` and `socket_can_receiver`), then configures and activates them automatically on the selected interface.

1. Build and source your workspace.
2. Ensure CAN interface is up (see script below).
3. Run:

```bash
ros2 launch can_bridge can_bridge.launch.py interface:=can0
```

## Parameters

Launch argument:

- `interface` (default: `can0`): SocketCAN interface name.

Node parameters set by launch:

- Sender (`socket_can_sender`):
	- `interface`: from launch argument
	- `timeout_sec`: `0.01`
- Receiver (`socket_can_receiver`):
	- `interface`: from launch argument
	- `interval_sec`: `0.01`

## Script note: `scripts/setup_can.sh`

This helper script brings up and configures a SocketCAN interface.

Usage:

```bash
./scripts/setup_can.sh [interface] [bitrate]
```

Defaults:

- `interface`: `can0`
- `bitrate`: `1000000`

Example:

```bash
./scripts/setup_can.sh can0 1000000
```

The script uses `sudo ip link` commands, so it must run on the host with permissions to configure networking.
