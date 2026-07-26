# setup_host.sh

Deploys system config (`system/`) to the Jetson rover or to the local machine.

## Usage

```
./scripts/setup_host.sh {rover,local} [OPTIONS]
```

## Targets

| Target  | Description |
|---|---|
| `rover` | Connects to the Jetson over Wi-Fi/SSH, copies `system/`, runs `setup.sh` remotely |
| `local` | Runs `setup.sh` directly on this machine (no SSH/Wi-Fi) |

## Options

| Flag | Description |
|---|---|
| `--can` | Deploy/configure CAN rules only |
| `--service` | Deploy/configure rover systemd service only |
| `-h, --help` | Show help |

For other flags use `-h` flag.

## Examples

```
./deploy.sh rover --can --service
./deploy.sh local --can
./deploy.sh rover -i 10.42.0.5 -u myuser --service
```
