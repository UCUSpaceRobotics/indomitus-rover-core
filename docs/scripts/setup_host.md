# setup_host.sh

Deploys system config (`system/`) to the Jetson rover or to the local machine.
The actual install is done by `system/setup.sh`, which runs on the target.

## Usage

```
./scripts/setup_host.sh {rover,local} [--udev A,B] [--service A,B] [--all] [OPTIONS]
```

## Targets

| Target  | Description |
|---|---|
| `rover` | Connects to the Jetson over Wi-Fi/SSH, copies `system/`, runs `setup.sh` remotely |
| `local` | Runs `setup.sh` directly on this machine (no SSH/Wi-Fi) |

## What to install

| Flag | Description |
|---|---|
| `--udev A,B` | Rules `system/rules.d/[NN-]A.rules`, ... (`can`, `playstation-led`, `cameras`, `arducam-no-superspeed`) |
| `--service A,B` | Units `system/systemd/A.service`, ... (e.g. `rover`) |
| `--all` | Every rule and unit in `system/` |

Names are file names without the `NN-` prefix and extension, comma-separated
(flags may also be repeated). Names are checked locally before any Wi-Fi/SSH step.

Every file of a kind goes through the same steps — a new rule or unit dropped
into `system/` needs no script changes:

- **udev rule**: copy to `/etc/udev/rules.d` → `udevadm control --reload-rules`
  → `udevadm trigger --action=add` scoped to the `SUBSYSTEM==` / `KERNEL==` /
  `ATTR{..}==` keys of each rule line (so devices already plugged in get the rule without
  replugging; unrelated devices are not replayed) → `udevadm settle`.
  A rule line with no `SUBSYSTEM==` key is not triggered (warning printed).
- **systemd unit**: copy to `/etc/systemd/system` → `daemon-reload` → `stop`
  → `disable`. Units are **always** left stopped and disabled; start/enable
  them by hand.

## Other options

| Flag | Description |
|---|---|
| `-i, --ip IP` / `-u, --user USER` | Jetson address / SSH user |
| `-w, --ssid` / `-p, --pass` | Wi-Fi credentials for auto-connect |
| `--no-wifi, -n` | Skip the Wi-Fi auto-connect and ssh straight to the Jetson |
| `-h, --help` | Show help |

`setup.sh` refuses to run inside a container (udev/systemd belong to the host);
from distrobox use `distrobox-host-exec ./scripts/setup_host.sh local ...`.

## Examples

```
./scripts/setup_host.sh rover --udev can --service rover-launch-core
./scripts/setup_host.sh local --udev can,playstation-led
./scripts/setup_host.sh rover -i 10.42.0.5 -u myuser --service rover-launch-core
./scripts/setup_host.sh rover --all
```
