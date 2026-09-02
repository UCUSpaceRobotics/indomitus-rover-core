
# Docker

## !!!ATTENTION!!!

> Each user may need to set up `docker-compose` individually — drivers and dependencies vary by platform.

> Run the following command to copy `docker/docker-compose.dev.example.yaml` to the project root and rename it to `docker-compose.yaml`.

```bash
cp ./docker/docker-compose.dev.example.yaml ./docker-compose.yaml
```

## Dependencies
```bash
curl -fsSL https://get.docker.com | sh  # installs docker + compose plugin
sudo usermod -aG docker $USER           # run docker without sudo (re-login required)
```

## Quick Reference

| Action | Command |
|--------|---------|
| Build | `docker compose build` |
| Force rebuild | `docker compose build --no-cache` |
| Create + start | `docker compose up -d` |
| Enter container | `docker compose exec rover_dev bash` |
| Stop (keep container) | `docker compose stop` |
| Start stopped container | `docker compose start` |
| Stop + delete | `docker compose down` |

> 💡 You can enter the same container from multiple terminals simultaneously.

## DDS discovery across the mast link

`docker-compose.prod.yaml` points Fast DDS at `/opt/dds/fastdds_rover_link.xml`,
written by `gen-dds-profile.sh` on every container start. It is not committed:
it names literal addresses (Fast DDS 2.6 accepts no CIDR) and every port in it
is derived from `ROS_DOMAIN_ID`, so a checked-in copy would rot silently.

Why it is needed: the mast Pi is an L3 **router** between the rover's
`10.42.0.0/24` and the ground station's `10.44.0.0/24`, and routers do not
forward multicast — so SPDP discovery finds nothing on its own. The profile
adds explicit unicast discovery peers for the GS, and whitelists this rover's
link address so Fast DDS stops advertising the wired lifeline and docker
bridges, which the GS cannot route to.

The ground station runs the **same script** with the two knobs mirrored:

| | `ROVER_LINK_PREFIX` | `ROVER_PEER` |
|---|---|---|
| Rover (here) | `10.42.0.` | `10.44.0.10` (the GS PC) |
| GS PC | `10.44.0.` | `10.42.0.1` (this rover) |

"Rover link" names the *path*, not the machine at the far end.

> **Keep in sync.** `docker/gen-dds-profile.sh` is duplicated in
> indomitus-ground-station. The rover mounts no checkout of that repo in the
> field, so the script ships inside this image instead. The two copies are meant
> to be byte-identical — diff them before trusting either.

Notes:

* The container waits up to `LINK_WAIT_SECS` (60) for `10.42.0.1` to appear,
  because it restarts on boot and races `rover-ap.service`. With no link it
  falls back to a local-only profile so same-host discovery still works.
* A participant whose ID falls outside `ROVER_PEER_RANGE` (0–49) is invisible
  to the other end with no fallback. Raise it if the GS ever runs more.
* `docker-compose.dev.gs.yaml` (this container on the GS laptop) deliberately
  leaves `DDS_PROFILE_OUT` unset and reads the GS's own profile over a
  read-only mount, so nothing there regenerates a file it does not own.
* A missing profile does **not** stop Fast DDS — it logs one XMLPARSER line and
  falls back to multicast, which cannot cross the Pi. Every symptom then points
  at the network, so the entrypoint fails hard instead.

## Display Access (for GUI / RViz)

Allow Docker to use your screen before entering the container:

```bash
xhost +local:docker
```

## Troubleshooting

* Commands not found? Try `docker-compose` (with -) or prepend sudo.
* Container already exists? `docker compose up -d` will just start it, not recreate.
* `Error: could not select device driver "nvidia" with capabilities: [[gpu]]`

Docker cannot access your GPU by default. If you encounter this error, you must install the NVIDIA Container Toolkit so Docker can bridge to your host's NVIDIA drivers.

> **Note:** Make sure your host NVIDIA drivers are already installed via `sudo ubuntu-drivers autoinstall` before proceeding.

**1. Add the NVIDIA repository keys**

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

**2. Install the toolkit**
```bash
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
```

**3. Configure Docker to use the NVIDIA runtime and restart**
```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```