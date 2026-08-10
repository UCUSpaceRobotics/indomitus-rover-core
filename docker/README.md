
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
curl -fsSL [https://nvidia.github.io/libnvidia-container/gpgkey](https://nvidia.github.io/libnvidia-container/gpgkey) | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L [https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list](https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list) | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
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