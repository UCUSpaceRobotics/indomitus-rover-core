
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

## Base images

| image | base | GPU |
|---|---|---|
| `…-dev` (amd64, laptops, sim) | `stereolabs/zed:5.4-runtime-cuda12.8-ubuntu24.04` + ROS 2 Jazzy from apt | CUDA 12.8, ZED SDK |
| `…-prod` (arm64, Jetson) | `dustynv/onnxruntime:1.22-r36.4.0-cu128-24.04` + ROS 2 Jazzy from apt + ZED SDK (`ZED_SDK_URL`, `docker/install_zed_sdk.sh`) | CUDA 12.8, cuDNN 9.7, TensorRT 10.7, onnxruntime, ZED SDK |

The Jetson runs JetPack 6.2 (Ubuntu 22.04, L4T r36.4) on the host; the container is Ubuntu
24.04 for ROS 2 Jazzy. The GPU driver comes from the host (`runtime: nvidia` in
`docker-compose.prod.yaml`); CUDA and the libraries on top of it come from the image, built for
L4T r36 by [jetson-containers](https://github.com/dusty-nv/jetson-containers) (CUDA 12.8 runs on
JetPack 6.2's driver through `/usr/local/cuda/compat`).

- `nvidia/cuda:*-ubuntu24.04` does **not** work: it is the SBSA (server Arm) build, and kernels
  fail on the Jetson's GPU ("no kernel image is available for execution on the device").
- `dustynv/ros:jazzy-*` does not fit either: its ROS is built from source, so rosdep and apt ROS
  packages (ros2_control, MoveIt, ...) do not mix with it. The Dockerfile installs ROS from apt
  when the base image has none.
- Stereolabs ships no Ubuntu 24.04 image for JetPack 6 (`stereolabs/zed:*-l4t-r36.4` is 22.04),
  so the Dockerfile installs the ZED SDK for L4T 36.4 into the 24.04 base from `ZED_SDK_URL`.
- jetson-containers keep Python packages in `/opt/venv`; the Dockerfile makes them visible to
  `/usr/bin/python3` (which ROS nodes use) through a `.pth` file and takes the venv off `PATH`.

Check the GPU inside `rover_ai`:

```bash
python3 -c "import onnxruntime as o; print(o.get_available_providers())"   # expect CUDA/TensorRT
python3 -c "import ctypes; print('cuInit', ctypes.CDLL('libcuda.so.1').cuInit(0))"   # 0 = OK
```

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