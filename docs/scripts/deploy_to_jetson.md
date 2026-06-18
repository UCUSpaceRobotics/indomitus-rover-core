# Jetson Nano Deployment Script (`deploy_to_jetson.sh`)

The `deploy_to_jetson.sh` script automates the offline deployment pipeline for the Indomitus Rover. Because the Jetson Nano is often operated offline via a hotspot, this script bridges the gap by handling the heavy lifting on your local laptop, transferring the payload over the Wi-Fi hotspot, and spinning up the production container.

The script ensures the `src/` directory is properly synced to the Jetson so the `../src` bind mount inside the production compose file resolves correctly. 

## Adding Execution Rights

Before running the script for the first time, you must grant it execution permissions. Open a terminal at the root of your repository and run:

```bash
chmod +x scripts/deploy_to_jetson.sh
```

## Running the Script

Thanks to automatic path resolution, you can run this script from **any folder on your computer**. The script will automatically locate the repository root.

Ensure your laptop is connected to a Wi-Fi network with internet access to build or download the image. Once the image is built or downloaded, the script will automatically attempt to connect to the Jetson hotspot for the transfer.

To run the script using the default configuration (Full Build Mode):

```bash
./scripts/deploy_to_jetson.sh
```

## Configuration Flags

The script is pre-configured with default values matching the standard repository layout. You can override any of these defaults using the following flags.

**Important:** Because the script can be run from anywhere, all local file paths provided via flags **MUST** be relative to the root of the repository.

* `-S, --sync` : **SYNC MODE**: Skips the Docker build. Syncs the `src` folder and auto-compiles on the Jetson.
* `-P, --pull` : **PULL MODE**: Laptop pulls the pre-built image from GHCR, transfers it via archive, and loads it on the Jetson.
* `-i, --ip IP` : The Jetson Nano IP address over the hotspot. (Default: `10.42.0.1`)
* `-u, --user USER` : The Jetson Nano SSH username. (Default: `ros`)
* `-d, --dir DIR` : Remote deployment directory on the Jetson. (Default: `/home/ros/Indomitus/indomitus-rover-core/`)
* `-n, --name NAME` : The base Docker image name. (Default: `ghcr.io/ucuspacerobotics/indomitus-rover-core`)
* `-t, --tag TAG` : The Docker image tag. (Default: `local-prod` for full builds, `develop-prod` for sync/pull modes).
* `-f, --file FILE` : Path to the Dockerfile. (Default: `docker/Dockerfile`)
* `-c, --compose FILE` : Path to the Production Compose file. (Default: `docker/docker-compose.prod.yaml`)
* `-w, --ssid SSID` : Wi-Fi SSID of the Jetson hotspot to automatically connect to. (Default: `JetsonRosIndomitus`)
* `-p, --pass PASS` : Wi-Fi password for the Jetson hotspot. (Default: `jetson1234`)
* `-h, --help` : Display the help message and exit.

---

## Deployment Modes

The script supports three distinct deployment strategies depending on your current development needs:

### 1. Full Image Deployment (Default)

Runs when you execute the script without `-S` or `-P`.

* **Use Case:** You have made deep system changes (e.g., modified the `Dockerfile`, installed new dependencies) and need a fresh image.
* **Process:** Cross-compiles the ROS 2 application into an ARM64 image using your laptop's resources (via QEMU). It exports the image as a `.tar` archive, transfers it and the `src/` directory to the Jetson, loads it into the Jetson's Docker engine, cleans up old dangling images, and restarts the container.

### 2. Pull & Bridge Mode (`--pull` or `-P`)

* **Use Case:** An image has already been successfully built by CI/CD (GitHub Actions) and you want to deploy that remote stable version of the image to the Jetson.
* **Process:** Your internet-connected laptop pulls the specific ARM64 manifest from GHCR (`ghcr.io/...`). It then exports the downloaded image to an archive, switches to the Jetson network, transfers the archive alongside your `src/` files, and spins up the container on the Jetson.

### 3. Rapid Code Sync Mode (`--sync` or `-S`)

* **Use Case:** You are making frequent, small logic changes in the source code (e.g., Python scripts, C++ nodes) and do not need to rebuild the entire Docker image.
* **Process:** Completely bypasses Docker image creation and transfer. It simply uses `rsync` to copy your local `src/` directory to the Jetson, and directly triggers a `colcon build --symlink-install` inside the *already running* production container on the Jetson.