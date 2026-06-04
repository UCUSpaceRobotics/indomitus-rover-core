# Jetson Nano Deployment Script (`deploy_to_jetson.sh`)

The `deploy_to_jetson.sh` script automates the offline deployment pipeline for the Indomitus Rover. It cross-compiles the ROS 2 application into an ARM64 image using your laptop's resources, compresses the image, transfers the production payload to the offline Jetson Nano over a hotspot connection, loads the image into the Jetson's Docker engine, and cleans up temporary files to save disk space.

In full deploy mode, the script also syncs the repository's `src/` directory to the Jetson. This is required because the production compose file bind-mounts `../src` into `/opt/ws/src` inside the container. If the host-side `src/` directory is missing or empty, Docker will mount an empty directory and hide the source code baked into the image.

## Adding Execution Rights

Before running the script for the first time, you must grant it execution permissions. Open a terminal at the root of your repository and run:

```bash
chmod +x scripts/deploy_to_jetson.sh
```

## Running the Script

Thanks to automatic path resolution, you can run this script from **any folder on your computer**. The script will automatically locate the repository root.

Ensure your laptop is connected to a Wi-Fi network with internet access to build the image. Once the image is built, the script will automatically attempt to connect to the Jetson hotspot for the transfer.

To run the script using the default configuration:

```bash
./scripts/deploy_to_jetson.sh
```

## Configuration Flags

The script is pre-configured with default values matching the standard repository layout. You can override any of these defaults using the following flags.

**Important:** Because the script can be run from anywhere, all local file paths provided via flags **MUST** be relative to the root of the repository.

* `-i, --ip IP` : The Jetson Nano IP address over the hotspot. (Default: 10.42.0.1)
* `-u, --user USER` : The Jetson Nano SSH username. (Default: ros)
* `-d, --dir DIR` : Remote deployment directory on the Jetson. Can be an absolute path (e.g., `/opt/rover`) or relative to the user's home folder. (Default: `/home/ros/Indomitus/indomitus-rover-core/`)
* `-n, --name NAME` : The base Docker image name. (Default: indomitus-rover)
* `-t, --tag TAG` : The Docker image tag for the production build. (Default: humble-prod)
* `-f, --file FILE` : The path to the Dockerfile, relative to the repository root. (Default: docker/Dockerfile)
* `-c, --compose FILE` : The path to the Production Compose file, relative to the repository root. (Default: docker/docker-compose.prod.yaml)
* `-w, --ssid SSID` : Wi-Fi SSID of the Jetson hotspot to automatically connect to. (Default: JetsonRosIndomitus)
* `-p, --pass PASS` : Wi-Fi password for the Jetson hotspot. (Default: jetson1234)
* `-h, --help` : Display the help message and exit.

## Deployment Notes

* **Sync mode (`--sync`)** sends only `src/` and rebuilds inside the Jetson container.
* **Full deploy mode** sends the Docker image, the production compose file, and `src/` so the `../src` bind mount resolves correctly on the Jetson.