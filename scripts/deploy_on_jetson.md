# Jetson Nano Deployment Script (`deploy_to_jetson.sh`)

The `deploy_to_jetson.sh` script automates the offline deployment pipeline for the Indomitus Rover. It cross-compiles the ROS 2 application into an ARM64 image using your laptop's resources, compresses the image, securely transfers the payload to the offline Jetson Nano over a hotspot connection, loads the image into the Jetson's Docker engine, and cleans up temporary files to save disk space.

## Adding Execution Rights

Before running the script for the first time, you must grant it execution permissions. Open a terminal at the root of your repository and run:

```bash
chmod +x scripts/deploy_to_jetson.sh
```

## Running Script

Ensure your laptop is disconnected from the internet and connected to the Jetson Nano's Wi-Fi hotspot. Run the script from the root of your repository.

To run the script using the default configuration:

```bash
./scripts/deploy_to_jetson.sh
```

## Configuration Flags

The script is pre-configured with default values matching the standard repository layout. You can override any of these defaults using the following flags:

* `-i IP` : The Jetson Nano IP address over the hotspot. (Default: 10.42.0.1)
* `-u USER` : The Jetson Nano SSH username. (Default: jetson_username)
* `-d DIR` : The remote deployment directory created on the Jetson. (Default: rover_deploy)
* `-n NAME` : The base Docker image name. (Default: indomitus-rover)
* `-t TAG` : The Docker image tag for the production build. (Default: humble-prod)
* `-f FILE` : The relative path to the Dockerfile. (Default: docker/Dockerfile)
* `-c FILE` : The relative path to the Production Compose file. (Default: docker/docker_compose.prod.yaml)
