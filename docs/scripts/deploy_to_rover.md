# Deployment Script (`deploy_to_rover.sh`)

The `deploy_to_rover.sh` script automates the offline deployment pipeline for the Indomitus Rover. Because the Jetson is operated offline via a hotspot, this script bridges the gap by handling the heavy lifting on your local laptop, transferring the payload over the Wi-Fi hotspot, and spinning up the production container.

The script ensures the `src/` directory is properly synced to the Jetson so the `../src` bind mount inside the production compose file resolves correctly.

## Requirements

The script relies on an `.rsync-filter` file located in the same directory as the script. This file tells `rsync` which local folders and files (like `__pycache__/`, `.git/`, or `log/`) to ignore so they aren't accidentally transferred to the Jetson. The script will throw an error if this file is missing.

## Adding Execution Rights

Before running the script for the first time, you must grant it execution permissions. Open a terminal at the root of your repository and run:

```bash
chmod +x scripts/deploy_to_rover.sh
```

## Running the Script

Thanks to automatic path resolution, you can run this script from **any folder on your computer**. The script will automatically locate the repository root.

Ensure your laptop is connected to a Wi-Fi network with internet access to build or download the image (for pull/build modes). Once the image is ready, the script will automatically attempt to connect to the Jetson hotspot for the transfer.

To run the script using the default configuration (Full Build Mode):

```bash
./scripts/deploy_to_rover.sh
```

## Configuration Flags

The script is pre-configured with default values matching the standard repository layout. You can override any of these defaults using the following flags.

**Important:** All local file paths provided via flags **MUST** be relative to the root of the repository.

### Action Modes

* `--sync` : **SYNC ALL**: Syncs both local `src` and the compose file, safely restarts the container, and auto-compiles on the Jetson.
* `--sync-src` : **SYNC SRC**: Syncs ONLY the local `src` directory and auto-compiles inside the *already running* container.
* `--sync-docker-compose` : **SYNC COMPOSE**: Syncs ONLY the compose file and cleanly restarts the container.
* `--pull` : **PULL MODE**: Pulls a pre-built image from GHCR, clones clean code directly from GitHub, and transfers everything to the Jetson.

### Options

* `-i, --ip IP` : The Jetson IP address over the hotspot. (Default: `10.42.0.1`)
* `-u, --user USER` : The Jetson SSH username. (Default: `indomitus-rover`)
* `-d, --dir DIR` : Remote deployment directory on the Jetson. (Default: `/home/indomitus-rover/indomitus-rover-core/`)
* `--image-name NAME` : The base Docker image name. (Default: `ghcr.io/ucuspacerobotics/indomitus-rover-core`)
* `-t, --tag TAG` : The Docker image tag. (Default: `local-prod` for full builds, `develop-prod` for sync/pull modes).
* `--container-name NAME`: The name of the Docker container on the Jetson. (Default: `rover_prod`)
* `-f, --file FILE` : Path to the local Dockerfile. (Default: `docker/Dockerfile`)
* `-c, --compose FILE` : Path to the Production Compose file. (Default: `docker/docker-compose.prod.yaml`)
* `-w, --ssid SSID` : Wi-Fi SSID of the Jetson hotspot to automatically connect to. (Default: `IndomitusRover`)
* `-p, --pass PASS` : Wi-Fi password for the Jetson hotspot. (Default: `12345678`)
* `-h, --help` : Display the help message and exit.

---

## Deployment Strategies

The script supports five distinct deployment strategies depending on your current development needs:

### 1. Rapid Source Sync (`--sync-src`)

* **When to use:** You modified Python scripts, C++ nodes, or launch files and did *not* change the Docker configuration or the dependencies in `package.xml` files. Ideal for iterative, day-to-day testing of code logic.
* **What it does:** Bypasses Docker builds and restarts entirely. It Securely transfer only your modified source code and triggers a `colcon build --symlink-install` directly inside the running container. Fastest mode.

### 2. Infrastructure Sync (`--sync-docker-compose`)

* **When to use:** You only changed the `docker-compose.prod.yaml` file (e.g., adding a new volume mount, changing an environment variable, or updating device privileges) and do not need to sync source code or rebuild the container image.
* **What it does:** Transfers the updated compose file to the Jetson and executes a safe `docker compose down` followed by `docker compose up -d --wait`.

### 3. Full Sync (`--sync`)

* **When to use:** You modified both your source code and your `docker-compose` configurations, but still don't need a heavy, from-scratch image rebuild.
* **What it does:** Combines the two steps above. It syncs the `src` folder, syncs the compose file, tears down and safely restarts the container infrastructure, and finally executes the compilation step inside the fresh container.

### 4. Pull & Bridge (`--pull`)

* **When to use:** You want to deploy a pre-built image generated by GitHub workflows with stable code from a specific branch or tag, *without* deploying your laptop's dirty local workspace.
* **What it does:** Uses your laptop's internet to resolve and pull the ARM64 image manifest from GHCR. It clones a fresh, clean copy of the `src/` code directly from the repository at the matching tag, bypassing your local files entirely. It exports the image to a `.tar` archive, securely transfers the clean codebase, compose file, and archive to the Jetson, and loads the fresh infrastructure.

### 5. Full Image Build (Default: no flags)

* **When to use:** You modified system dependencies, `package.xml` requirements, or the `Dockerfile` itself locally, and need a completely fresh system environment to test code that is not yet pushed to GitHub.
* **What it does:**It cross-compiles a brand new ARM64 Docker image on the laptop. Then packages the image, syncs your local `src/` folder and compose file, transfers the payload to the Jetson, loads the newly built image, prunes old dangling images, and spins up the new container infrastructure. Slowest mode.