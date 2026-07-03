# Deployment Script (`deploy_to_rover.sh`)

The `deploy_to_rover.sh` script automates the deployment pipeline for the Indomitus Rover. It bridges the gap between your local development environment and the Jetson, handling code synchronization, remote or local image building, and container orchestration over a local Wi-Fi hotspot or wired Ethernet connection.


## Requirements

The script relies on an `.rsync-filter-deploy` file located in the same directory as the script. This file tells `rsync` which local folders and files (like `__pycache__/`, `.git/`, or `log/`) to ignore so they aren't accidentally transferred to the Jetson. The script will throw an error if this file is missing.


## Running the Script

Ensure your laptop is connected to Wi-Fi with internet access, the Jetson is turned on, and either its hotspot is active or you are connected to it via Ethernet.

To run the script, you must specify exactly one deployment mode:

```bash
./scripts/deploy_to_rover.sh [MODE] [OPTIONS]
```


## Configuration Flags

The script is pre-configured with default values matching the standard repository layout. You can override any of these defaults using the following flags.

**Important:** All local file paths provided via flags **MUST** be relative to the root of the repository.

### Action Modes (Required)

* `remote-build` : **REMOTE BUILD**: Syncs the entire local repository to the Jetson, builds the image natively using Compose, and restarts the container.
* `pull` : **PULL MODE**: Pulls a pre-built image from GHCR, intelligently extracts the exact commit SHA, clones clean code directly from GitHub, and transfers everything to the Jetson.
* `sync-src` : **SYNC SRC**: Syncs ONLY the local `src/` directory and auto-compiles inside the *already running* container.
* `sync-docker-compose` : **SYNC COMPOSE**: Syncs ONLY the compose file and cleanly restarts the container.
* `local-build` : **LOCAL BUILD (Deprecated)**: Cross-compiles a new ARM64 image locally using QEMU emulators, packages it, transfers the payload, and spins it up.

### Options

* `--eth` : Use a wired Ethernet connection instead of the Wi-Fi hotspot.
* `--ip IP` : The Jetson IP address. (Default: `10.42.0.1`)
* `--user USER` : The Jetson SSH username. (Default: `indomitus-rover`)
* `--dir DIR` : Remote deployment directory on the Jetson. (Default: `/home/indomitus-rover/indomitus-rover-core/`)
* `--image-name NAME` : The base Docker image name. (Default: `ghcr.io/ucuspacerobotics/indomitus-rover-core`)
* `--tag TAG` : The Docker image tag (e.g., `develop-prod`, `feature-shared-branch-prod`). The script automatically derives the GitHub branch and exact commit from this tag.
* `--commit SHA` : Git commit SHA (7+ hex characters) to pull. Overrides `--tag` in `pull` mode.
* `--container-name NAME`: The name of the Docker container on the Jetson. (Default: `rover_prod`)
* `--dockerfile FILE` : Path to the local Dockerfile. (Default: `docker/Dockerfile`)
* `--compose FILE` : Path to the Production Compose file. (Default: `docker/docker-compose.prod.yaml`)
* `--ssid SSID` : Wi-Fi SSID of the Jetson hotspot to automatically connect to. (Default: `IndomitusRover`)
* `--pass PASS` : Wi-Fi password for the Jetson hotspot. (Default: `12345678`)
* `-h, --help` : Display the help message and exit.


## Deployment Strategies

The script supports five distinct deployment strategies depending on your current development needs:

### 1. Native Remote Build (`remote-build`)

* **When to use:** You modified the `Dockerfile`, system dependencies, or want to test unpushed system-level changes safely.
* **What it does:** Syncs your entire local repository to the Jetson, and natively builds the ARM64 image directly on the rover's hardware using Compose. This completely bypasses all local emulator bugs.
* **Prerequisites:** The Jetson must have internet access. It must either be connected to a router via Ethernet cable, or your laptop must be connected to the Jetson via Ethernet with internet connection sharing/forwarding enabled.

### 2. Pull & Bridge (`pull`)

* **When to use:** You want to deploy a pre-built stable image and clean code directly from GitHub.
* **What it does:** Uses your laptop's internet to pull the ARM64 image manifest from GHCR. It extracts the exact Git commit SHA from the image metadata, clones a fresh, clean copy of the `src/` code directly from the repository matching that commit, bypassing your local files entirely. It exports the image to an archive, securely transfers the clean codebase and infrastructure to the Jetson, and loads it.
* **Pro-Tip: Offload Builds to GitHub Actions**
You can build your images in the cloud instead of locally by utilizing GitHub Actions. First, push your branch to GitHub, navigate to the **Actions** tab, and select the **Publish Production And Development Images** workflow on the left. Click the **Run workflow** dropdown, choose your branch, and click the green button to trigger the cloud build. Once the build is successfully finished, you can deploy the new image to the Jetson using the script's pull mode. For example, use `--tag <branch-name>-prod` (ensuring any slashes in your branch name are replaced with dashes, like `--tag feature-shared-some-feature-prod`) to automatically deploy the image and the exact code commit on which that image was built.

### 3. Rapid Source Sync (`sync-src`)

* **When to use:** You only modified code (Python, C++, launch files) and did *not* change the Docker configuration or the dependencies in `package.xml` files. Ideal for iterative, day-to-day testing of code logic.
* **What it does:** Bypasses Docker builds and restarts entirely. It transfers only your modified source code and triggers a `colcon build --symlink-install` directly inside the running container. Fastest mode.

### 4. Infrastructure Sync (`sync-docker-compose`)

* **When to use:** You only changed the `docker-compose.prod.yaml` file (e.g., adding a new volume mount, changing an environment variable, or updating device privileges) and do not need to sync source code or rebuild the container image.
* **What it does:** Transfers the updated compose file to the Jetson and executes a safe `docker compose down` followed by `docker compose up -d --wait`.

### 5. Local Cross-Compile (`local-build`) — ⚠️ DEPRECATED

* **When to use:** There is no internet access on the Jetson and you cannot pull the image from GitHub for some reason.
* **What it does:** Cross-compiles a brand new ARM64 image on your laptop using QEMU emulators, packages it into a `.tar` archive, syncs your local repository, transfers the heavy payload to the Jetson, and spins it up. Slowest mode.

> ⚠️ **ATTENTION: QEMU SEGMENTATION FAULTS** ⚠️
> When using `local-build`, you are highly likely to experience a segmentation fault during C++ compilation. This is an unresolved bug with QEMU memory translation.
> To temporarily fix this, you **must** turn off virtual space randomization on your laptop before using `local-build`:
> ```bash
> sudo sysctl kernel.randomize_va_space=0
> ```
> 
> **Turn it back on immediately after deployment:**
> ```bash
> sudo sysctl kernel.randomize_va_space=2
> ```
> 
> *Note: You only need to do this only for the `local-build` mode.*


## Ethernet Connection

For faster file transfers, we recommend connecting your laptop to the Jetson via Ethernet (if possible). To use this method, connect the cable and append the `--eth` flag when running the script.

> Note: Your laptop requires additional setup before you can use Ethernet mode. To do this, refer to the...