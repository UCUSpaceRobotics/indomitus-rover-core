# Deployment Script (`deploy_to_rover.sh`)

The `deploy_to_rover.sh` script automates the deployment pipeline for the Indomitus Rover. It bridges the gap between your local development environment and the Jetson, handling code synchronization, remote or local image building, and container orchestration over a local Wi-Fi hotspot or wired Ethernet connection.

## This fork on the Jetson

This repository (`indomitus-rover-core-ai`) deploys **next to** the team's code, without touching it:

| | team (`indomitus-rover-core`) | this fork |
|---|---|---|
| directory on the Jetson | `~/indomitus-rover-core` | `~/indomitus-rover-core-ai` |
| container | `rover_prod` | `rover_ai` |
| image | team image (ROS 2 Humble) | **this fork's CI image** `ghcr.io/yuriifito/indomitus-rover-core-ai:develop-prod` (ROS 2 Jazzy, built on every push to `develop`), recorded in `~/indomitus-rover-core-ai/.env` |
| `build/`, `install/`, `log/` | inside the container | host directories in `~/indomitus-rover-core-ai` (survive container re-creation) |

- First time: `./scripts/deploy_to_rover.sh pull` (downloads `develop-prod` on the laptop, copies it to the Jetson, writes `.env`, starts `rover_ai`), then `./scripts/deploy_to_rover.sh sync-src` (syncs `src/`, builds inside `rover_ai`). After that, `docker compose up -d` / `down` in `~/indomitus-rover-core-ai` on the Jetson work on their own.
- `build/`, `install/`, `log/` are cleared automatically once when the image's ROS distro changes (marker `install/.ros_distro`), e.g. after moving from the team's Humble image to Jazzy.
- Both containers can exist, but **launch nodes in only one of them at a time**: they share host networking, `ROS_DOMAIN_ID` and the CAN buses. The script warns when `rover_prod` is running; stop it with `cd ~/indomitus-rover-core && docker compose down`.


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
* `pull` : **PULL MODE**: Pulls a pre-built image of this fork from GHCR (default tag `develop-prod`) and switches `rover_ai` to it; our `src/` on the Jetson is kept and rebuilt. (The original behaviour, cloning the team's code into `src/`, is removed in this fork.)
* `sync-src` : **SYNC SRC**: Syncs the local `src/` directory and the compose file, starts the container if it is not running (first-time setup), and compiles inside it.
* `sync-docker-compose` : **SYNC COMPOSE**: Syncs ONLY the compose file and cleanly restarts the container.
* `local-build` : **LOCAL BUILD (Deprecated)**: Cross-compiles a new ARM64 image locally using QEMU emulators, packages it, transfers the payload, and spins it up.

### Options

* `--eth` : Use a wired Ethernet connection instead of the Wi-Fi hotspot.
* `--ip IP` : The Jetson IP address. (Default: `10.42.0.1`)
* `--user USER` : The Jetson SSH username. (Default: `indomitus-rover`)
* `--dir DIR` : Remote deployment directory on the Jetson. (Default: `/home/${JETSON_USER}/indomitus-rover-core-ai/`)
* `--image-name NAME` : The Docker image name. (Default: `pull` → `ghcr.io/yuriifito/indomitus-rover-core-ai`, `remote-build`/`local-build` → `indomitus-rover-core-ai`, other modes → the Jetson's `.env`, else `ghcr.io/yuriifito/indomitus-rover-core-ai:develop-prod`)
* `--tag TAG` : The Docker image tag (e.g., `develop-prod`, `sha-1a2b3c4-prod`, or a version tag like `1.0.3-dev.1-prod`). Default for `pull`: `develop-prod`.
* `--commit SHA` : Git commit SHA (7+ hex characters) to pull. Overrides `--tag` in `pull` mode.
* `--container-name NAME`: The name of the Docker container on the Jetson. (Default: `rover_ai`)
* `--dockerfile FILE` : Path to the local Dockerfile. (Default: `docker/Dockerfile`)
* `--compose FILE` : Path to the Production Compose file. (Default: `docker/docker-compose.prod.yaml`)
* `--ssid SSID` : Wi-Fi SSID of the Jetson hotspot to automatically connect to. (Default: `ERC_UCUSpaceRobotics_A`)
* `--pass PASS` : Wi-Fi password for the Jetson hotspot. (Default: `19283746`)
* `--no-wifi, -n` : Skip the Wi-Fi auto-connect and ssh straight to the Jetson (already on the network, or connected some other way).
* `-h, --help` : Display the help message and exit.


## Deployment Strategies

The script supports five distinct deployment strategies depending on your current development needs:

### 1. Native Remote Build (`remote-build`)

* **When to use:** You modified the `Dockerfile`, system dependencies, or want to test unpushed system-level changes safely.
* **What it does:** Syncs your entire local repository to the Jetson, and natively builds the ARM64 image directly on the rover's hardware using Compose. This completely bypasses all local emulator bugs.
* **Prerequisites:** The Jetson must have internet access. It must either be connected to a router via Ethernet cable, or your laptop must be connected to the Jetson via Ethernet with internet connection sharing/forwarding enabled.

### 2. Pull & Bridge (`pull`)

* **When to use:** You want to move `rover_ai` onto a newer (team) image, e.g. after new system dependencies.
* **What it does:** Uses your laptop's internet to pull the ARM64 image from GHCR, exports it to an archive, transfers it with the compose file to the Jetson, loads it, records it in `.env`, restarts `rover_ai` on it and rebuilds our `src/` there. Our code on the Jetson is not replaced.
* **Pro-Tip: Offload Builds to GitHub Actions**
You can build your images in the cloud instead of locally by utilizing GitHub Actions. First, push your branch to GitHub, navigate to the **Actions** tab, and select the **Publish Production And Development Images** workflow on the left. Click the **Run workflow** dropdown, choose your branch, and click the green button to trigger the cloud build. Once the build is successfully finished, you can deploy the new image to the Jetson using the script's pull mode. For example, use `--tag <branch-name>-prod` (ensuring any slashes in your branch name are replaced with dashes, like `--tag feature-shared-some-feature-prod`) to deploy that image.

### 3. Rapid Source Sync (`sync-src`)

* **When to use:** You only modified code (Python, C++, launch files) and did *not* change the Docker configuration or the dependencies in `package.xml` files. Ideal for iterative, day-to-day testing of code logic.
* **What it does:** Bypasses Docker builds. It transfers your source code (and the compose file), starts the container if it is not running yet, and triggers a `colcon build --symlink-install` inside it. Fastest mode, and the first-time setup.

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

> Note: Your laptop requires additional setup before you can use Ethernet mode. To do this, refer to the section **Laptop Setup** in the [ssh.md](./networking/ssh.md)