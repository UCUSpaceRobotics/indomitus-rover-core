### Quick Deployment Guide

To deploy code to the rover computer, ensure your laptop is connected to Wi-Fi with internet access, the Jetson is turned on, and its hotspot is active. Open a terminal and run the deployment script from the root of the repository:

```bash
./scripts/deploy_to_rover.sh [OPTIONS]
```

#### Available Deployment Modes

Choose your deployment strategy based on the type of changes you just made:

**Rapid Source Sync (`--sync-src`)**
* **When to use:** You only modified code (Python, C++, launch files).
* **What it does:** Syncs the local `src/` folder and triggers a compile inside the *already running* container. Fastest mode.


**Infrastructure Sync (`--sync-docker-compose`)**
* **When to use:** You only modified the `docker-compose.prod.yaml` file.
* **What it does:** Transfers the compose file and cleanly restarts the container infrastructure.


**Full Sync (`--sync`)**
* **When to use:** You modified *both* your code and the compose file.
* **What it does:** Syncs code and config, restarts the container, and compiles inside the fresh environment.


**Pull & Bridge (`--pull`)**
* **When to use:** You want to deploy a pre-built stable image and clean code directly from GitHub.
* **What it does:** Pulls the image from GHCR, clones a clean codebase, transfers everything over the hotspot, and restarts the container.


**Full Image Build (Default: no flags)**
* **When to use:** You modified the `Dockerfile` or system dependencies and need to test locally unpushed changes.
* **What it does:** Cross-compiles a brand new ARM64 image on your laptop, transfers the heavy payload, and spins it up. Slowest mode.



---

> For further details, read the [documentation](https://www.google.com/search?q=./scripts/deploy_to_jetson.md) for the deployment script.