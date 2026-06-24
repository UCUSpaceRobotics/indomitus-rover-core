### Quick Deployment Guide

To deploy code to the rover computer, ensure your laptop is connected to Wi-Fi with internet access, the Jetson is turned on, and its hotspot is active. Open a terminal and run the deployment script from the root of the repository:

```bash
./scripts/deploy_to_rover.sh [OPTIONS]
```

#### Available Deployment Modes

Choose your deployment strategy based on the type of changes you just made:

**Rapid Code Sync (`-S` or `--sync`)**
* **When to use:** You modified Python scripts, C++ nodes, or launch files and did not change the Docker configuration or dependencies in the `package.xml` files.
* **What it does:** Bypasses Docker entirely. It syncs the `src/` folder to the Jetson and directly triggers a `colcon build` inside the running container. Fastest mode.


**Pull & Bridge (`-P` or `--pull`)**
* **When to use:** You want to deploy an image built by GitHub workflows with stable code from the `develop` or `main` branch.
* **What it does:** Uses your laptop's internet to pull the pre-built image from GHCR, packages it, transfers it over the hotspot, and restarts the container.


**Full Image Build (Default: no flags)**
* **When to use:** You modified dependencies or Docker configuration and want to deploy code which is not yet present on GitHub for testing.
* **What it does:** Cross-compiles a brand new ARM64 Docker image on your laptop, transfers the heavy payload to the Jetson, and spins it up. Slowest mode.



---

> For further details, read the [documentation](./scripts/deploy_to_jetson.md) for the deployment script.