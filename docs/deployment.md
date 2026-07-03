# Quick Deployment Guide

To deploy code to the rover computer, ensure your laptop is connected to Wi-Fi with internet access, the Jetson is turned on, and either its hotspot is active or you are connected to it via ethernet cable (in case of the ethernet everything need to be set up according to the ...). Open a terminal and run the deployment script from the root of the repository:

```bash
./scripts/deploy_to_rover.sh [MODE] [OPTIONS]
```


## Available Deployment Modes

Choose your deployment strategy based on the type of changes you just made. You must specify exactly one mode.

**Native Remote Build (`remote-build`)**

* **When to use:** You modified the `Dockerfile`, system dependencies, or want to test unpushed system-level changes safely.
* **What it does:** Syncs your entire local repository to the Jetson, and natively builds the ARM64 image directly on the rover's hardware using Compose.
* **Prerequisites:** Jetson either connected to the router with ethernet cable or you are connected to Jetson with ethernet cable and set up internet forwarding over ethernet connection (refer to ...)

**Pull & Bridge (`pull`)**

* **When to use:** You want to deploy a pre-built stable image and clean code directly from GitHub (develop/main branch). Or you have manually activated workflow on your branch and want to deploy it for testing.
* **What it does:** Pulls an image from GHCR, extracts the exact commit SHA from the image metadata, clones a clean codebase from GitHub, transfers everything over the hotspot, and restarts the container.
* **Pro-Tip: Offload Builds to GitHub Actions** You can build your images in the cloud instead of locally by utilizing GitHub Actions. First, push your branch to GitHub, navigate to the **Actions** tab, and select the **Publish Production And Development Images** workflow on the left. Click the **Run workflow** dropdown, choose your branch, and click the green button to trigger the cloud build. Once the build is successfully finished, you can deploy the new image to the Jetson using the script's pull mode. For example, use `--tag <branch-name>-prod` (ensuring any slashes in your branch name are replaced with dashes, like `--tag feature-shared-some-feature-prod`) to deploy the image and the code for the commit on which image was built.

**Rapid Source Sync (`sync-src`)**

* **When to use:** You only modified code (Python, C++, launch files).
* **What it does:** Syncs the local `src/` folder and triggers a compile inside the *already running* container. Fastest mode.

**Infrastructure Sync (`sync-docker-compose`)**

* **When to use:** You only modified the `docker-compose.prod.yaml` file.
* **What it does:** Transfers the compose file and cleanly restarts the container infrastructure.

**Local Cross-Compile (`local-build`) — ⚠️ DEPRECATED**

* **When to use:** There is no internet acces on Jetson and you cannot pull the image from the GitHub for some reason.
* **What it does:** Cross-compiles a brand new ARM64 image on your laptop using QEMU emulators, packages it, transfers the heavy payload, and spins it up. Slowest mode.

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
> *Note: You need to do this only for the `local-build` mode.*

> For further details, read the [documentation](./scripts/deploy_to_rover.md) for the deployment script.


## Ethernet Connection

For faster file transfers, we recommend connecting your laptop to the Jetson via Ethernet (if possible). To use this method, connect the cable and append the `--eth` flag when running the script.

> Note: Your laptop requires additional setup before you can use Ethernet mode. To do this, refer to the...