# Rover Quickstart

## Table of Contents

**Rover Usage**
- [Rover Quickstart](#rover-quickstart)
  - [Table of Contents](#table-of-contents)
  - [Rover Usage](#rover-usage)
    - [Starting the Jetson-Controlled Rover](#starting-the-jetson-controlled-rover)
    - [Starting the Laptop-Controlled Rover](#starting-the-laptop-controlled-rover)
    - [Turning Off the Rover](#turning-off-the-rover)
    - [Current System Credentials and Network Info](#current-system-credentials-and-network-info)
    - [SSH Access to the Jetson](#ssh-access-to-the-jetson)
  - [Docker](#docker)
    - [Getting the Docker Image](#getting-the-docker-image)
    - [Start the Container and Build the Workspace](#start-the-container-and-build-the-workspace)
    - [Image Tags](#image-tags)
  - [Scripts](#scripts)
    - [Deployment Script](#deployment-script)
    - [Script to Enter Containers](#script-to-enter-containers)
  - [Testing](#testing)

## Rover Usage

### Starting the Jetson-Controlled Rover

Follow these steps to power on and operate the rover:

1. **Power on the rover:** Rotate the blue switch on the back of the rover clockwise, then press the blue button.
2. **Turn on the joystick:** Power on the red joystick. It will automatically connect to the rover's computer.
3. **Verify the connection:** Wait for the LED on the joystick to stop blinking and turn solid white, indicating a successful connection.
4. **Control the rover:** The launch file that initializes all the required nodes will run automatically. No further action is needed, and you are now ready to drive the rover using the joystick.


### Starting the Laptop-Controlled Rover

Follow these steps to power on and operate the rover using a laptop:

1. **Power on the rover:** Rotate the blue switch on the back of the rover clockwise, then press the blue button.

2. **Set up the CAN-to-USB adapter:** Connect the CAN-to-USB adapter to your laptop, then run the following commands in your terminal:

Verify the CAN interface is visible to the laptop:
```bash
ip link show can0
```
*The output should display the can0 interface details and include the state DOWN.*

Bring the CAN interface up:
```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
```

Verify the interface is up:
```bash
ip link show can0
```
*The output should include the state `UP LOWER_UP` and blue LED on the adapter should light up.*

> **Note:** If you have multiple CAN adapters connected, you may need to replace `can0` in these commands with the correct interface name (e.g., `can1`).

3. **Build and start the container:** Please refer to the [Docker setup section](#docker) to complete this step.

4. **Run the rover launch file:** From the bash terminal inside your Docker container, run the launch file to start all the necessary rover control nodes:
```bash
ros2 launch rover_bringup rover.launch.py
```

5. **Connect the joystick:** Pair the joystick with your laptop and connect it.

6. **Verify the joystick connection:** Wait for the LED on the joystick to stop blinking and turn solid white, indicating a successful connection.

7. **Start the joystick nodes:** Run the following command to start the nodes responsible for handling joystick input:
```bash
ros2 launch rover_bringup joy.launch.py
```

8. **Control the rover:** You are now ready to drive the rover using the joystick.


### Turning Off the Rover

To power down the rover, perform **one** of the following actions:

* **Use the power switch:** Rotate the blue switch on the back of the rover counterclockwise.
* **Use the Emergency Stop:** Press the **left** red button on the top of the rover.

> **Attention:** Currently, only the **left** red button on top of the rover is connected. The right red button is inactive and will not stop the rover. Always use the left button for an emergency stop.


### Current System Credentials and Network Info

| Property | Value |
| --- | --- |
| **Jetson Username** | `indomitus-rover` |
| **Jetson Password** | `1` |
| **Wi-Fi Hotspot Name (SSID)** | `IndomitusRover` |
| **Wi-Fi Password** | `12345678` |
| **Jetson Static IP** | `10.42.0.1` |


### SSH Access to the Jetson

You can SSH into the Jetson to access its bash shell for debugging or configuration.

1. **Connect to the network:** Connect your computer to the Jetson's Wi-Fi hotspot (`IndomitusRover`) using the password `12345678`.
2. **Initiate the connection:** Open your terminal and run the following command:
```bash
ssh indomitus-rover@10.42.0.1
```

1. **Authenticate:** If prompted with a security fingerprint warning, type `yes` to continue. When asked for the password, enter `1`.

> **Note:** For further details on the network configuration, refer to [hotspot.md](./networking/hotspot.md).

> **Pro Tip (ROS2 Debugging):** Because our architecture utilizes ROS2 networking, you do not always need to SSH into the Jetson to debug. As long as you are connected to the Jetson's hotspot, you can simply open your local Docker container and use standard ROS2 commands (e.g., `ros2 node list`, `ros2 topic echo /topic_name`) to see what is happening on the rover directly from your laptop.


---


## Docker

### Getting the Docker Image

Before doing anything, you need to prepare your local environment:

1. Navigate to the root of the **indomitus-rover-core** repository.
2. Copy the example Docker Compose file:

```bash
cp ./docker/docker-compose.dev.example.yaml ./docker-compose.yaml
```

> In most cases example docker compose file should be suitable for you but still there may be cases where you will need to modify it to be compatible with you machine.

Next, you need the Docker image. You can either pull a pre-built image or build it locally.

**Option A: Pull the image from GitHub**
*This saves significant time.* Pull the image corresponding to your target branch:

From `develop`:

```bash
docker pull ghcr.io/ucuspacerobotics/indomitus-rover-core:develop-dev
```

From `main`:

```bash
docker pull ghcr.io/ucuspacerobotics/indomitus-rover-core:main-dev
```

**Option B: Build the image locally**

```bash
docker compose build
```

> **Note:** The all containers mount your local `src/` directory. Ensure you are on the correct branch locally and have pulled the latest changes before starting the container.

### Start the Container and Build the Workspace

1. Start the container in the background:

```bash
docker compose up -d
```

2. Enter the running container:

```bash
docker exec -it rover_dev /bin/bash
```

3. Build the ROS 2 workspace:

```bash
cd /opt/ws
colcon build --symlink-install
source install/setup.bash
```

> **Note: Simulation Package Build Errors**
At this stage, you might encounter build errors related to the `rover_viz` or `rover_sim` packages. This typically occurs because your current Docker container was built without the necessary visualization and simulation dependencies (controlled by the `INSTALL_SIM_TOOLS` variable in `docker-compose.yaml`).

Depending on your requirements, choose one of the following solutions:

**Option A: Skip the packages (If you do not need simulations)**
You can instruct `colcon` to ignore the failing packages and safely build the rest of the workspace:
```bash
colcon build --symlink-install --packages-ignore rover_viz rover_sim
source install/setup.bash
```

**Option B: Fix the dependencies (If you need simulations)**
You must either manually install the missing dependencies inside your running Docker container or rebuild the entire Docker image with the simulation tools enabled.


### Image Tags

Understanding the tag naming convention will help you easily identify the correct Docker image for your hardware and environment.

The `Dockerfile` is split into two primary targets: **`dev`** and **`prod`**. The `prod` target strips out heavy dependencies used for simulations and visualizations (e.g., Gazebo, RViz) to keep the image lightweight for the rover.

Images built locally or by GitHub workflows will use the following tags:

* **`local-prod`**: Production image built locally.
* **`develop-dev`** (or `main-dev`): Development image built continuously by GitHub workflows.
  * *Architecture:* **AMD64**
  * *Use Case:* Can be used on standard Intel/AMD laptops for development and simulation. Cannot run on the Jetson.
* **`develop-prod`** (or `main-prod`): Production image built continuously by GitHub workflows.
  * *Architecture:* **ARM64**
  * *Use Case:* Designed strictly for deployment on the NVIDIA Jetson. Cannot run natively on standard Intel/AMD laptops.


---


## Scripts

The repository contains utility scripts to simplify common workflow tasks. All scripts are located in the `scripts/` directory.


### Deployment Script

Use the dedicated deployment script to transfer your codebase and Docker environment to the Jetson on the rover.

To build the image locally and transfer it, along with your local `src/` directory and `docker-compose.prod.yaml` file, to the Jetson:
```bash
./scripts/deploy_to_rover.sh
```

To pull the latest image built by the CI pipeline on the `develop` branch, and transfer it along with the `src/` directory and `docker-compose.prod.yaml` file to the Jetson:
```bash
./scripts/deploy_to_rover.sh --pull
```

> **Note:** For further details on available deploy modes, refer to [deployment.md](./deployment.md). For full documentation refer to [deploy_to_rover.md](./scripts/deploy_to_rover.md).


### Script to Enter Containers

This script automates the process of opening a bash terminal inside your Docker containers, whether they are running on your computer or on the physical robot.

Enter the **local** development container:
```bash
./scripts/enter_container.sh local
```

Enter the **remote** rover container:
```bash
./scripts/enter_container.sh rover
```

> **Note:** For further details, refer to [enter_container.md](./scripts/enter_container.md).


---


## Testing

Currently, testing is fully automated via GitHub Actions.

Tests run automatically whenever you open or update a Pull Request. To merge your PR into the `develop` or `main` branches, you must ensure that all functional tests have passed successfully.

The results of each test run are compiled into an interactive dashboard. To access it, follow these steps:

1. **Open your Pull Request** in GitHub.
2. **Scroll down** to the workflow checks section at the bottom of the "Conversation" page.
3. **Locate the job** named `PR Pipeline / run-tests (pull_request)`.
4. **Click the three dots (`...`)** next to the job name, then select **View details**.
5. **Select "Summary"** from the left-hand sidebar.
6. **Scroll down** to view the complete test results and identify any specific failures.

> **Note:** For further details on how to run tests locally or write your own test suites, refer to [testing.md](./testing.md).