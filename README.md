# ERC-UCU
Mars Rover project by UCU Space Robotics Team for ERC competitions

## Table of Contents

- [ERC-UCU](#erc-ucu)
  - [Table of Contents](#table-of-contents)
  - [Getting Started](#getting-started)
  - [Documentation](#documentation)
  - [Rover Usage](#rover-usage)
    - [Starting the Jetson-Controlled Rover](#starting-the-jetson-controlled-rover)
    - [Starting the Laptop-Controlled Rover](#starting-the-laptop-controlled-rover)
    - [Turning Off the Rover](#turning-off-the-rover)
    - [Current System Credentials and Network Info](#current-system-credentials-and-network-info)
    - [SSH Access to the Jetson](#ssh-access-to-the-jetson)
      - [SSH via Hotspot](#ssh-via-hotspot)
      - [SSH via Ethernet](#ssh-via-ethernet)
  - [ROS\_DOMAIN\_ID](#ros_domain_id)
    - [Managing ROS\_DOMAIN\_ID](#managing-ros_domain_id)
  - [Docker](#docker)
    - [Getting the Docker Image](#getting-the-docker-image)
    - [Start the Container and Build the Workspace](#start-the-container-and-build-the-workspace)
    - [Image Tags](#image-tags)
  - [Scripts](#scripts)
    - [Deployment Script](#deployment-script)
    - [Script to Enter Containers](#script-to-enter-containers)
  - [Testing](#testing)
  - [Arm Usage](#arm-usage)
    - [Starting the Arm in Simulation (Laptop)](#starting-the-arm-in-simulation-laptop)
      - [Standalone Visualization](#standalone-visualization)
      - [MoveIt Planning Simulation](#moveit-planning-simulation)
      - [Fake Hardware vs. Real Hardware](#fake-hardware-vs-real-hardware)

## Getting Started
1. Read [README](README.md)
2. Set up [Docker](docker/README.md)
3. Read [CONTRIBUTING](https://github.com/UCUSpaceRobotics/.github/blob/main/.github/CONTRIBUTING.md) before making any changes

## Documentation
This file provide all neccessary documentation for basic work with the rover. For more details you may refer to the other docs

- [Hardware](docs/hardware/can_bus.md) — CAN bus wiring
- [Networking](docs/networking/ssh.md) — SSH connection
- [Software](docs/software/) — launch, nodes, testing, gazebo
- [Testing](docs/software/testing.md) — code testing and tests creation
- [Scripts](docs/scripts/) — scripts automatinng workflows
- [Deployment](docs/deployment.md) — code deployment to the rover
- [CAD](docs/CAD/README.md) — CAD files naming conventions


## Rover Usage

This section describes how to start the rover when it is controlled by the Jetson and the laptop.

### Starting the Jetson-Controlled Rover

Follow these steps to power on and operate the rover:

1. **Power on the rover:** Rotate the blue switch on the back of the rover clockwise, then press the blue button. When the rover is powered on, the CAN interface is automatically set up, the Wi-Fi hotspot is turned on, and the Docker container starts.

2. **Connect to the hotspot:** Connect to the rover's Wi-Fi hotspot. The credentials can be found in the [Current System Credentials and Network Info](#current-system-credentials-and-network-info) section.

3. **Enter the container on the rover:** Use the following command to enter the container's terminal on the rover:
   ```bash
   ./scripts/enter_container rover
   ```

4. **Launch the rover:** Run the following command inside the rover's container to start all the necessary rover control nodes:
   ```bash
   ros2 launch rover_bringup rover.launch.py
   ```

5. **Get the image and start the container on the laptop:** Please refer to the [Docker](#docker) section to complete this step.

6.  **Connect the joystick:** Pair the joystick with your laptop and connect it.

7.  **Verify the joystick connection:** Wait for the joystick LEDs to stop blinking and turn solid white, indicating a successful connection.

8.  **Start the joystick nodes:** Run the following command inside the laptop's container to start the nodes responsible for handling joystick input.
    ```bash
    ros2 launch rover_teleop joy.launch.py
    ```

9.  **Control the rover:** You are now ready to drive the rover using the joystick.

The steps below are currently irrelevant:

1. ~~**Power on the rover:** Rotate the blue switch on the back of the rover clockwise, then press the blue button.~~
2. ~~**Turn on the joystick:** Power on the red joystick. It will automatically connect to the rover's computer.~~
3. ~~**Verify the joystick connection:** Wait for the joystick LEDs to stop blinking and turn solid white, indicating a successful connection.~~
4. ~~**Control the rover:** The launch file that initializes all the required nodes will run automatically. No further action is needed, and you are now ready to drive the rover using the joystick.~~

### Starting the Laptop-Controlled Rover

Follow these steps to power on and operate the rover using a laptop:

1. **Power on the rover:** Rotate the blue switch on the back of the rover clockwise, then press the blue button.

2. **Set up the CAN-to-USB adapter:** Connect the CAN-to-USB adapter to your laptop, then run the following commands in your terminal:

   Verify the CAN interface is visible to the laptop:
   ```bash
   ip link show can0
   ```
   *The output should display the `can0` interface details and include the state `DOWN`.*

   Bring the CAN interface up:
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   sudo ip link set can0 txqueuelen 1000
   ```

   Verify the interface is up:
   ```bash
   ip link show can0
   ```
   *The output should include the state `UP LOWER_UP`, and the blue LED on the adapter should light up.*

   > **Note:** If you have multiple CAN adapters connected, you may need to replace `can0` in these commands with the correct interface name (e.g., `can1`).

3. **Get the image and start the container on the laptop:** Please refer to the [Docker](#docker) section to complete this step.

4. **Launch the rover:** Run the following command inside the laptop's container to start all the necessary rover control nodes.
   ```bash
   ros2 launch rover_bringup rover.launch.py
   ```

5. **Connect the joystick:** Pair the joystick with your laptop and connect it.

6. **Verify the joystick connection:** Wait for the joystick LEDs to stop blinking and turn solid white, indicating a successful connection.

7. **Start the joystick nodes:** Run the following command inside the laptop's container to start the nodes responsible for handling joystick input.
    ```bash
    ros2 launch rover_teleop joy.launch.py
    ```

8.  **Control the rover:** You are now ready to drive the rover using the joystick.


### Turning Off the Rover

To power down the rover, perform **one** of the following actions:

* **Use the power switch:** Rotate the blue switch on the back of the rover counterclockwise.
* ~~**Use the Emergency Stop:** Press the **left** red button on the top of the rover.~~


### Current System Credentials and Network Info

| Property | Value |
| --- | --- |
| **Jetson Username** | `indomitus-rover` |
| **Jetson Password** | `1` |
| **Wi-Fi Hotspot Name (SSID)** | `IndomitusRover` |
| **Wi-Fi Password** | `12345678` |
| **Jetson Static IP for Hotspot** | `10.42.0.1` |
| **Jetson Container ROS_DOMAIN_ID** | `132` |


### SSH Access to the Jetson

You can SSH into the Jetson to access its bash shell for debugging or configuration. You can use either the Jetson's hotspot or an Ethernet cable.

> **Note:** If you are prompted with a security fingerprint warning, type `yes` to continue. When asked for the password, enter `1`.

#### SSH via Hotspot

1. **Connect to the network:** Connect your computer to the Jetson's Wi-Fi hotspot. The credentials can be found in the [Current System Credentials and Network Info](#current-system-credentials-and-network-info) section.

2. **Initiate the connection:** Open your terminal and run one of the following commands:

   Either use ip addres:
   ```bash
   ssh indomitus-rover@10.42.0.1
   ```

   or automatic ip resolution:
   ```bash
   ssh indomitus-rover@indomitus-rover-computer.local
   ```

#### SSH via Ethernet

1. **Prerequisites:** Ensure the prerequisites are satisfied. Refer to the **Laptop Setup** and **Jetson Setup** sections in [ssh.md](docs/networking/ssh.md).
   
2. Connect the Jetson to the laptop with an Ethernet cable, using the `ETH0` port on the Jetson.

3. Click the Network icon in your laptop's taskbar and select **Jetson Tether** profile.

4. **Initiate the connection:** Open your terminal and run the following command:
   ```bash
   ssh indomitus-rover@indomitus-rover-computer.local
   ```

> **Note:** For further details on the network configuration, refer to [ssh.md](docs/networking/ssh.md).

> **Pro Tip (ROS2 Debugging):** Because our architecture utilizes ROS2 networking, you do not always need to SSH into the Jetson to debug. As long as you are connected to the Jetson's hotspot, you can simply open your local Docker container and use standard ROS2 commands (e.g., `ros2 node list`, `ros2 topic echo /topic_name`) to see what is happening on the rover directly from your laptop. For this to work you need to ensure that `ROS_DOMAIN_ID` in the rover and your laptop's container match, refer to [Ros Domain ID](#ros-domain-id) section.


---


## ROS_DOMAIN_ID

The **`ROS_DOMAIN_ID`** is a number that acts like a private radio channel for your ROS 2 network. It isolates different ROS 2 applications running on the same physical network, ensuring that only nodes sharing the exact same ID can discover and communicate with each other.

Because this ID is an environment variable that nodes read only at creation, it cannot be changed while a node is running. You must restart the node to apply a new ID.

* **Connecting to the Rover:** To interact with or view the topics and nodes running on the rover from your laptop, your local container must use the same `ROS_DOMAIN_ID` as the rover's container.
* **Avoiding Network Interference:** If multiple developers are working independently on the same Wi-Fi network using the default or identical IDs, their data will mix together and cause system issues. To prevent this, each developer must choose a **unique** `ROS_DOMAIN_ID`.

> **Note:** The current `ROS_DOMAIN_ID` assigned to the rover container can be found in the [Current System Credentials and Network Info](#current-system-credentials-and-network-info) section.

### Managing ROS_DOMAIN_ID

**To check your current ID:**
Run the following command in your terminal:
```bash
echo $ROS_DOMAIN_ID
```

**To change the ID and restart your container:**

1. Stop the currently running container:
   ```bash
   docker compose stop
   ```

2. Start the container with the new ID. You can pass the variable directly into the start command:
   ```bash
   ROS_DOMAIN_ID=132 docker compose up -d
   ```

*(Alternatively, you can permanently change the `ROS_DOMAIN_ID` value inside your `docker-compose.yml` file before running the standard `docker compose up -d` command).*

---


## Docker

### Getting the Docker Image

Before doing anything, you need to prepare your local environment:

1. Navigate to the root of the **indomitus-rover-core** repository.

2. Copy the example Docker Compose file:

   ```bash
   cp ./docker/docker-compose.dev.example.yaml ./docker-compose.yaml
   ```

> In most cases, the example Docker Compose file should work for you, but you may still need to modify it to match your machine.

Next, you need the Docker image. You can either pull a pre-built image or build it locally.

**Option A: Pull the image from GitHub**

*This may save significant time.*

For example, pull the image from `develop`:

```bash
docker pull ghcr.io/ucuspacerobotics/indomitus-rover-core:develop-dev
```

> **Pro Tip:** The images for the `develop` and `main` branches are built automatically, but you can also build an image for any branch. To do so, open the [GitHub Actions page](https://github.com/UCUSpaceRobotics/indomitus-rover-core/actions/workflows/publish_image.yaml). Press the gray button on the right, select the branch for which you want to build an image, and click **Run workflow**. When the image is built, you can pull it with the command `docker pull ghcr.io/ucuspacerobotics/indomitus-rover-core:<branch-name>-dev` (ensuring any slashes in your branch name are replaced with dashes, like `feature-shared-some-feature-dev`).

**Option B: Build the image locally**

```bash
docker compose build
```

> **Note:** All containers mount your local `src/` directory, which means your code will be used inside the container. Ensure that your `src/` directory contains the code you want to test.

### Start the Container and Build the Workspace

1. Start the container in the background:

   ```bash
   docker compose up -d
   ```

> **Important:** By default, this starts the container using the image tag `local-dev` (defined in `docker-compose.yaml`). If you pulled the image from GitHub, it will have a different tag (refer to the [Image Tags](#image-tags) section). To start the container using a different image tag, you need to set the `IMAGE_TAG` environment variable. You can do this either in the `docker-compose.yaml` file or directly in the command:
> ```bash
> IMAGE_TAG=feature-shared-some-feature-dev docker compose up -d
> ```
> If you omit this, Docker will either build a new image (if there is no image tagged `local-dev`), or it will start the container using the existing `local-dev` image, which may have different dependencies installed.

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

> **Note: Simulation Package Build Errors** At this stage, you might encounter build errors related to the `rover_viz` or `rover_sim` packages. This typically occurs because your current Docker container was built without the necessary visualization and simulation dependencies (controlled by the `INSTALL_SIM_TOOLS` variable in `docker-compose.yaml`).

Depending on your requirements, choose one of the following solutions:

**Option A: Skip the packages (If you do not need simulations)**
You can instruct `colcon` to ignore the failing packages and safely build the rest of the workspace:
```bash
colcon build --symlink-install --packages-ignore rover_viz rover_sim arm_viz arm_sim
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
* **`<branch-name>-dev`** and **`<branch-name>-prod`**: Images built by GitHub workflows when you manually trigger **Publish Production and Development Images** for your branch (slashes in the branch name are replaced with dashes).
* **`latest-prod`**: Production image build on merge to main branch 

---


## Scripts

The repository contains utility scripts to simplify common workflow tasks. All scripts are located in the `scripts/` directory.


### Deployment Script

Use the dedicated deployment script to transfer your codebase and Docker environment to the Jetson on the rover.

To pull the latest image built by the CI pipeline on the `develop` branch, and transfer it along with the `src/` directory and `docker-compose.prod.yaml` file pulled from the GitHub to the Jetson:

```bash
./scripts/deploy_to_rover.sh pull
```

The images for the `develop` and `main` branches are built automatically, but you can also use GitHub Actions to build an image for any other branch. To do so, open the [GitHub Actions page](https://github.com/UCUSpaceRobotics/indomitus-rover-core/actions/workflows/publish_image.yaml). Press the gray button on the right, select the branch for which you want to build an image, and click **Run workflow**. When the image is built, you can deploy it with the following command.

```bash
./scripts/deploy_to_rover.sh pull --tag <branch-name>-prod
```

Replace the branch name with your branch name, with slashes replaced by dashes.

**Building the image on GitHub and then deploying it with the script is the recommended deployment method.**

> **Note:** For further details on available deploy modes, refer to [deployment.md](docs/deployment.md). For full documentation refer to [deploy_to_rover.md](docs/scripts/deploy_to_rover.md).


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

> **Note:** For further details, refer to [enter_container.md](docs/scripts/enter_container.md).


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

> **Note:** For further details on how to run tests locally or write your own test suites, refer to [testing.md](docs/software/testing.md).


---


## Arm Usage

> **Note:** Arm packages are located in `src/arm/`. To build only the arm subsystem:
> ```bash
> colcon build --symlink-install --packages-select-regex "^arm_"
> ```

### Starting the Arm in Simulation (Laptop)

There are two simulation modes available locally. Choose based on what you need:

| Mode | Launch file | Use case |
|---|---|---|
| Standalone visualization | `arm_bringup/arm_standalone.launch.py` | Quick URDF/mesh checks, manual joint testing via GUI, no planning needed |
| MoveIt planning simulation | `arm_moveit_config/demo.launch.py` | Testing trajectories, kinematics, motion planning, task development |

#### Standalone Visualization

Starts RViz with the Joint State Publisher GUI but no motion planning stack. Useful for quickly
inspecting the URDF model, meshes, and TF tree without loading ros2_control or the full MoveIt overhead.

1. **Allow GUI access:** Run the following command on your **host machine** terminal (not inside Docker) before launching:
   ```bash
   xhost +local:docker
   ```
2. **Build and start the container:** Please refer to the [Docker setup section](#docker) to complete this step.
3. **Run the standalone launch file:** From the bash terminal inside your Docker container:
   ```bash
   ros2 launch arm_bringup arm_standalone.launch.py
   ```

By default this runs with `use_fake_hardware:=true`, which loads `mock_components/GenericSystem`
(see [Fake Hardware vs. Real Hardware](#fake-hardware-vs-real-hardware) below). To attempt loading
the real CAN hardware interface instead:
```bash
ros2 launch arm_bringup arm_standalone.launch.py use_fake_hardware:=false
```

#### MoveIt Planning Simulation

Starts the full MoveIt 2 stack with motion planning, collision checking, and trajectory
execution using Fake Hardware. This is the primary mode for developing and testing arm
movements locally.

1. **Allow GUI access:** Run the following command on your **host machine** terminal (not inside Docker) before launching:
   ```bash
   xhost +local:docker
   ```
2. **Build and start the container:** Please refer to the [Docker setup section](#docker) to complete this step.
3. **Run the MoveIt demo launch file:** From the bash terminal inside your Docker container:
   ```bash
   ros2 launch arm_moveit_config demo.launch.py
   ```

In RViz, use the **MotionPlanning** panel to set a goal pose for the end-effector and click
**Plan & Execute** to run a full plan-and-execute cycle.

#### Fake Hardware vs. Real Hardware

The `arm_macro.xacro` model exposes a `use_fake_hardware` xacro argument that controls which
`ros2_control` hardware plugin gets loaded:

| Value | Plugin | Behavior |
|---|---|---|
| `true` (default) | `mock_components/GenericSystem` | Joint commands are written directly into the joint state and read back immediately — no physics, no motor, no delay. Useful for testing planning logic, SRDF groups, and the MoveIt API without any physical or simulated dynamics. |
| `false` | `arm_hardware_interface/ArmCanSystem` | Sends commands over the real CAN bus to the physical actuators. Requires the Jetson and a working `arm_hardware_interface` build. |

Because `mock_components/GenericSystem` reports back whatever position it was just told to move
to, it does **not** validate motor dynamics, CAN latency, encoder noise, or mechanical limits like
backlash or sag — only the kinematic/geometric correctness of a trajectory is verified.
