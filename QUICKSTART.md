# Indomitus Rover Usage

This document provides a quick introduction on how to start the rover. For further details, refer to the [README](README.md).

## Starting the Rover

Follow these steps to power on and operate the rover:

> **Note:** All commands below need to be run from the root of the `indomitus-rover-core` repository.

1. **Power on the rover:** Rotate the blue switch on the back of the rover clockwise, then press the blue button. When the rover is powered on, the CAN interface is automatically set up, the Docker container starts, and the Wi-Fi hotspot is turned on (**note that it takes ~1 minute for the network to appear**).

2. **Connect to the hotspot:** Connect your computer to the Jetson's Wi-Fi hotspot (`IndomitusRover`) using the password `12345678`.

3. **Enter the container on the rover:** Use the following command to enter the container's terminal on the rover:
   ```bash
   ./scripts/enter_container.sh rover
   ```

4. **Launch the rover:** Run the following command inside the rover's container to start all the necessary rover control nodes:
   ```bash
   ros2 launch rover_bringup rover.launch.py
   ```

   > **⚠️ Important:** Open a new terminal on your laptop for all subsequent steps, and ensure you navigate back to the root of the `indomitus-rover-core` repository.

5. **Pull the latest code:** Run the following command to switch to the `develop` branch and pull the latest changes from it:
   ```bash
   git switch develop && git pull
   ```

6. **Pull the Docker image for the laptop from GitHub:** Run the following command to pull the image from GitHub to the laptop:
   ```bash
   docker pull ghcr.io/ucuspacerobotics/indomitus-rover-core:develop-dev
   ```

   > **Note:** This step only needs to be done once. If you already have the image, you can skip it.

7. **Copy the Docker Compose file:** Run the following command to copy the example Docker Compose file to the root of the repository:
   ```bash
   cp ./docker/docker-compose.dev.example.yaml ./docker-compose.yaml
   ```

   > **Note:** This step only needs to be done once. If you have already done it, you can skip it.

8. **Start the container:** Run the following command to start the container on the laptop:
   ```bash
   IMAGE_TAG=develop-dev ROS_DOMAIN_ID=132 docker compose up -d
   ```

9. **Enter the container terminal:** Run the following command to enter the container terminal on the laptop:
   ```bash
   docker exec -it rover_dev /bin/bash
   ```

10. **Build and source the workspace:** Run the following command in the container terminal to build and source the workspace:
    ```bash
    colcon build --packages-ignore-regex ".*_sim|.*_viz" && source install/setup.bash
    ```

11. **Connect the joystick:** Pair the joystick with your laptop and connect it.

12. **Verify the joystick connection:** Wait for the joystick LEDs to stop blinking and turn solid white, indicating a successful connection.

13. **Start the joystick nodes:** Run the following command inside the laptop's container to start the nodes responsible for handling joystick input:
    ```bash
    ros2 launch rover_teleop joy.launch.py
    ```

14. **Control the rover:** You are now ready to drive the rover using the joystick.


## Turning Off the Rover

To power down the rover, perform **one** of the following actions:

* **Use the power switch:** Rotate the blue switch on the back of the rover counterclockwise.
* ~~**Use the Emergency Stop:** Press one of the two red buttons on the top of the rover.~~