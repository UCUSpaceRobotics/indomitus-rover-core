# Rover Usage

This document provides fast introduction of how to start the rover

## Starting the Rover

Follow these steps to power on and operate the rover:

1. **Power on the rover:** Rotate the blue switch on the back of the rover clockwise, then press the blue button. When the rover is powered on, the CAN interface is automatically set up, the Wi-Fi hotspot is turned on, and the Docker container starts.

2. **Connect to the hotspot:** Connect your computer to the Jetson's Wi-Fi hotspot (`IndomitusRover`) using the password `12345678`

3. **Enter the container on the rover:** Use the following command to enter the container's terminal on the rover:
   ```bash
   ./scripts/enter_container rover
   ```

4. **Launch the rover:** Run the following command inside the rover's container to start all the necessary rover control nodes:
   ```bash
   ros2 launch rover_bringup rover.launch.py
   ```

5. **Pull docker image for the laptop from the GitHub:** Run the following command to pull the image from the GitHub to the laptop:
   ```bash
   docker pull ghcr.io/ucuspacerobotics/indomitus-rover-core:develop-dev
   ```

6. **Copy docker compose file:** Run the following command to copy example docker compose file to the root of the repository:
   ```bash
   cp ./docker/docker-compose.dev.example.yaml ./docker-compose.yaml
   ```

7. **Start the container:** Run the following command to start the container on the laptop:
   ```bash
   IMAGE_TAG=develop-dev ROS_DOMAIN_ID=132 docker compose up -d
   ```

8. **Enter container terminal:** Run the following command to enter container terminal on the laptop:
   ```bash
   docker exec -it rover_dev /bin/bash
   ```

9. **Build and source the workspace:** Run the following command to build and source the workspace.
   ```bash
   colcon build --packages-ignore-regex ".*_sim|.*_viz"
   source install/setup.bash
   ```

10. **Connect the joystick:** Pair the joystick with your laptop and connect it.

11. **Verify the joystick connection:** Wait for the joystick LEDs to stop blinking and turn solid white, indicating a successful connection.

12. **Start the joystick nodes:** Run the following command inside the laptop's container to start the nodes responsible for handling joystick input.
    ```bash
    ros2 launch rover_teleop joy.launch.py
    ```

13. **Control the rover:** You are now ready to drive the rover using the joystick.


## Turning Off the Rover

To power down the rover, perform **one** of the following actions:

* **Use the power switch:** Rotate the blue switch on the back of the rover counterclockwise.
* ~~**Use the Emergency Stop:** Press the red button on the top of the rover.~~