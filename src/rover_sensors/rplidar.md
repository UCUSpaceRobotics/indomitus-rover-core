# Sensors

## RPLIDAR S2

### Description

`RPLIDAR S2` is a 2D LiDAR sensor.

It publishes laser scan data to the `/scan` topic that can be used for:

* obstacle detection
* mapping
* SLAM
* navigation
* debugging the rover environment

### RPLIDAR node notes

The current `RPLIDAR S2` implementation is based on the original Slamtec/SLLIDAR repository.

For this node, the public executable and launch file were renamed to use `rplidar_*` naming:

```text
rplidar_node
rplidar_s2_launch.py
```

However, some files inside the `./sdk/` folder may still use `sllidar` naming.
This is expected because they come from the original SDK used by the node.

Do not rename SDK files unless you are ready to fully refactor and test the driver.

The node is launched using:

```bash
ros2 launch rover_sensors rplidar_s2_launch.py
```

---

## Run RPLIDAR S2

### Default launch

```bash
ros2 launch rover_sensors rplidar_s2_launch.py
```

By default, the launch file uses:

```text
serial_port:=/dev/ttyUSB0
serial_baudrate:=1000000
frame_id:=laser
scan_mode:=DenseBoost
```

---

### Run with custom serial port

Use this if the LiDAR is not connected as `/dev/ttyUSB0`.

```bash
ros2 launch rover_sensors rplidar_s2_launch.py serial_port:=/dev/ttyUSB1
```

Check connected USB devices with:

```bash
ls /dev/ttyUSB*
```

---

### Run with custom frame ID

```bash
ros2 launch rover_sensors rplidar_s2_launch.py frame_id:=lidar_link
```

Use this when the robot URDF has a different LiDAR frame name.

---

### Run with custom scan mode

```bash
ros2 launch rover_sensors rplidar_s2_launch.py scan_mode:=DenseBoost
```

For `RPLIDAR S2`, the current default is:

```text
DenseBoost
```

Supported scan modes:

* `DenseBoost` — up to ~30 meters range
* `Standart` — up to ~16 meters range

Use only these modes, as they are the ones supported by the sensor and driver.

---

## RPLIDAR S2 launch parameters

| Parameter          | Default value  | Description                                                                                 |
| ------------------ | -------------- | ------------------------------------------------------------------------------------------- |
| `channel_type`     | `serial`       | Communication type. For USB connection, keep this as `serial`.                              |
| `serial_port`      | `/dev/ttyUSB0` | Device path of the connected LiDAR.                                                         |
| `serial_baudrate`  | `1000000`      | Serial baudrate. For RPLIDAR S2, this is usually `1000000`.                                 |
| `frame_id`         | `laser`        | Frame name used in the published laser scan message.                                        |
| `inverted`         | `false`        | Inverts scan data direction if set to `true`. Usually keep `false`.                         |
| `angle_compensate` | `true`         | Enables angle compensation for scan data. Usually keep `true`.                              |
| `scan_mode`        | `DenseBoost`   | LiDAR scan mode. Supported modes are `DenseBoost` (up to ~30m) and `Standart` (up to ~16m). |

---

## Check if the LiDAR is working

After launching the node, open another terminal and run:

```bash
ros2 topic list
```

You should see a scan topic, usually:

```text
/scan
```

Check the scan data:

```bash
ros2 topic echo /scan
```

Check the message type:

```bash
ros2 topic info /scan
```

Expected message type:

```text
sensor_msgs/msg/LaserScan
```

---

## Common problems

### Permission denied for `/dev/ttyUSB0`

If the node cannot access the LiDAR device, check permissions:

```bash
ls -l /dev/ttyUSB0
```

Temporary fix:

```bash
sudo chmod a+rw /dev/ttyUSB0
```

Better fix on a normal Linux system:

```bash
sudo usermod -aG dialout $USER
```

Then log out and log in again.

In Docker, make sure the container has access to the device, for example by passing `/dev/ttyUSB0` or using proper device permissions.

---

### Wrong serial port

If `/dev/ttyUSB0` does not exist, check available ports:

```bash
ls /dev/ttyUSB*
```

Then launch with the correct one:

```bash
ros2 launch rover_sensors rplidar_s2_launch.py serial_port:=/dev/ttyUSB1
```

---

### No LiDAR connected

If no LiDAR is connected to the PC, the node may fail with the following error:

```text
Error, unexpected error, code: 80008004
```

This usually means that the driver cannot communicate with the device.

Check:

* LiDAR is physically connected via USB
* correct serial port is used
* USB cable is working
* device appears in `/dev/ttyUSB*`

---

### No scan data

Check:

```bash
ros2 topic list
ros2 topic echo /scan
```

Also check:

* LiDAR is connected
* correct serial port is used
* correct baudrate is used
* user/container has permission to access the USB device
* LiDAR motor is spinning
* launch output does not show driver errors
