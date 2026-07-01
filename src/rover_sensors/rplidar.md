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

The driver executable comes from the upstream Slamtec `sllidar_ros2` package, which is built in the Docker hardware workspace. The rover package keeps the rover-specific launch file and default parameter file:

```text
src/rover_sensors/launch/rplidar_s2_launch.py
src/rover_sensors/config/rplidar_s2.yaml
```

The launched executable is `sllidar_node`, but the node is named `rplidar_node` in our launch file so the YAML parameter namespace matches rover naming.

---

## Run RPLIDAR S2

### Default launch

```bash
ros2 launch rover_sensors rplidar_s2_launch.py
```

By default, the launch file reads:

```text
share/rover_sensors/config/rplidar_s2.yaml
```

The tracked source file is:

```text
src/rover_sensors/config/rplidar_s2.yaml
```

Current rover defaults:

```text
serial_port: /dev/ttyUSB0
serial_baudrate: 1000000
frame_id: laser_link
scan_mode: DenseBoost
```

---

### Change frame ID or scan mode

Edit `frame_id` or `scan_mode` in `src/rover_sensors/config/rplidar_s2.yaml`.

For `RPLIDAR S2`, the current default scan mode is:

```text
DenseBoost
```

Supported scan modes:

* `DenseBoost` — up to ~30 meters range
* `Standart` — up to ~16 meters range

Use only these modes, as they are the ones supported by the sensor and driver.

---

## RPLIDAR S2 parameters

| Parameter          | Default value     | Description                                                    |
| ------------------ | ----------------- | -------------------------------------------------------------- |
| `channel_type`     | `serial`          | Communication type. For USB connection, keep this as `serial`. |
| `serial_port`      | `/dev/ttyUSB0`    | Device path of the connected LiDAR.                            |
| `serial_baudrate`  | `1000000`         | Serial baudrate. For RPLIDAR S2, this is usually `1000000`.    |
| `frame_id`         | `laser_link`      | Frame name used in the published laser scan message.           |
| `inverted`         | `false`           | Inverts scan data direction if set to `true`.                  |
| `angle_compensate` | `true`            | Enables angle compensation for scan data.                      |
| `scan_mode`        | `DenseBoost`      | LiDAR scan mode.                                               |
| `scan_frequency`   | `10.0`            | Requested scan frequency used by the upstream driver.          |
| `tcp_ip`           | `192.168.0.7`     | Upstream TCP mode default. Usually unused for USB S2.          |
| `tcp_port`         | `20108`           | Upstream TCP mode default. Usually unused for USB S2.          |
| `udp_ip`           | `192.168.11.2`    | Upstream UDP mode default. Usually unused for USB S2.          |
| `udp_port`         | `8089`            | Upstream UDP mode default. Usually unused for USB S2.          |

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

# Setting up udev rules

```bash
sudo nano /etc/udev/rules.d/99-rplidar-s2.rules
```

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar-s2"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

4. Verify the Connection

Check if the symlink was created successfully by running:

```bash
ls -l /dev/rplidar-s2
```

You should see an output showing that /dev/rplidar-s2 points to your actual USB port (e.g., -> ttyUSB0). The permissions string on the left should look like lrwxrwxrwx.

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

Then update `serial_port` in `src/rover_sensors/config/rplidar_s2.yaml` or launch with another `params_file`.

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
