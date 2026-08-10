# Sensors

## RPLIDAR S2

### Description

`RPLIDAR S2` is a 2D LiDAR sensor.

It publishes laser scan data to the `/scan` (`/rplidar/scan`) topic.

### RPLIDAR node notes

The driver executable comes from the upstream Slamtec `sllidar_ros2` package, which is built in the Docker hardware workspace. The rover package keeps the rover-specific launch file and default parameter file:

```text
src/rover_sensors/launch/rplidar_s2_launch.py
src/rover_sensors/config/rplidar_s2.yaml
```

The launched executable is `sllidar_node`, but the node is named `rplidar_node` in our launch file so the YAML parameter namespace matches rover naming.

---

### Configure RPLIDAR S2 Udev Rules

Setting up udev rules ensures the LiDAR is consistently recognized at the same device path and is automatically granted the correct read/write permissions upon connection.

**1. Create the rule file:**

```bash
sudo nano /etc/udev/rules.d/99-rplidar-s2.rules
```

**2. Add the following line, then save and exit:**

```text
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar-s2"
```

**3. Apply the changes:**

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**4. Verify the setup:**

```bash
ls -l /dev/rplidar-s2
```

> **Success:** The output will display a symlink pointing to your active USB port (e.g., `lrwxrwxrwx ... /dev/rplidar-s2 -> ttyUSB0`).

---

### Run RPLIDAR S2

```bash
ros2 launch rover_sensors rplidar_s2.launch.py
```

By default, the launch file reads:

```text
share/rover_sensors/config/rplidar_s2.yaml
```

The tracked source file is:

```text
src/rover_sensors/config/rplidar_s2.yaml
```

Current rover defaults (utilizing the udev rule):

```text
serial_port: /dev/rplidar-s2
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

### RPLIDAR S2 parameters

| Parameter | Default value | Description |
| --- | --- | --- |
| `channel_type` | `serial` | Communication type. For USB connection, keep this as `serial`. |
| `serial_port` | `/dev/rplidar-s2` | Device path of the connected LiDAR (uses udev symlink). |
| `serial_baudrate` | `1000000` | Serial baudrate. For RPLIDAR S2, this is usually `1000000`. |
| `frame_id` | `laser_link` | Frame name used in the published laser scan message. |
| `inverted` | `false` | Inverts scan data direction if set to `true`. |
| `angle_compensate` | `true` | Enables angle compensation for scan data. |
| `scan_mode` | `DenseBoost` | LiDAR scan mode. |
| `scan_frequency` | `10.0` | Requested scan frequency used by the upstream driver. |
| `tcp_ip` | `192.168.0.7` | Upstream TCP mode default. Usually unused for USB S2. |
| `tcp_port` | `20108` | Upstream TCP mode default. Usually unused for USB S2. |
| `udp_ip` | `192.168.11.2` | Upstream UDP mode default. Usually unused for USB S2. |
| `udp_port` | `8089` | Upstream UDP mode default. Usually unused for USB S2. |

---

### Check if the LiDAR is working

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

### Common problems

#### Issues if Udev Rules are Not Set

If you skip the udev rule configuration in the setup steps, your operating system will dynamically assign a port (e.g., `/dev/ttyUSB0`), which frequently leads to the following failures:

* **Permission Denied:** By default, Linux restricts access to serial ports. The driver node will fail to read the device. Without the `MODE:="0666"` rule, you must either temporarily grant permissions manually (`sudo chmod a+rw /dev/ttyUSB0`) or permanently add your user to the dialout group (`sudo usermod -aG dialout $USER`) and reboot.
* **Wrong Serial Port (Dynamic Assignment):** If you plug the LiDAR into a different USB port, or if another serial device boots up first, the OS might assign the LiDAR to `/dev/ttyUSB1` instead of `ttyUSB0`. Because your `serial_port` configuration statically looks for one port, the launch will fail until you manually hunt down the new port (`ls /dev/ttyUSB*`) and update your `.yaml` config file.

#### No LiDAR connected

If no LiDAR is connected to the PC (or the symlink failed to create), the node may fail with the following error:

```text
Error, unexpected error, code: 80008004
```

This usually means that the driver cannot communicate with the device.

Check:

* LiDAR is physically connected via USB
* `ls -l /dev/rplidar-s2` shows a valid symlink
* USB cable is working

#### No scan data

Check:

```bash
ros2 topic list
ros2 topic echo /scan
```

Also check:

* LiDAR is connected
* The udev rule is properly configured and successfully triggered
* Correct baudrate is used (`1000000` for S2)
* The user or Docker container has proper pass-through permission to access the `/dev/rplidar-s2` device
* LiDAR motor is spinning physically
* Launch output does not show upstream driver errors