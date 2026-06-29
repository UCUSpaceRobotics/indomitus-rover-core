!#/usr/bin/env bash

# 1. Add the USB Video permissions
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="2b03", MODE="0666"' | sudo tee /etc/udev/rules.d/99-slabs.rules

# 2. Add the internal USB Hub permissions
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="04b4", MODE="0666"' | sudo tee -a /etc/udev/rules.d/99-slabs.rules

# 3. Add the IMU/MCU hidraw permissions (This fixes your error!)
echo 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="2b03", MODE="0666"' | sudo tee -a /etc/udev/rules.d/99-slabs.rules

sudo udevadm control --reload-rules
sudo udevadm trigger