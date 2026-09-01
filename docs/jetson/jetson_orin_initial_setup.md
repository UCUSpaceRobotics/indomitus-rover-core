# Jetson Initial Setup

**This file is made for the reComputer J4012 with Jetson Orin 16GB (flashed with JetPack 6.2 / L4T 36.4.3), setup of the other models may be similar but not identical.**


---


## IMPORTANT

> ⚠️⚠️⚠️ **CRITICAL: FREEZE SYSTEM UPDATES BEFORE PROCEEDING** ⚠️⚠️⚠️

> **Do not run `sudo apt upgrade` before locking your NVIDIA packages.**

> **The Issue:** On the reComputer J4012, the bootloader (UEFI) lives on the onboard QSPI memory chip, while the OS lives on your main drive. Running `apt upgrade` updates your OS to a newer version (e.g., L4T 36.4.7) but leaves the QSPI firmware behind (e.g., 36.4.3). This bootloader-to-OS mismatch will break the boot sequence and require a complete reflash of the device.


---


## Freezing System Packages

To prevent the issue with packages update, run the following command to place all critical NVIDIA and JetPack packages on hold:

```bash
sudo apt-mark hold cuda-toolkit-12-6 cuda-toolkit-12-6-config-common cuda-toolkit-12-config-common cuda-toolkit-config-common libcudnn9-cuda-12 libcudnn9-dev-cuda-12 libcudnn9-samples libnvvpi3 nvidia-jetpack nvidia-jetpack-dev nvidia-jetpack-runtime nvidia-l4t-3d-core nvidia-l4t-apt-source nvidia-l4t-bootloader nvidia-l4t-camera nvidia-l4t-configs nvidia-l4t-core nvidia-l4t-cuda nvidia-l4t-cuda-utils nvidia-l4t-cudadebuggingsupport nvidia-l4t-dgpu-apt-source nvidia-l4t-dgpu-config nvidia-l4t-dgpu-tools nvidia-l4t-dgpu-x11 nvidia-l4t-display-kernel nvidia-l4t-dla-compiler nvidia-l4t-factory-service nvidia-l4t-firmware nvidia-l4t-gbm nvidia-l4t-graphics-demos nvidia-l4t-gstreamer nvidia-l4t-init nvidia-l4t-initrd nvidia-l4t-jetson-io nvidia-l4t-jetson-multimedia-api nvidia-l4t-jetson-orin-nano-qspi-updater nvidia-l4t-jetsonpower-gui-tools nvidia-l4t-kernel nvidia-l4t-kernel-dtbs nvidia-l4t-kernel-headers nvidia-l4t-kernel-oot-headers nvidia-l4t-kernel-oot-modules nvidia-l4t-libwayland-client0 nvidia-l4t-libwayland-cursor0 nvidia-l4t-libwayland-egl1 nvidia-l4t-libwayland-server0 nvidia-l4t-multimedia nvidia-l4t-multimedia-utils nvidia-l4t-nvfancontrol nvidia-l4t-nvml nvidia-l4t-nvpmodel nvidia-l4t-nvpmodel-gui-tools nvidia-l4t-nvsci nvidia-l4t-oem-config nvidia-l4t-openwfd nvidia-l4t-optee nvidia-l4t-pva nvidia-l4t-tools nvidia-l4t-vulkan-sc nvidia-l4t-vulkan-sc-dev nvidia-l4t-vulkan-sc-samples nvidia-l4t-vulkan-sc-sdk nvidia-l4t-wayland nvidia-l4t-weston nvidia-l4t-x11 nvidia-l4t-xusb-firmware nvidia-tensorrt nvidia-tensorrt-dev nvidia-vpi nvidia-vpi-dev python3.10-vpi3 tensorrt tensorrt-libs vpi3-dev vpi3-python-src vpi3-samples
```

**Create an APT Preference Pin (Fail-safe):**
As an extra layer of protection against accidental upgrades, generate an APT preferences file to lock the packages dynamically. Copy and paste this entire block into your terminal to create the script, make it executable, and run it:

```bash
cat << 'EOF' > generate_pins.sh
#!/bin/bash
OUT="/tmp/nvidia-jetson-pin"
> "$OUT"

for pkg in $(apt-mark showhold); do
  ver=$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null)
  if [ -n "$ver" ]; then
    {
      echo "Package: $pkg"
      echo "Pin: version $ver"
      echo "Pin-priority: 1001"
      echo ""
    } >> "$OUT"
  fi
done

echo "Generated $OUT with $(grep -c '^Package:' "$OUT") entries"
EOF

chmod +x generate_pins.sh
./generate_pins.sh
```

Finally, apply the generated file to your system's APT configuration and clean up the script:

```bash
sudo cp /tmp/nvidia-jetson-pin /etc/apt/preferences.d/nvidia-jetson-pin && rm generate_pins.sh
```

---

## Other Setup Steps

- Setup hotspot. Refer to [hotspot.md](../networking/hotspot.md#jetson-hotspon-setup)
- Setup ssh. Refer to [ssh.md](../networking/ssh.md#jetson-setup-first-time-only)
- Disable the GUI/desktop environment on headless setups to free RAM/GPU resources:
  ```bash
  sudo systemctl set-default multi-user.target
  sudo reboot
  ```
  Revert anytime with `sudo systemctl set-default graphical.target`.