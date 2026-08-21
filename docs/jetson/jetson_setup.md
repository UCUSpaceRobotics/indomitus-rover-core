## Installation of the Drivers for the Alfa AWUS036ACH (RTL8812AU)

1. **Verify physical connection:**
Ensure the system detects the adapter physically.

    ```bash
    lsusb
    ```

    *Look for: `Realtek Semiconductor Corp. RTL8812AU`*

2. **Install build dependencies:**

    ```bash
    sudo apt update && sudo apt install -y dkms git build-essential
    ```

3. **Clone the driver source code:**
    Download the community-optimized driver.

    ```bash
    git clone https://github.com/aircrack-ng/rtl8812au.git && cd rtl8812au
    ```

4. **Fix the Jetson kernel header symlink:** Fixes DKMS missing headers error.
    Nvidia placed the kernel source code in a nested `3rdparty` directory but failed to link it properly. Run these commands to fix the path so DKMS can find it:

    ```bash
    sudo rm -f /lib/modules/$(uname -r)/build \
    && sudo ln -s /usr/src/linux-headers-$(uname -r)-ubuntu22.04_aarch64/3rdparty/canonical/linux-jammy/kernel-source /lib/modules/$(uname -r)/build
    ```

5. **Compile and install via DKMS:**
Because you fixed the symlink first, the standard DKMS installation will now compile the driver smoothly for the ARM64 architecture.

    ```bash
    sudo make dkms_install
    ```

6. **Activate the driver:**
Load the newly compiled module into the kernel (note the `XX` in the module name).

    ```bash
    sudo modprobe 88XXau
    ```

7. **Verify success:** Confirm the driver has loaded and created a network interface.

    ```bash
    ip link show
    ```

    *You should see a new interface starting with `wlx` followed by the adapter's MAC address (e.g., `wlx00c0caba86c1`). The blue LED on the Alfa adapter should now be active.*