# Hotspot Setup


## Prerequisites

* wifi module (Alfa AWUS036ACH, RTL8812AU chipset) installed on Jetson with the `morrownr/8812au` driver built and installed via DKMS
* Jetson creates wifi-hotspot and connects to it on boot up
* your laptop is connected to that same network (`ERC_UCUSpaceRobotics_A` with password `19283746`)

> **Important:** If at any stage you are asked for the password for the user on the Jetson, the current password is `1`

> **Competition rule note:** Per the ERC radio frequency rules, the SSID and frequency configuration below are not arbitrary — they must match specific required values:
> - SSID for the 5 GHz low band must follow the pattern `ERC_TeamName_(A/B)` — hence `ERC_UCUSpaceRobotics_A`.
> - The hotspot must run in the 5 GHz low band (5150–5725 MHz), on a channel width of 20 or 40 MHz (40 MHz max without prior organizer approval), at or below 1 W EIRP.
> - The channel itself is assigned by judges just before competition and does **not** stay fixed at channel 36 — see the note in step 6 below.
> - Any mismatch between the declared RF Form and the actual running configuration can cost up to −20 points per violation, so keep this document in sync with whatever is actually running on the Jetson.


## Jetson hotspot setup

> **Note:** Follow this only after OS reinstallation or if migrating to a new Jetson


### 1. Install the Wi-Fi adapter driver

The AWUS036ACH (RTL8812AU chipset) has no in-kernel driver on Jetson's L4T kernel and needs to be built via DKMS against matching kernel headers.

```bash
sudo apt update && sudo apt install -y build-essential dkms git bc libelf-dev
```

Jetson's headers package doesn't land at the path DKMS expects by default, so link it manually (adjust the version string to match your `uname -r`):

```bash
sudo ln -s /usr/src/linux-headers-<version>-ubuntu22.04_aarch64/3rdparty/canonical/linux-jammy/kernel-source \
            /usr/src/linux-headers-<version>
```

Confirm `Makefile`, `Module.symvers`, and `.config` all exist inside that linked path before continuing.

Build and install the driver:

```bash
mkdir -p ~/src && cd ~/src && git clone https://github.com/morrownr/8812au-20210820.git && cd 8812au-20210820 && sudo ./install-driver.sh NoPrompt && dkms status
```

`dkms status` should show the module as `installed` against your running kernel version. Unplug and replug the adapter afterward so it binds to the new driver.

### 2. Set the regulatory domain

Set this to match the country the rover is **physically operating in** - it governs which channels/power the driver will allow. For `<COUNTRY_CODE>` code write `UA` or `PL` depending on the country.

```bash
echo "options cfg80211 ieee80211_regdom=<COUNTRY_CODE>" | sudo tee /etc/modprobe.d/cfg80211.conf && sudo update-initramfs -u && sudo reboot
```

Verify after reboot:

```bash
iw reg get
```

### 3. Find the interface name

```bash
ip -br link show
```

*Look for an interface name that starts with wlx followed by a MAC address (for example, `wlx00c0caba86c1`). Copy it, you will need it for the next step.*

### 4. Create the hotspot

Create hotspot `ERC_UCUSpaceRobotics_A` with password `19283746` and force the connection name to be `Hotspot`. Replace `wlx00c0caba86c1` in the command with the copied name:

```bash
sudo nmcli dev wifi hotspot ifname wlx00c0caba86c1 con-name "Hotspot" ssid "ERC_UCUSpaceRobotics_A" password "19283746"
```

### 5. Verify the connection was created successfully

```bash
nmcli connection show
```

*You should see in the list connection named `Hotspot`*

### 6. Force 5 GHz band, set channel, disable power saving, set static IP, enable autoconnect

```bash
sudo nmcli connection modify Hotspot \
  802-11-wireless.band a \
  802-11-wireless.channel 36 \
  802-11-wireless.mode ap \
  wifi.powersave 2 \
  ipv4.addresses 10.42.0.1/24 \
  ipv4.method shared \
  connection.autoconnect yes \
&& sudo nmcli connection up Hotspot
```

> **Channel note:** Channel 36 is used here as a stable non-DFS default for testing. At competition, judges assign the actual channel during the RF check. To switch channels without rebuilding the connection:
> ```bash
> sudo nmcli connection modify Hotspot 802-11-wireless.channel <assigned_channel> && sudo nmcli connection up Hotspot
> ```
> Rehearse this before the event, and make sure the channel declared on the Final RF Form matches whatever is actually running.

### 7. Verify it's actually broadcasting on 5 GHz

```bash
iw dev wlx00c0caba86c1 info
```

Confirm the output shows a 5 GHz frequency (e.g. `5180 MHz`) and the correct channel.

### 8. SSH command

Now your SSH command should always be:

```bash
ssh indomitus-rover@10.42.0.1
```