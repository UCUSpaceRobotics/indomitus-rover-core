# Connection to Jetson over hotspot network

## Prerequisites
- wifi module installed on Jetson
- Jetson creates wifi-hotspot and connects to it on boot up
- your laptop is connected to that same network(`JetsonRosIndomitus` password "jetson1234")
- (probably optional) Jetson is setup to have static ipv4 on it's network


## Connect over ssh
- connect to `JetsonRosIndomitus` password "jetson1234"
- ```bash
    ssh <username>@10.42.0.1
  ```
  change username to appropriate username on Jetson

## Jetson hotspot setup
> follow this only if migrating to new Jetson

- create hotspot `JetsonROS` with password `jetson1234`
  ```bash
  sudo nmcli dev wifi hotspot ifname wlan0 ssid JetsonROS password "jetson1234"
  ```

- Check the hotspot connection name:
  ```bash
  nmcli connection show
  ```

- Assume it is called Hotspot. Set the Jetson hotspot IP manually:
  ```bash
  sudo nmcli connection modify Hotspot ipv4.addresses 10.42.0.1/24
  sudo nmcli connection modify Hotspot ipv4.method shared
  sudo nmcli connection modify Hotspot connection.autoconnect yes
  sudo nmcli connection up Hotspot
  ```

- Now your SSH command should always be:
  ```bash
  ssh jetson@10.42.0.1
  ```