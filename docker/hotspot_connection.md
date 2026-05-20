# Connection to Jetson over hotspot network

## Prerequisites

* wifi module installed on Jetson
* Jetson creates wifi-hotspot and connects to it on boot up
* your laptop is connected to that same network (`JetsonRosIndomitus` password `jetson1234`)
* (probably optional) Jetson is setup to have static ipv4 on its network


## Connect over ssh

1. Connect to `JetsonRosIndomitus` Wi-Fi network with password `jetson1234`

2. Run the command to ssh into the Jetson
   
   ```bash
   ssh <username>@10.42.0.1
   ```

   Change username to appropriate username on Jetson, currently the username is `ros` and the actual command is

   ```bash
   ssh ros@10.42.0.1
   ```


## Setup passwordless connection

For convenience the passwordless ssh connection may be set up. To do it run these commands on your laptop terminal:

1. Connect to `JetsonRosIndomitus` Wi-Fi network with password `jetson1234`

2. Generate an SSH key pair (press Enter at all prompts to accept the defaults):

   ```bash
   ssh-keygen -t ed25519
   ```

3. Copy the public key to the Jetson (you will be prompted for the Jetson password one last time):

   ```bash
   ssh-copy-id ros@10.42.0.1
   ```

Now you can run `ssh <username>@10.42.0.1` and connect automatically without a password.

> **Note:** this setup needs to be done only once 


## Jetson hotspot setup

> **Note:** Follow this only if migrating to new Jetson

1. create hotspot `JetsonRosIndomitus` with password `jetson1234`

   ```bash
   sudo nmcli dev wifi hotspot ifname wlan0 ssid "JetsonRosIndomitus" password "jetson1234"
   ```

2. Check the hotspot connection name:

   ```bash
   nmcli connection show
   ```

3. Assume it is called Hotspot. Set the Jetson hotspot IP manually:

   ```bash
   sudo nmcli connection modify Hotspot ipv4.addresses 10.42.0.1/24
   sudo nmcli connection modify Hotspot ipv4.method shared
   sudo nmcli connection modify Hotspot connection.autoconnect yes
   sudo nmcli connection up Hotspot
   ```

4. Now your SSH command should always be:

   ```bash
   ssh ros@10.42.0.1
   ```