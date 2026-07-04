# Connection to Jetson over hotspot network


## Prerequisites

* wifi module installed on Jetson
* Jetson creates wifi-hotspot and connects to it on boot up
* your laptop is connected to that same network (`IndomitusRover` with password `12345678`)

> **Important:** If at any stage you are asked for the password for the user on the Jetson, the current password is `1`


## Connect over ssh

1. Connect to `IndomitusRover` Wi-Fi network with password `12345678`

2. Run the command to ssh into the Jetson
   
   ```bash
   ssh <username>@10.42.0.1
   ```

   Change username to appropriate username on Jetson, currently the username is `indomitus-rover` and the actual command is

   ```bash
   ssh indomitus-rover@10.42.0.1
   ```


## Jetson hotspot setup

> **Note:** Follow this only if migrating to new Jetson

1. Create hotspot `IndomitusRover` with password `12345678` and force the connection name to be `Hotspot`:

   ```bash
   sudo nmcli dev wifi hotspot ifname wlan0 con-name "Hotspot" ssid "IndomitusRover" password "12345678"
   ```

2. Verify the connection was created successfully:

   ```bash
   nmcli connection show
   ```

   *You should see in the list connection named `Hotspot`*

3. Set the Jetson hotspot IP manually and enable autoconnect:

   ```bash
   sudo nmcli connection modify Hotspot ipv4.addresses 10.42.0.1/24
   sudo nmcli connection modify Hotspot ipv4.method shared
   sudo nmcli connection modify Hotspot connection.autoconnect yes
   sudo nmcli connection up Hotspot
   ```

4. Now your SSH command should always be:

   ```bash
   ssh indomitus-rover@10.42.0.1
   ```