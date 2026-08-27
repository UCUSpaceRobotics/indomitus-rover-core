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

   Change username to appropriate username on Jetson, currently the username is `jetson` and the actual command is

   ```bash
   ssh jetson@10.42.0.1
   ```


## Jetson hotspot setup

> **Note:** Follow this only after OS reinstallation or if migrating to a new Jetson


1. Find the interface name by running this command:
    
    ```bash
    ip -br link show
    ```

    *Look for an interface name that starts with wlx followed by a MAC address (for example, `wlx00c0caba86c1`). Copy it, you will need it for the next step.*

2. Create hotspot `IndomitusRover` with password `12345678` and force the connection name to be `Hotspot`. Replace `wlx00c0caba86c1` in the command with the copied name:

   ```bash
   sudo nmcli dev wifi hotspot ifname wlx00c0caba86c1 con-name "Hotspot" ssid "IndomitusRover" password "12345678"
   ```

3. Verify the connection was created successfully:

   ```bash
   nmcli connection show
   ```

   *You should see in the list connection named `Hotspot`*

4. Set the Jetson hotspot IP manually and enable autoconnect:

    ```bash
    sudo nmcli connection modify Hotspot \
    ipv4.addresses 10.42.0.1/24 \
    ipv4.method shared \
    connection.autoconnect yes \
    && sudo nmcli connection up Hotspot
    ```

5. Now your SSH command should always be:

   ```bash
   ssh jetson@10.42.0.1
   ```