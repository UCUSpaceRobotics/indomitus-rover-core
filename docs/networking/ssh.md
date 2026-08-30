# SSH

- [SSH](#ssh)
  - [SSH via Hotspot](#ssh-via-hotspot)
  - [SSH via Ethernet](#ssh-via-ethernet)
    - [The Connection Steps](#the-connection-steps)
    - [Laptop Setup (First Time Only)](#laptop-setup-first-time-only)
    - [Jetson Setup (First Time Only)](#jetson-setup-first-time-only)
      - [Network Routing Priority (Multiple Connections)](#network-routing-priority-multiple-connections)
  - [Passwordless Connection](#passwordless-connection)
  - [Github Agent Forwarding](#github-agent-forwarding)
  - [Configure the SSH Shortcut \& Tunnel](#configure-the-ssh-shortcut--tunnel)
    - [The New Workflow](#the-new-workflow)

SSH is a very useful tool to remotely access the Jetson on the rover, enter the Docker containers running on it, or deploy code to it.

> **Important:** If at any stage you are asked for the password for the user on the Jetson, the current password is `1`



## SSH via Hotspot

**Prerequisites:** Hotspot on the Jetson is setup. Refer to the [hotspot.md](./hotspot.md)

> **Note:** The current static IP for the Jetson hotspot is `10.42.0.1`

1. Connect your laptop to the `ERC_UCUSpaceRobotics_A` Wi-Fi network with the password `19283746`.

2. Run one of the following commands to SSH into the Jetson:

   Either use ip addres:
   ```bash
   ssh indomitus-rover@10.42.0.1
   ```

   or automatic ip resolution:
   ```bash
   ssh indomitus-rover@indomitus-rover-computer.local
   ```



## SSH via Ethernet

**Prerequisites:**

* The Jetson and laptop is set up for Ethernet SSH connection. Refer to the [Laptop Setup](#laptop-setup-first-time-only) and [Jetson Setup](#jetson-setup-first-time-only) 
* An Ethernet cable connects your laptop to the Jetson's `ETH0` port (labeled on the front panel).

> **Note:** Over Ethernet, the Jetson does not rely on a static IP. You use the `.local` hostname, and the correct IP is resolved automatically.


### The Connection Steps

1. Click the Network icon in your laptop's taskbar.

2. Select the **Jetson Tether** profile.

3. Run the SSH command using the Jetson's local hostname:

  ```bash
   ssh indomitus-rover@indomitus-rover-computer.local
   ```


### Laptop Setup (First Time Only)

To ensure your laptop's Ethernet port can be used for **both** connecting to the Jetson and connecting to standard wired internet, we will create two distinct network profiles.

1. Open your laptop's terminal.

2. **Create the "Normal Internet" profile:** This ensures your port functions normally when plugged into a standard router or wall jack (assuming your ethernet port is `eno1`).

   ```bash
   sudo nmcli connection add type ethernet ifname eno1 con-name "Normal Internet" ipv4.method auto
   ```

3. **Create the "Jetson Tether" profile:** This configures the port to share your laptop's Wi-Fi internet with the Jetson when plugged into the rover.

   ```bash
   sudo nmcli connection add type ethernet ifname eno1 con-name "Jetson Tether" ipv4.method shared ipv4.addresses 10.43.0.1/24
   ```

**How to switch connections:** From now on, when you plug in an Ethernet cable, click the Network icon in your taskbar. Choose **Jetson Tether** when plugging into the rover, or **Normal Internet** when plugging into a router.


### Jetson Setup (First Time Only)

Ensure the Jetson broadcasts its `.local` name over the cable:

1. SSH into the Jetson (e.g., via the Hotspot first) or use a display and keybord connected to it.

2. Enable and start the Avahi daemon:

   ```bash
   sudo systemctl enable avahi-daemon
   ```

   ```bash
   sudo systemctl start avahi-daemon
   ```


#### Network Routing Priority (Multiple Connections)

To ensure the Jetson prioritizes internet from a router (**ETH1**) over your laptop (**ETH0**), we will configure the laptop connection as a fallback. First, let's rename Ubuntu's generic connection name (e.g., "Wired connection 1") to **ETH0** to prevent configuration errors.

1. **Find your active connection name:** Plug Ethernet cable from your laptop to the Jetson's **ETH0** port. Then run this command on the Jetson to see what NetworkManager is currently calling the port plugged into your laptop:

   ```bash
   nmcli connection show --active
   ```

   *Example output:*

   ```text
   NAME                UUID                                  TYPE      DEVICE  
   Wired connection 1  a7e9e939-2ff3-32c4-8b6c-e8360f75a304  ethernet  enP8p1s0
   ```

2. **Rename the connection:** Look at the `NAME` column from your output. Use that exact name in the following command to permanently rename it to `ETH0` (replace `"Wired connection 1"` if your output showed something different):

   ```bash
   sudo nmcli connection modify "Wired connection 1" connection.id "ETH0"
   ```

   *For consistency you may do the same for **ETH0** port.*

3. **Set the fallback metric:** Now that the connection is safely named `ETH0`, set it as a fallback gateway by assigning it a high route metric (low priority). Run these commands:

   ```bash
   sudo nmcli connection modify "ETH0" ipv4.never-default no ipv4.route-metric 10000
   sudo nmcli connection up "ETH0"
   ```

> **Important:** From now on you **must** use the **ETH0** port only to connect the Jetson to the laptop, and **ETH1** for connecting the Jetson to the router. If you plug the Ethernet cable from the router into **ETH0**, the Jetson will incorrectly treat the router as a low-priority fallback.


## Passwordless Connection

For convenience, you should set up passwordless SSH. Run these commands on your laptop terminal (connect to the Jetson via Hotspot or Ethernet first):

1. Generate an SSH key pair (press Enter at all prompts to accept the defaults. Skip this if you already have an SSH key):

   ```bash
   ssh-keygen -t ed25519
   ```

2. Copy the public key to the Jetson (you will be prompted for the Jetson password one last time):

   ```bash
   ssh-copy-id indomitus-rover@indomitus-rover-computer.local
   ```



## Github Agent Forwarding

This allows you to pull from GitHub directly on the Jetson using your laptop's private SSH key, meaning your keys never leave your laptop.

To make sure your laptop always remembers your GitHub keys when you open a terminal:

1. Install keychain on your laptop:

   ```bash
   sudo apt install keychain
   ```

2. Add this line to the bottom of your `~/.bashrc` or `~/.zshrc` file depending on your shell (replace `id_ed25519` with your actual key name if different):

   ```bash
   eval $(keychain --eval --quiet id_ed25519)
   ```



## Configure the SSH Shortcut & Tunnel

To simplify the connection command and automatically enable key forwarding, configure a local SSH profile.

1. Open your SSH config file on your laptop:

   ```bash
   nano ~/.ssh/config
   ```

2. Add this block to the bottom of the file to create the `rover` shortcut and enable the tunnel:

   ```text
   Host rover
     HostName indomitus-rover-computer.local
     User indomitus-rover
     ForwardAgent yes
   ```


### The New Workflow

You can now connect to the rover simply by typing:

```bash
ssh rover
```

> **Note:** The same command will work for both ethernet and hotspot connection. If you connected to Jetson with both ethernet cable and hotspot the ethernet will win when running the command above and you will ssh into the Jetson via ethernet.