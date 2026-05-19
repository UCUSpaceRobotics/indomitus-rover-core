# Jetson Code Sync

This script synchronizes your codebase from your PC to the Jetson device. Because when the Jetson hosts the network it cannot connect to the WI-Fi so the code changes cannot be pulled directly. This tool provides a reliable way to push your latest updates directly over the local ssh connection.

## Usage Instructions

**Step 1: Make the script executable**
Before running the script for the first time, you need to grant it execution rights. Run the following command:

```bash
chmod +x scripts/sync.sh
```

**Step 2: Run the sync script**
Run the script from your terminal using the following command format:

```bash
./scripts/sync.sh --local-dir /path/to/your/code
```

## Script Flags

You can pass the following flags to adjust the sync parameters:

* **`-l`, `--local-dir`**: The local directory on your PC that you want to sync. (Default: `./`)
* **`-u`, `--user`**: The username for the Jetson device. (Default: `ros`)
* **`-i`, `--ip`**: The IP address of the Jetson. (Default: `10.42.0.1`)
* **`-d`, `--remote-dir`**: The target directory on the Jetson. (Default: `/home/ros/Indomitus/indomitus-rover-core`)

> **Note:** The remote user, remote host, and remote path flags should only be changed with a strong reason. The script is designed so that everything works correctly out of the box without altering these default parameters.