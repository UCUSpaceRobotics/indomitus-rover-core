# Scripts

This directory contains executable utility and automation scripts for the Indomitus Rover project. To keep this folder organized, it should strictly house the execution files (e.g., `.sh`, `.py`); all long-form documentation, guides, and tutorials belong in the [`docs/scripts/`](../docs/scripts/) directory.

Before running any scripts you need to give execution rights to it:
```bash
chmod +x ./script/your_script
```

## Available Scripts

| Script | Description | Documentation |
| :--- | :--- | :--- |
| **`enter_container.sh`** | Connects to local or remote Docker environments, manages container states, and opens an interactive terminal. | [Read Docs](../docs/scripts/enter_container.md) |
| **`deploy_to_rover.sh`** | Builds and deploys the new image or synchronizes the local workspace with the Jetson workspace without image rebuild. | [Read Docs](../docs/scripts/deploy_to_rover.md) |
| **`test_motors.py`** | A standalone hardware utility to quickly verify CAN bus communication and basic motor behavior. | - |
| **`setup_host.sh`** | Deploys the CAN udev rule and rover systemd service to the Jetson. Preserves the previous enable state of `rover.service`. | [Read Docs](../docs/scripts/setup_host.md) |
