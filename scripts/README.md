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
| **`deploy_to_rover.sh`** | Pulls image from the github, then deploys; synchronizes the whole workspace and the builds the image on Jetson; builds localy and deploys the new image; synchronizes the src/docker-compose.prod.yaml with the Jetson without image rebuild. | [Read Docs](../docs/scripts/deploy_to_rover.md) |
| **`test_motors.py`** | A standalone hardware utility to quickly verify CAN bus communication and basic motor behavior. | - |
| **`setup_host.sh`** | Deploys the CAN udev rule and rover systemd service to the Jetson. Preserves the previous enable state of `rover.service`. | [Read Docs](../docs/scripts/setup_host.md) |
| **`navigation/compare_pose.py`** | Compares RViz's estimated rover pose against Gazebo ground truth (sim only). | [Read Docs](../docs/scripts/navigation_diagnostics.md) |
| **`navigation/track_pose_drift.py`** | Tracks drift of an estimated pose from its starting value over time (sim or real hardware). | [Read Docs](../docs/scripts/navigation_diagnostics.md) |
