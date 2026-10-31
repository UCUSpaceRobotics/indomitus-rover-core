# Host Config

This directory contains configs, services, rulles and other stuff for the NVIDIA Jetson used on the rover

## Missions at boot (`systemd/rover-mission.service`)

- Runs `rover-mission default` in `rover_prod` at boot, `rover-mission stop` before Docker stops; all logic + missions: [`indomitus_robot`](../src/indomitus/indomitus_robot/README.md).
- Skipped (not failed) unless `rover_prod` runs within 30 s: the Jetson also hosts other projects.
- Install once: `scripts/setup_host.sh rover --service rover-mission`, then `sudo systemctl enable rover-mission`; pick the mission: `scripts/mission.sh nav --default`.
- Replaces `rover-launch-{core,cameras,arm}.service`: on a Jetson that has them, `sudo systemctl disable --now rover-launch-{core,cameras,arm}` and remove them from `/etc/systemd/system/`.
