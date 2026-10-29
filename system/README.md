# Host Config

This directory contains configs, services, rulles and other stuff for the NVIDIA Jetson used on the rover

## Subsystem autostart (`systemd/rover-launch-*.service`)

- Each unit types its `ros2 launch …` into tmux session `core` / `arm` in `rover_prod`, as if typed by hand.
- Waits in the background (boot not delayed) for its hardware, then for the container:
  - `core`: CAN `can_rover` up (`Environment=CAN_IFACE=`, = `rover.launch.py` default `interface`);
  - `arm`: CAN `can0` up (= `arm_macro.xacro` default `can_interface`);
- Container itself comes back at boot via `restart: unless-stopped` (not after `docker compose down`).
- Install: `system/setup.sh --service rover-launch-core,rover-launch-arm` (installed stopped + disabled).
- Autostart at boot: `sudo systemctl enable rover-launch-arm`; dev: `disable` and launch by hand.
- Watch / Ctrl-C / re-run: `./scripts/enter_container.sh rover arm`.
- `systemctl stop` = Ctrl-C in the session.
- Don't `start` a unit whose subsystem already runs in its session: the command gets typed into the running launch and runs after it exits.
- Cameras run inside `core` (`rover.launch.py use_cameras:=true`).
- Launch args: edit the `ExecStart=… send-keys` line in the unit.
