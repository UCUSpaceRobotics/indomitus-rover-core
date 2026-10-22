# Host Config

This directory contains configs, services, rulles and other stuff for the NVIDIA Jetson used on the rover

## Subsystem autostart (`systemd/rover-launch-*.service`)

- Each unit types its `ros2 launch …` into tmux session `core` / `arm` / `cameras` in `rover_prod`, as if typed by hand.
- Install: `system/setup.sh --service rover-launch-core,rover-launch-arm,rover-launch-cameras` (installed stopped + disabled).
- Autostart at boot: `sudo systemctl enable rover-launch-arm`; dev: `disable` and launch by hand.
- Watch / Ctrl-C / re-run: `./scripts/enter_container.sh rover arm`.
- `systemctl stop` = Ctrl-C in the session.
- Don't `start` a unit whose subsystem already runs in its session: the command gets typed into the running launch and runs after it exits.
- Launch args: edit the `ExecStart=… send-keys` line in the unit.
