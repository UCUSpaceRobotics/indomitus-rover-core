# Host Config

This directory contains configs, services, rulles and other stuff for the NVIDIA Jetson used on the rover

## Subsystems and missions (`systemd/rover-launch-*.service`, `rover-mission`)

- One unit per subsystem; each runs its `ros2 launch …` in tmux session `core` / `cameras` / `nav` / `arm` in `rover_prod`, as if typed by hand.
- Unit → [`rover-session`](rover-session.sh) (`/usr/local/bin/`; also by hand: `rover-session status core`, `rover-session stop core`):
  - start: waits for its CAN (`Environment=CAN_IFACE=`) and the container, stops whatever runs in the session, types the launch;
  - stop: Ctrl-C, waits up to 30 s for the prompt, then kills the session (leftover nodes possible - check `ps`);
  - log per start: `/bags/logs/<session>_<date>_<boot-id>.log` (host `~/bags/logs`; no RTC → date repeats across boots).
- Hardware waits: `core` → `can_rover` (= `rover.launch.py` `interface`), `arm` → `can0` (= `arm_macro.xacro` `can_interface`), `cameras` / `nav` → none (they wait / respawn themselves).
- `Type=simple` + `RemainAfterExit`: waiting doesn't hold boot; unit stays `active` after the launch is typed.
- Missions: [`rover-mission`](rover-mission.sh) (`/usr/local/bin/`; from a laptop `scripts/mission.sh`):
  - `rover-mission nav` - start the mission's subsystems, stop the rest; running ones with unchanged args untouched;
  - `rover-mission nav --default` - same + enable exactly these units at boot;
  - `stop`, `restart cameras`, `status`, `list`;
  - missions: [`missions/<mission>.conf`](missions/) → `/etc/rover/missions/` - one `subsystem [extra launch args]` per line, in start order (e.g. `cameras zed2i_mode:=nav`);
  - subsystems: whatever `rover-launch-*.service` units are installed - a new one = a new unit, nothing else.
- Per-mission args → `LAUNCH_ARGS` in `/run/rover/<subsystem>.env` (this boot) over `/etc/rover/<subsystem>.env` (`--default`), appended to the unit's launch.
- Container itself comes back at boot via `restart: unless-stopped` (not after `docker compose down`). Restarted container → units still `active`; `rover-mission` checks the session, re-run the mission.
- Install: `scripts/setup_host.sh rover --service rover-launch-core,rover-launch-cameras,rover-launch-nav,rover-launch-arm` (also installs both scripts + `missions/`; units stopped + disabled).
- Watch / Ctrl-C / re-run by hand: `./scripts/enter_container.sh rover core`.
- Fixed launch args: edit `ExecStart=` in the unit.
