# Host Config

This directory contains configs, services, rulles and other stuff for the NVIDIA Jetson used on the rover

## Subsystems and missions (`systemd/rover-launch@.service`, `rover-mission`)

- Config + scripts live in the repo, in [`indomitus_robot`](../src/indomitus/indomitus_robot/) - edit there, deploy, done (no host reinstall):
  - `subsystems/<name>.conf` - `COMMAND=` (its `ros2 launch …`), `CAN_IFACE=` to wait for (empty: none); new subsystem = new file;
  - `missions/<mission>.conf` - one `subsystem [extra launch args]` per line, in start order (e.g. `cameras zed2i_mode:=nav`);
  - `scripts/rover-session`, `scripts/rover-mission` - run on the host from `~/indomitus-rover-core/src/...` (the path the deploy syncs to).
- Host: one template unit, an instance per subsystem (`rover-launch@core.service`, …) → `rover-session start|stop <name>`:
  - start: waits for its CAN and the container, stops whatever runs in tmux session `<name>`, types `COMMAND` + mission args;
  - stop: Ctrl-C, waits up to 30 s for the prompt, then kills the session (leftover nodes possible - check `ps`);
  - skipped (not failed) unless `rover_prod` runs within 30 s, or the repo / `<name>.conf` is missing: the Jetson hosts other projects too;
  - log per start: `/bags/logs/<name>_<date>_<boot-id>.log` (host `~/bags/logs`; no RTC → date repeats across boots).
- `Type=simple` + `RemainAfterExit`: waiting doesn't hold boot; unit stays `active` after the launch is typed.
- Missions (`rover-mission`, from a laptop `scripts/mission.sh`):
  - `rover-mission nav` - start the mission's subsystems, stop the rest; running ones with unchanged args untouched;
  - `rover-mission nav --default` - same + enable exactly these instances at boot;
  - `stop`, `restart cameras`, `status`, `list`.
- Per-mission args → `LAUNCH_ARGS` in `/run/rover/<name>.env` (this boot) over `/etc/rover/<name>.env` (`--default`).
- Restarted container → units still `active`; `rover-mission` checks the session, re-run the mission.
- Install: `scripts/setup_host.sh rover --service rover-launch@` (stopped + disabled), then `rover-mission <mission> --default`.
- Replaces `rover-launch-{core,cameras,arm}.service`: on a Jetson that has them, `sudo systemctl disable --now rover-launch-{core,cameras,arm}` and remove them from `/etc/systemd/system/`.
- By hand (dev): `rover-session status core`, `rover-session stop core`, `rover-session start core ros2 launch … custom:=args`; watch / Ctrl-C: `./scripts/enter_container.sh rover core`.
