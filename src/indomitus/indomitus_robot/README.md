# indomitus_robot

- Metapackage: everything that runs on the rover's Jetson (`colcon build --packages-up-to indomitus_robot`).
- Also what runs per mission: `missions/`, `scripts/rover-mission`, `scripts/rover-session`.

## Missions

- `missions/<mission>.conf` - one line per tmux session, in start order:
  ```
  # session   [can=IFACE]    command
  core        can=can_rover  ros2 launch rover_bringup rover.launch.py
  cameras                    ros2 launch rover_bringup cameras.launch.py zed2i_mode:=nav
  ```
  - `command` is typed into session `<session>` as if by hand;
  - `can=` - the session first waits for that CAN interface to be UP (shown in the session, Ctrl-C cancels);
  - new mission = new file; names `default`, `stop`, `restart`, `enter`, `status`, `list` are taken.
- Commands live in each mission (no shared definitions): changing e.g. `core`'s launch = every mission that has it.

## Commands (in the container, dev and prod - on `PATH` via `docker/Dockerfile`)

- `rover-mission nav` - start the mission's sessions, stop other rover sessions; same command already running → left alone.
- `rover-mission nav --default` - same + it's the one `rover-mission default` starts (at boot).
- `rover-mission stop` / `restart cameras` / `status` / `list` / `enter core [-r]`.
- `rover-session start|stop|enter|status NAME` - one session (used by `rover-mission`, handy by hand).
- From a laptop: `scripts/mission.sh nav` (ssh + `docker exec rover_prod rover-mission …`).
- Run from the bind-mounted `src/`: editing a mission or a script needs no rebuild.

## State

- Default mission: `default-mission` (one line), written by `--default`; not in git, not overwritten by the deploy (`scripts/.rsync-filter-deploy`).
- Everything else lives in tmux and goes away with the sessions: current mission (`@rover_mission`), each session's command (`@rover_cmd`, `@rover_can`).
- Only sessions started by `rover-session` are touched; your own (`sim`, …) never are.

## Sessions

- Stop: Ctrl-C, wait up to 30 s for the prompt, then kill the session (leftover nodes possible - check `ps`).
- Log per start: `/bags/logs/<session>_<date>_<boot-id>.log` (Jetson has no RTC → date repeats across boots).

## Boot / shutdown

- [`system/systemd/rover-mission.service`](../../../system/systemd/rover-mission.service): `rover-mission default` at boot, `rover-mission stop` before Docker stops.
- Skipped unless `rover_prod` runs within 30 s: the Jetson also hosts other projects.
