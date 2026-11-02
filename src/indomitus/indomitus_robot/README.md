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
  - new mission = new file.
- Hardware: `drive`, `nav`, `arm`, `all`; simulation (dev container, Gazebo): `drive-sim`, `nav-sim`, `arm-sim`, `all-sim`.
  - only a naming convention: any mission runs anywhere (`arm` in sim → waits for `can0` forever).
- Commands live in each mission (no shared definitions): changing e.g. `core`'s launch = every mission that has it.

## Commands (in the container, dev and prod - on `PATH` via `docker/Dockerfile`)

- `rover-mission start nav` - start the mission's sessions, stop other rover sessions; same command already running → left alone.
- Command typed by hand in a rover session (not in any mission, e.g. own flags) → left alone with a warning; `--force` replaces / stops it.
- `rover-mission start nav --default` - same + it's the one `rover-mission default` starts (at boot).
- `rover-mission stop [nav]` - stops the mission's sessions / every rover session, hand-typed commands too.
- `rover-mission restart nav` - stops its sessions, starts them fresh with the mission's commands.
- `rover-mission status` / `list` / `enter core [-r]`.
- `rover-session start|stop|restart|enter|status NAME` - one session (used by `rover-mission`, handy by hand); `restart` = Ctrl-C + `!!` (reruns the last command there, also a hand-typed one).
- From a laptop: `scripts/missions.sh {rover,local} start nav` - `rover`: ssh + `docker exec rover_prod`, `local`: `docker exec rover_dev`.
- Run from the bind-mounted `src/`: editing a mission or a script needs no rebuild.

## State

- Default mission: `default-mission` (one line), written by `--default`; not in git, not overwritten by the deploy (`scripts/.rsync-filter-deploy`).
- Current mission: tmux `@rover_mission`, goes away with the tmux server.
- What runs in a session: read from its processes (`rover-session status`: shell's child command line, `ros2 launch …`), so hand-typed commands are seen as they are.
- Rover sessions = names used in any mission; your own (`sim`, …) are never touched.

## Sessions

- Stop: Ctrl-C, wait up to 30 s for the prompt, then kill the session (leftover nodes possible - check `ps`).
- Log per start: `/bags/logs/<session>_<date>_<boot-id>.log` (Jetson has no RTC → date repeats across boots).

## Boot / shutdown

- [`system/systemd/rover-mission.service`](../../../system/systemd/rover-mission.service): `rover-mission default` at boot, `rover-mission stop` before Docker stops.
- Skipped unless `rover_prod` runs within 30 s: the Jetson also hosts other projects.
