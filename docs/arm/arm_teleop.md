# Arm control (hardware + visualization)

> Arm packages live in `src/arm/`. Build them inside the Docker workspace:
> ```bash
> cd /opt/ws
> colcon build --symlink-install --packages-select-regex "^arm_"
> source install/setup.bash
> ```

## Modes

| Mode | Launch | Use case |
|---|---|---|
| GUI-only RViz | `arm_viz/urdf_preview.launch.py` | Inspect URDF / joint sliders — **no** motors, **no** ros2_control |
| MoveIt stack | `arm_bringup/arm.launch.py` | Plan&Execute + `servo_node` |
| Cartesian teleop | `ros2 launch arm_teleop keyboard.launch.py` | After demo is up; gamepad: `arm_teleop/gamepad_joy.launch.py` (+ `gamepad_servo.launch.py` in sims) |

On the host (for RViz):

```bash
xhost +local:docker
```

## Hardware Cartesian teleop

### 1. CAN

```bash
# host
./scripts/setup_host.sh local --can   # once
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
```

### 2. Stack, then input (container)

Terminal 1 — MoveIt + controllers + Servo:

```bash
ros2 launch arm_bringup arm.launch.py use_fake_hardware:=false
```

The real arm starts with its motors **off**: `ArmCanSystem` is configured but
inactive — it reads every motor (live `/arm/joint_states`, including measured
torque as `effort`) but sends no torque. Teleop's **activate** (keyboard **c**,
gamepad **R1+A**) turns them on, holding the current pose while the stiffness
ramps up (`gain_ramp_secs`, 15 s). `arm_initial_state:=active|inactive`
overrides the default (inactive on real hardware, active on mock).

Runs **headless by default** (works on the Jetson, no display). To watch live, on a laptop (same `ROS_DOMAIN_ID`):

```bash
ros2 launch arm_viz moveit_rviz.launch.py
```

`arm.launch.py` also always brings up its own `ros2_socketcan` bridge under
the `/arm` namespace (`/arm/socket_can_sender`, `/arm/socket_can_receiver`,
topics `/arm/to_can_bus` / `/arm/from_can_bus`) for the end-effector tool's
CAN traffic (see `arm_peripherals/end_effector_can_node.py`), independent of
any bridge `rover.launch.py` runs.

Terminal 2 — keyboard (wait until spawners / `servo_node` are up):

```bash
ros2 launch arm_teleop keyboard.launch.py
```

Gamepad instead of keyboard:

```bash
ros2 launch arm_teleop gamepad_joy.launch.py     # where the gamepad is plugged in
ros2 launch arm_teleop gamepad_servo.launch.py   # sims only: arm.launch.py already starts it
```
Starts `game_controller_node` (not `joy_node`) — canonical SDL button/axis
mapping, stable across machines/controllers, instead of joy_node's raw
per-device layout (see `GamepadInputLoop`'s docstring).

`gamepad_servo_node` no longer touches CAN directly — the
end-effector's CAN link now lives in a plain node (`arm_peripherals`, the
same `can_msgs/Frame`-over-`ros2_socketcan` pattern `rover_peripherals`'s
lighting node already uses for the rover's own ESP32 peripherals), spawned
by `arm.launch.py` itself. So `gamepad_joy.launch.py` can run on a
**separate machine** (e.g. a laptop) from `arm.launch.py`, the same way
`rover_teleop/gamepad.launch.py` already runs separately from the rover's own
bringup — provided both machines share the same `ROS_DOMAIN_ID` and have
working DDS discovery between them (same requirement as the optional RViz
laptop viewer above).

Do not run keyboard and gamepad together — both publish `/servo_node/delta_twist_cmds`.

### 3. Controls

On the real arm, first **c** / **R1+A** (motors on, see above). Then press **r** (keyboard) or **A** (gamepad) — move to named **`home`**, then start Servo (switches to the streaming controller). **x** / **X** / Esc — stop Servo and give joints back to JTC.

**Keyboard** — two independent translation sets plus rotation about `arm_tcp_link` (everything is rotated into mount before publish):

| Key | Action |
|---|---|
| W / S | EEF +X / −X (mount) |
| A / D | EEF +Y / −Y (mount) |
| Q / E | EEF +Z / −Z (mount) |
| ↑ / ↓ | forward / back (camera) |
| ← / → | left / right (camera) |
| T / G | up / down (camera) |
| I / K | pitch (TCP) |
| U / O | yaw (TCP) |
| J / L | roll (TCP) |
| r | home + start Servo |
| c | activate: motors on |
| z | park: lay the arm on its support (motors stay on) |
| y | release: motors off — only from the park pose |
| Esc / x | exit |

WASD/QE move along **fixed mount axes** — they mean the same thing no matter how the arm is posed. The arrow block moves along the **camera's** axes, so "forward" is always where the camera and gripper are pointing right now.

The frame is `arm_camera_link` (parameter `view_frame`) rather than `arm_tcp_link` because the camera link is REP-103 — `+X` forward, `+Y` left, `+Z` up — which is the same sign convention the mount keys already use. The camera is bolted to `arm_end_effector_link` exactly like the gripper, so both point the same way (verified: camera `+X` · gripper direction = 1.0). `arm_tcp_link` inherits the EEF axes, where the gripper points along `+Z`, so it is **not** a drop-in value for `view_frame`.

Both sets are summed, so holding W and ↑ together gives the sum of the two motions. The view transform is resolved through TF on every publish, not latched at key-press, so a long arrow-key move keeps curving with the camera. If TF for the view frame does not resolve, the arrow keys are ignored and WASD keeps working.

**Gamepad** (e.g. Stadia) — translation is **entirely view-relative** (camera frame); rotation is about TCP, same as the keyboard:

| Input | Action |
|---|---|
| Left stick ←→ | left / right (camera) |
| Left stick ↑↓ | forward / back (camera) |
| Right stick ↑↓ | up / down (camera) |
| Right stick ←→ | yaw (TCP) |
| **R1** + right stick ↑↓ | pitch (TCP) |
| **R1** + right stick ←→ | roll (TCP) |
| A | home + start Servo |
| **R1** + A / BACK / X | activate / park / release (keyboard c / z / y) |
| X | exit |

R1 is a hold-to-shift modifier: while it is down the right stick rotates instead of translating, so the two never mix. L2/R2 and LB are unmapped in this layout.

The default shift button index is **5** (R1/RB on most layouts and pads). Button numbering is not portable across controller models, or even across USB vs Bluetooth on the same pad — a Google Stadia controller over Bluetooth tested 5 as a dead slot, with R1 actually surfacing at button index 10 instead.

If the shift button does nothing on your pad, the node logs the complete `/joy` message on every button change:

```
/joy raw (button(s) [10] changed; shift configured as 5) — axes[6]: {...}  buttons[17]: {..., 10:1, ...}
```

Press the physical R1 alone, read which index actually flips in the `buttons[...]` list (not just whatever this doc or the default says), and pass it:

```bash
ros2 run arm_teleop gamepad_servo_node --ros-args -p gamepad_shift_button:=10
```

**Rotation naming is from the camera's point of view, not the TCP axis letters.** TCP `+X` is the camera's left-right axis → **pitch** (`wx`), TCP `+Y` its vertical axis → **yaw** (`wy`), TCP `+Z` its line of sight → **roll** (`wz`).

Unlike the keyboard, the gamepad has no mount-frame (absolute) translation — the operator is looking through the camera, so every stick axis follows it.

Gamepad mapping stays in `gamepad_servo_node`, not `teleop_twist_joy`: that package publishes all six twist axes in **one** frame, which would break camera-XYZ + TCP-ω.

### 4. Shutdown / releasing the arm

Lay the arm down and turn the motors off from teleop:

1. **Park** (z / R1+BACK): from further than 0.3 rad (any joint) a
   collision-checked MoveIt move to just above the rest support (shoulder and
   elbow `park_approach_lift_rad` = 0.15 rad up), then — or directly, when
   already that close — a slow JTC trajectory down onto it (poses.json `park`;
   shoulder and elbow are aimed `park_press_rad` = 0.03 rad past it, so the arm
   presses lightly on the support and the support carries its weight).
2. **Release** (y / R1+X), refused unless the arm is in the park pose and
   still: `ArmCanSystem::on_deactivate` ramps kp, kd and the gravity
   feed-forward to zero over `release_ramp_secs` (3 s) holding the last
   command — first shoulder and elbow (the arm settles onto the support
   through them), then base and wrists — then disables the motors. The arm
   may move while it settles: the park pose is only approximate, because the
   resting pose itself is in collision for MoveIt. An optional drift check
   (`release_drift_tol_rad` > 0 in `arm_macro.xacro`, off by default) restores
   full stiffness and keeps the arm active if a joint moves more than that.

After Release the motors are free, but positions are still read; **c** turns
them back on.

Turning the motors on and off belongs to the `arm_power` node, not to teleop
(the arm's counterpart of the rover's `drive_power`): **c** and **y** just call
`/arm/power` (`std_srvs/SetBool`, true / false). Anything else can call it too —
`ros2 service call /arm/power std_srvs/srv/SetBool "{data: false}"` releases a
parked arm; a refusal comes back in the reply with its reason. The latched
`/arm/state` (`indomitus_interfaces/ArmState`) says whether the motors are on,
the stiffness ramp is over, the arm is in the park pose, and why the last
request failed. Park itself is a motion and stays in teleop.

Record `park` once, with the arm lying on its support and the motors off:
`ros2 topic echo /arm/joint_states --once`, then add the six arm joints to
`src/arm/arm_teleop/poses.json` under `"park"`. Until then Park and Release
refuse.

Ctrl+C on `arm.launch.py` (or exiting teleop) still does **not** torque-disable
the motors: on process exit (`on_shutdown`) Steadywin (mount/shoulder/elbow)
keeps executing its last MIT command indefinitely, and Damiao (wrists) goes
limp once its comm-loss watchdog trips. Do not assume Ctrl+C means the arm is
safe to let go of. Without the stack, `python3 scripts/arm/disable_motors.py`
(talks to `can0` directly, refuses while MIT frames are still being sent)
disables them — support the arm first, it falls under its own weight.

To test the driver without the arm: `scripts/arm/sim_motors.py --gravity
--support 21:<pos>,22:<pos>` on a `vcan0` (see its docstring) simulates the
support; without `--support` a Release must abort.

### Architecture

```
keyboard / gamepad → TwistStamped (frame: arm_mount_link)
         → MoveIt Servo (inverse Jacobian, ~33 Hz)
         → Float64MultiArray positions
         → JointGroupPositionController
         → ArmCanSystem (MIT)
```

Home / Plan&Execute still use **JTC** (`FollowJointTrajectory`). Controllers are mutually exclusive; the teleop node switches them around **r** / **A** / exit.

Servo `publish_period` is **0.03 s (~33 Hz)** so each joint step stays above MIT stiction (~0.02 rad). Keyboard publishes twists at **100 Hz**; extra messages are overwritten. `controller_manager` / CAN stay at **100 Hz**.

Cartesian Servo is open-loop `J⁺ · twist`. Pure XYZ (`ω = 0`) still lets TCP attitude walk (Q/E especially — shoulder/elbow fold). The keyboard node adds a light **orientation hold** on unused rotation axes and does **not** slow XYZ. That is not industrial Cartesian pose-IK; drift can remain on a 6-DOF arm near the home wrist.

Servo collision checking is **on** (start-time-only param — see `servo.yaml`). Plan&Execute in `move_group` still checks collisions too. Singularity deceleration thresholds are set very high (NHWA-style) so Jacobian condition number does not freeze +X from home.

Before **Plan & Execute** in RViz: stop teleop (exit the input node) so JTC owns the joints.

## Fake hardware (no CAN)

```bash
ros2 launch arm_bringup arm.launch.py use_fake_hardware:=true
ros2 launch arm_teleop keyboard.launch.py
```

## GUI-only (no control)

```bash
ros2 launch arm_viz urdf_preview.launch.py
```

## Named poses

SRDF group states: `home`, `ready`, `experiment` (RViz Goal State).

Also `src/arm/arm_teleop/poses.json` — used by **r** / **A** and:

```bash
ros2 run arm_tasks teach_poses goto home
```

## Fake vs real hardware

| `use_fake_hardware` | Plugin | Behavior |
|---|---|---|
| `true` (default) | `mock_components/GenericSystem` | Instant joint echo — no CAN |
| `false` | `arm_hardware_interface/ArmCanSystem` | Real MIT motors on `can0` |
