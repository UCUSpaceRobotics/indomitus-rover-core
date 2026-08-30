# Rover Peripherals

## Container (ESP)

### Launch file & prerequisites

> ⚠️ Warning: This launch files assumes that ros2_socketcan is already running!

```bash
ros2 launch rover_bringup container.launch.py
```

### ROS2 Interface details

**Action**:  /container/lid  ([indomitus_interfaces/action/ContainerLid](../indomitus_interfaces/action/ContainerLid.action))

  goal.open = True  -> open lid   (CAN cmd_id, byte 0 = cmd_open)  
  goal.open = False -> close lid  (CAN cmd_id, byte 0 = cmd_close)  
  feedback.status   -> "opening" | "closing" | "done" | "error"  
  result.success    -> True if ESP32 finished OK within timeout  
  result.message    -> human-readable status  

**Service**: /container/weight  ([indomitus_interfaces/srv/GetWeight](../indomitus_interfaces/srv/GetWeight.srv))
  request:  (empty)
  response: float32 weight, bool success, string message

### CAN interface

Used IDs: [config/container_can.yaml](config/container_can.yaml)
## Lights (ESP)

### Launch file & prerequisites

> ⚠️ Warning: This launch file assumes that ros2_socketcan is already running!

```bash
ros2 launch rover_peripherals lighting.launch.py
```

Included by `rover_bringup/launch/rover.launch.py`.

### ROS2 Interface details

`lights_can_node` **owns** the light state. Two operators command the same
lights and they ask differently — the onboard joystick has momentary buttons,
the ground station has latching switches — so neither keeps a copy of its own,
and neither can drift out of step with the other. Both read the truth back off
`/lights/state`.

**Services** — absolute forms for the ground station's switches, `/toggle`
forms for the joystick's buttons:

| Service | Type | Meaning |
|---|---|---|
| `/lights/spotlight` | `std_srvs/SetBool` | set the spotlight |
| `/lights/spotlight/toggle` | `std_srvs/Trigger` | invert the spotlight |
| `/lights/beautiful` | `std_srvs/SetBool` | set the decorative light |
| `/lights/beautiful/toggle` | `std_srvs/Trigger` | invert the decorative light |
| `/lights/traffic_light` | [`indomitus_interfaces/SetTrafficLight`](../indomitus_interfaces/srv/SetTrafficLight.srv) | set any subset of the four colours |

`SetTrafficLight` is tri-state per colour: `KEEP=0`, `OFF=1`, `ON=2`. `KEEP`
is zero so a request only ever changes the colours it names — an empty request
is a no-op, and switching blue does not disturb red:

```bash
ros2 service call /lights/traffic_light indomitus_interfaces/srv/SetTrafficLight "{blue: 2}"
ros2 service call /lights/traffic_light indomitus_interfaces/srv/SetTrafficLight "{red: 1}"
```

The traffic light is meant to signal the rover's own state, so nothing on the
joystick or the ground station panel is bound to it. What drives it from rover
state is not built yet — this is the interface it will use.

**Topic**: `/lights/state` ([indomitus_interfaces/msg/LightsState](../indomitus_interfaces/msg/LightsState.msg))

Latched (`TRANSIENT_LOCAL`, depth 1), published on change and rate limited to
`state_pub_rate` (2 Hz) so it does not load the Wi-Fi link to the ground
station. There is no periodic heartbeat, which is exactly why it is latched: a
UI that connects late gets the current state immediately.

```bash
ros2 topic echo /lights/state
```

### CAN interface

Used IDs: [config/lights.yaml](config/lights.yaml)

Traffic-light bitmask is `R=bit0 Y=bit1 G=bit2 B=bit3`, computed in the node
from its own state — the firmware protocol did not change.
