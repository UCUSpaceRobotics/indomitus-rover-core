# Container interface for ROS2 on main pc

## Main purpose
Container should have two main functions:
- Close/open container (realized via actions, since it takes time to accomplish this mission)
- Take weight measurements from sensor and send to Jetson (via service, responce time shouldn't be that long)

## Launch file & prerequisites

> ⚠️ Warning: This launch files assumes that ros2_socketcan is already running!

```bash
ros2 launch indomitus_rover_bringup container.launch.py
```

## Interface details

Action:  /container/lid  (indomitus_interfaces/action/ContainerLid)
  goal.open = True  -> open lid   (CAN cmd_id, byte 0 = cmd_open)
  goal.open = False -> close lid  (CAN cmd_id, byte 0 = cmd_close)
  feedback.status   -> "opening" | "closing" | "done" | "error"
  result.success    -> True if ESP32 finished OK within timeout
  result.message    -> human-readable status

Service: /container/weight  (indomitus_interfaces/srv/GetWeight)
  request:  (empty)
  response: float32 weight, bool success, string message

## CAN interface (id that I used)

CAN TX (PC -> ESP32)  ID cmd_id:
  byte 0 = cmd_open | cmd_close | cmd_poll_status | cmd_get_weight

CAN RX (ESP32 -> PC):
  ID lid_resp_id    byte 0 = echo cmd, byte 1 = status
    status: 0x00 ACK | 0x01 IN_PROGRESS | 0x02 DONE | 0x03 ERROR
  ID weight_resp_id bytes 0-3 = float32 weight little-endian
