# Rover Peripherals

## Container (ESP)

### Launch file & prerequisites

> ⚠️ Warning: This launch files assumes that ros2_socketcan is already running!

```bash
ros2 launch indomitus_rover_bringup container.launch.py
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