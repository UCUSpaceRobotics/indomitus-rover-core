# Joystick

The joystick connects to the Jetson either over USB or over Bluetooth.

| Method | When to use |
|--------|-------------|
| USB cable | Default, especially during field operation |
| Bluetooth dongle | When a cable is impractical |

## USB Connection

Plug **USB-A** into the Jetson and **USB Type-C** into the joystick. No further setup is needed.

## Bluetooth Connection

1. Plug the Bluetooth dongle into the USB hub.

   > ⚠️ Unplug the dongle before rebooting — the Jetson will not boot with it attached.

2. If the joystick is not paired yet, pair it with `bluetoothctl`:

   ```bash
   bluetoothctl
   ```

   | Command | Purpose |
   |---------|---------|
   | `scan on` / `scan off` | Start / stop discovery of nearby devices |
   | `pair <mac-address>` | Pair with the joystick |
   | `connect <mac-address>` | Connect to a paired joystick |
   | `remove <mac-address>` | Forget the device (use before re-pairing) |

3. Toggle the power button on the joystick — it connects automatically.

## Joystick Layout

![Joystick layout](../assets/joystick_layout.png)

## Light Bar

On a PlayStation controller, `joystick_interpreter` paints the light bar with the current drive state. The colours are ordered by severity — the bar always shows the most serious state that applies, so green means the joystick can move the rover *right now*:

| Color | Meaning |
|-------|---------|
| 🔴 Red | Motors off — hardware inactive |
| 🟣 Magenta | Motor faults cleared — cycle the motor button to re-enable |
| 🟠 Orange | Motors on, controller inactive |
| 🔵 Blue | Yielding to navigation |
| 🟢 Green | Joystick in command |
| ⚪ White | `joystick_interpreter` is not running |

White is painted as the node shuts down.

Magenta appears after the clear-errors button: the faults are gone, but the hardware will not drive again until the motor button is cycled off and on.

The drive state itself is read off `drive/state`, which `drive_power_node` publishes — the joystick no longer keeps its own copy, so the bar tracks the drive even when the ground station is the one commanding it. With `drive_power_node` not running nothing arrives on that topic and the bar stays red, which is the truth: no controller has been activated, so the rover cannot move.

## Unplugging the Joystick

When `/joy` goes stale the interpreter publishes a short burst of zero twists and then **stops publishing entirely**.

This matters because `cmd_vel_joy` outranks every other command source in twist_mux. A node that kept publishing zeros would hold the top priority forever, so an unplugged gamepad would lock the ground station out of a rover nobody was driving. Going quiet lets the mux time this input out (0.5 s) and fall through to whoever is still there.

Nothing is lost by the silence: the swerve controller has its own `cmd_vel_timeout_s`, so an unattended rover stops either way.

To hand control over deliberately without unplugging anything, press the active-toggle button — the bar goes blue and the ground station or nav2 takes the mux immediately.
