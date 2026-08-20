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
