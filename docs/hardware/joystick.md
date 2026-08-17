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

On a DualSense controller, `joystick_interpreter` paints the RGB light bar with the current drive state:

| Color | Meaning |
|-------|---------|
| 🔴 Red | Motors off — hardware inactive |
| 🟠 Orange | Motors on, controller inactive |
| 🔵 Blue | Yielding to navigation |
| 🟢 Green | Joystick in command |

### Setup

The light bar lives under `/sys/class/leds/*:rgb:indicator/` and is root-owned by default. A udev rule hands it to the `plugdev` group so the node can write to it without root:

```bash
# On the Jetson, over SSH
./scripts/setup_host.sh rover --joystick-led

# On this machine
./scripts/setup_host.sh local --joystick-led
```

> **Note:** Controllers without an RGB bar (e.g. Xbox pads) are simply skipped.
