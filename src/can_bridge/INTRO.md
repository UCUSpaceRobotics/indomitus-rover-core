# Can emulation

since we do not have can bus in our notebook, we
will simulate via slcand

```bash
sudo apt install can-utils

# Підняти віртуальний can0 поверх ttyUSB0
sudo slcand -o -c -s8 /dev/ttyUSB0 can0
sudo ip link set up can0

# Перевірити
candump can0
```

Check:
```bash
ip link show can0
```
should show UP
