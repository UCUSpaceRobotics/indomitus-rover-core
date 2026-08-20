# Communication architecture

Three separate links, three separate subnets. Each device's IP depends on
which link it's using.

```
                     ┌──────────────┐
   10.42.0.0/24      │   Jetson     │     10.43.0.0/24
  (rover hotspot)    │   (rover)    │   (laptop tether)
        ┌────────────┤  10.42.0.1   ├───────────────┐
        │            │ (AP, static) │               │
   Laptop / clients  └───────┬──────┘         Laptop (direct)
   10.42.0.50–150            │               10.43.0.1 (static)
                             │
                             │ Wi-Fi client
                             | Jetson: DHCP
                    ┌────────┴───────┐
                    │     Mast Pi    │
                    │ (Raspberry Pi) │
                    │    10.42.0.2   │
                    └───────┬────────┘
                            │ wired, 10.44.0.0/24
                      ┌─────┴─────┐
                      │   GS PC   │
                      │ 10.44.0.10│
                      └───────────┘
                Mast Pi's wired side: 10.44.0.1
```

## 1. Laptop ↔ Jetson, over rover Wi-Fi hotspot

- **Subnet:** `10.42.0.0/24`
- **Jetson:** `10.42.0.1` — static, runs the AP itself (`hostapd`)
- **Laptop / any client:** DHCP-assigned, `10.42.0.50`–`10.42.0.150`
- SSID `IndomitusRover`, 5 GHz only
- `ssh indomitus-rover@10.42.0.1`

## 2. Laptop ↔ Jetson, direct ethernet (ETH0)

- **Subnet:** `10.43.0.0/24`
- **Laptop:** `10.43.0.1` — static, set by the "Jetson Tether" NetworkManager
  profile, which also shares the laptop's internet to the Jetson (for setting refer to [ssh.md](./ssh.md)).
- **Jetson:** DHCP-assigned by the laptop, changes on reconnect — check with
  `ip addr show enP8p1s0` (run on Jetson) or use `indomitus-rover.local` for automatic resolution.
- **ETH0** port on the Jetson must be used. ETH1 is reserved for a real router.

## 3. Jetson ↔ Mast Pi ↔ Ground Station PC

- Jetson reaches the Mast Pi as a **Wi-Fi client on the rover hotspot**
  (same `10.42.0.0/24` network as case 1): Mast Pi = `10.42.0.2`.
- Mast Pi is also **wired to the GS PC** on a separate subnet,
  `10.44.0.0/24`: Mast Pi = `10.44.0.1`, GS PC = `10.44.0.10`.
- The Jetson reaches the GS PC by routing through the Mast Pi:
  ```
  10.44.0.0/24 via 10.42.0.2 dev <rover-wifi-iface>
  ```
- The Mast Pi has a **static** IP, so it doesn't depend on rover DHCP.

## Why the subnets must not overlap

Each device only knows "which interface to use for which subnet." If two
different links claim the same subnet (e.g. laptop tether and the Mast Pi
link both using `10.44.0.0/24`), the Jetson's routing table has two
candidate routes for the same destination and picks one by metric —
possibly sending replies out the wrong interface. This looks like a silent
hang or timeout, not an error. Keep every link on its own subnet
(`10.42`, `10.43`, `10.44`, ...) and this can't happen.

## Quick reference

| Device | Address | Link |
|---|---|---|
| Jetson | `10.42.0.1` | rover Wi-Fi hotspot (AP) |
| Jetson | `10.43.0.x` (DHCP) | direct ethernet from laptop |
| Jetson | `10.42.0.1` | as seen by Mast Pi (same hotspot) |
| Mast Pi | `10.42.0.2` | rover Wi-Fi (client) |
| Mast Pi | `10.44.0.1` | wired to GS PC |
| GS PC | `10.44.0.10` | wired from Mast Pi |
| Laptop | `10.42.0.50`–`150` (DHCP) | rover Wi-Fi hotspot |
| Laptop | `10.43.0.1` (static) | direct ethernet to Jetson |