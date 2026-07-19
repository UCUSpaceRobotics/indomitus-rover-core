# How to share ROS 2 network between laptop and Jetson over Wi-Fi (deprecated)


## How to ssh into Jetson

### Laptop
``` bash
ip link
```
```bash
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN mode DEFAULT group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
2: wlp2s0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP mode DORMANT group default qlen 1000
    link/ether 14:d4:24:a1:47:51 brd ff:ff:ff:ff:ff:ff
3: docker0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN mode DEFAULT group default
    link/ether 36:55:4a:97:8a:26 brd ff:ff:ff:ff:ff:ff
5: enx00e04c1b6418: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP mode DEFAULT group default qlen 1000
    link/ether 00:e0:4c:1b:64:18 brd ff:ff:ff:ff:ff:ff
```

there's lots of devices, but important is the weird one `enx00e04c1b6418`


```bash
nmcli device status
```
```bash
DEVICE           TYPE      STATE                   CONNECTION
wlp2s0           wifi      connected               yagodanr
enx00e04c1b6418  ethernet  connected               Wired connection 1
lo               loopback  connected (externally)  lo
docker0          bridge    connected (externally)  docker0
```

here we can see what connection is associated with what device. In our case it is `Wired connection 1`

- configure ethernet IP
```bash
sudo nmcli connection modify "Wired connection 1" \
  ipv4.method manual \
  ipv4.addresses 192.168.50.1/24 \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv6.method auto \
  connection.autoconnect yes
```

removes dns, gateway but still use ipv4.

- restart connection
```bash
sudo nmcli connection down "Wired connection 1"
sudo nmcli connection up "Wired connection 1"
```

should output saying successfully:
```bash
Connection 'Wired connection 1' successfully deactivated (D-Bus active path: /org/freedesktop/NetworkManager/ActiveConnection/16)
Connection successfully activated (D-Bus active path: /org/freedesktop/NetworkManager/ActiveConnection/17)
```

- check device address
```bash
ip addr show enx00e04c1b6418
```
```bash
5: enx00e04c1b6418: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP group default qlen 1000
    link/ether 00:e0:4c:1b:64:18 brd ff:ff:ff:ff:ff:ff
    inet 192.168.50.1/24 brd 192.168.50.255 scope global noprefixroute enx00e04c1b6418
       valid_lft forever preferred_lft forever
    inet6 fe80::2469:1715:8b73:420d/64 scope link noprefixroute
       valid_lft forever preferred_lft forever
```

here we see inet and inet6 which are IPv4 and IPv6 addresses of the devices on our Laptops

- ping Jetson (preconfigured to static IP 192.168.50.2)
```bash
ping 192.168.50.2 # static IP configured on Jetson
```
```bash
PING 192.168.50.2 (192.168.50.2) 56(84) bytes of data.
64 bytes from 192.168.50.2: icmp_seq=1 ttl=64 time=1.67 ms
64 bytes from 192.168.50.2: icmp_seq=2 ttl=64 time=0.967 ms
64 bytes from 192.168.50.2: icmp_seq=3 ttl=64 time=0.932 ms
...
```

- connect to Jetson
``` bash
ssh ros@192.168.50.2
```

Hope that helped!


## Connect Jetson to Wifi
### Jetson

- turn on wifi
```bash
sudo nmcli radio wifi on
sudo ip link set wlan0 up
```

check status with
```bash
nmcli device status
```

Real example output:
```bash
DEVICE           TYPE      STATE      CONNECTION
wlan0            wifi      connected  yagodanr 4
eth0             ethernet  connected  jetson-direct-eth
br-0eecd4ddeb92  bridge    connected  br-0eecd4ddeb92
docker0          bridge    connected  docker0
l4tbr0           bridge    unmanaged  --
dummy0           dummy     unmanaged  --
rndis0           ethernet  unmanaged  --
usb0             ethernet  unmanaged  --
lo               loopback  unmanaged  --
```

- scan for networks
```bash
sudo nmcli device wifi rescan
nmcli device wifi list
```
```bash
IN-USE  SSID       MODE   CHAN  RATE        SIGNAL  BARS  SECURITY
        WIFI-UCU   Infra  149   540 Mbit/s  70      ▂▄▆_  WPA2 802.1X
        UCU_Guest  Infra  149   540 Mbit/s  70      ▂▄▆_  --
        WIFI-UCU   Infra  1     260 Mbit/s  69      ▂▄▆_  WPA2 802.1X
        UCU_Guest  Infra  1     260 Mbit/s  69      ▂▄▆_  --
*       yagodanr   Infra  149   270 Mbit/s  63      ▂▄▆_  --
        WIFI-UCU   Infra  40    270 Mbit/s  35      ▂▄__  WPA2 802.1X
        UCU_Guest  Infra  40    270 Mbit/s  35      ▂▄__  --
        UCU_Guest  Infra  11    130 Mbit/s  34      ▂▄__  --
        WIFI-UCU   Infra  11    130 Mbit/s  30      ▂___  WPA2 802.1X
```

- connection to Wifi.

  * if with password
    ```bash
    sudo nmcli device wifi connect "WIFI_NAME" password "WIFI_PASSWORD"
    ```

  * UCU_Guest
    ```bash
    sudo nmcli device wifi connect "UCU_Guest"
    ```


- Check IP address
```bash
ip addr show wlan0
```
```bash
7: wlan0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP group default qlen 1000
    link/ether 98:af:65:2c:d0:99 brd ff:ff:ff:ff:ff:ff
    inet 10.10.240.59/20 brd 10.10.255.255 scope global dynamic noprefixroute wlan0
    valid_lft 1682sec preferred_lft 1682sec
    inet6 fe80::70bc:2780:86a6:f2ad/64 scope link noprefixroute
    valid_lft forever preferred_lft forever
```
the important part is actually `inet 10.10.240.59/20` -> `10.10.240.59` -- ID for ssh


Now we can connect to Jetson over Wifi:
```bash
ssh ros@10.10.240.59
```
Now connected over Wifi!!!

## Send messages over network
... Just have same `ROS_DOMAIN_ID` and you should be good to go.

Jeston:
```bash
cd ***
docker compose up -d
docker exec -it indomitus-rover-core bash
```
subscribe to the topic to echo
```bash
ros2 topic echo /chatter std_msgs/msg/String
```

Laptop:
```bash
cd ***
docker compose up -d
docker exec -it indomitus-rover-core bash
```
publish to the topic
```bash
ros2 topic pub /chatter std_msgs/msg/String "{data: 'hello over wifi'}"
```

That should transfer messages



## Troubleshooting
!!!Important!!!:
- `ROS_LOCALHOST_ONLY` -- environmental variable that has to be set to 0. Otherwise, no networking in ROS
- `network_mode: host` is set in docker compose file. That deletes docker network virtualization, that it has eth0 connection to your pc and pc gives it internet. Weird thing.
- DDS might be a problem. In theory. I didn't get to it