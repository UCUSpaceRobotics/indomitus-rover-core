#!/usr/bin/env bash
#
# Generates the Fast DDS profile that pins DDS traffic to the rover link.
#
# RUNS ON BOTH ENDS OF THE LINK. The ground station and the rover face the same
# two problems, mirrored, so this is one script parameterised by two values
# rather than two scripts that drift apart:
#
#                    ROVER_LINK_PREFIX   ROVER_PEER
#   ground station   10.44.0.            10.42.0.1    (the rover)
#   rover            10.42.0.            10.44.0.10   (the ground station)
#
# ROVER_LINK names the *path* between the two machines, not the machine at the
# far end: on the rover, ROVER_PEER is the ground station. The names are kept
# as they are because they are already spelled this way in the .env and compose
# files on both sides, and a rename buys nothing but a migration.
#
# KEEP IN SYNC. This file is duplicated in indomitus-ground-station and
# indomitus-rover-core. The rover in the field mounts no checkout of the other
# repo (see docker-compose.prod.yaml: only src/ is bind-mounted) and there is no
# shared package to put this in, so duplication is the honest option. The two
# copies are meant to be byte-identical — diff them before trusting either.
#
# Why this exists: each end has more than one live interface — the rover link,
# plus internet, wired lifeline, docker bridges. Fast DDS announces unicast
# locators for *every* interface it can see, so the far end picks one and aims
# user data at it. When it picks an address that is not routable across the
# link, every sample is dropped, while discovery keeps working perfectly over
# multicast.
#
# The result is the worst kind of failure: `ros2 node list`, `ros2 node info`
# and `ros2 topic info -v` all look completely healthy — correct types, correct
# QoS, correct publisher counts — and `ros2 topic hz` reports nothing on any
# topic at all. Whitelisting the link interface is what fixes it.
#
# Fast DDS 2.6 (Humble) only accepts literal addresses in interfaceWhiteList —
# no CIDR or netmask filters — so the address has to be resolved and baked in
# here.
#
# TWO MODES, because a baked-in address is a liability when the link is down.
#
#   linked — an address in the rover link subnet exists on this machine.
#            Emits the whitelist + static discovery peers described below.
#
#   local  — no rover link. Emits UDP with *no* whitelist and *no* custom peer
#            list, so nodes on this machine (ground station container,
#            rover_dev, Gazebo) discover each other normally.
#
# The mode split is the entire point of this rewrite. The previous version
# resolved the link address with `ip route get 10.42.0.0`, which does not fail
# when the link is down — it follows the *default route* and cheerfully returns
# the internet-side address. So it either baked in the wrong address (exactly
# the failure this file exists to prevent) or, once written, kept a whitelist
# entry for an address that no longer existed. A whitelist naming an absent
# address leaves Fast DDS with no usable interface for SPDP: `ros2 topic list`
# comes back with nothing but /parameter_events and /rosout while every
# container is plainly running and sharing both the host network and /dev/shm.
# Symptom and cause look nothing alike, which is why it burned so much time.
#
# Notes on the 'linked' profile, all of them load-bearing:
#
#   127.0.0.1 is whitelisted alongside the link address so the multicast
#   locator works locally. Without it the ground station's own nodes are
#   invisible to each other: topic data still flows over SHM, but `ros2 node
#   list`, `ros2 service list` and the UI's calibration service calls all come
#   back empty.
#
#   initialPeersList adds a unicast peer for the far end, because the mast Pi
#   is an L3 router between 10.44.0.0/24 and 10.42.0.0/24, and routers do not
#   forward link-local multicast — so SPDP finds nothing at all.
#
#   SPDP is symmetric *per participant*, not per host. Every ROS 2 node is its
#   own DDS participant, and a participant we never probe never hears from us:
#   its own announcements go out over multicast, which the mast Pi does not
#   forward. So one peer entry is not enough — the far end needs one per
#   participant ID we intend to find.
#
#   RUN IT AT BOTH ENDS. One-sided config half-works, which is why this was
#   easy to miss: an SPDP announcement carries the sender's own unicast
#   locators, so a rover that is probed can answer the ground station directly
#   without ever being configured. What it cannot do is speak first. A node
#   that starts on an unprobed end is found only on the *other* end's next
#   announcement round, never at its own startup, and a participant whose ID
#   falls outside PEER_RANGE is never found at all. Configuring only one side
#   also leaves the other announcing locators for every interface it happens to
#   have — the exact failure described at the top of this file, pointed the
#   other way.
#
#   Those entries are written with EXPLICIT PORTS, one locator per participant
#   ID, and that detail is the whole reason this script was rewritten a second
#   time. The obvious spelling is a single portless entry plus
#   maxInitialPeersRange=50, letting Fast DDS fan out across IDs 0..49 — and
#   that spelling quietly destroys discovery on the local machine. Measured on
#   this setup, holding everything else constant:
#
#     portless peer, maxInitialPeersRange=50   -> local discovery DEAD
#     portless peer, maxInitialPeersRange=4    -> local discovery OK
#     peer with an explicit port, range=50     -> local discovery OK
#     multicast peer alone, no unicast peer    -> local discovery OK
#
#   A participant could not see a publisher in its *own container* under the
#   first one. Reachability of the peer was not the variable — substituting
#   127.0.0.1 for the rover address failed exactly the same way. It is the
#   portless fan-out itself, so the fan-out is done here instead, where it is
#   explicit and bounded, and maxInitialPeersRange is left at its default.
#
#   Unicast metatraffic port for participant N in domain D is
#   PB + DG*D + d1 + PG*N = 7400 + 250*D + 10 + 2*N.
#
#   The SPDP multicast locator is repeated explicitly, and that is not
#   redundant: Fast DDS puts it in initialPeersList by default, but defining
#   the element *replaces* that default rather than adding to it. Listing only
#   the rover silently breaks discovery between nodes on this machine.
#
#   Shared memory is kept as a user transport so the ground station container
#   talks to a same-host rover_dev/Gazebo container over SHM.
#
# Usage:
#   ./docker/gen-dds-profile.sh                    # autodetect
#   ROVER_LINK_IP=10.44.0.10 ./docker/gen-dds-profile.sh   # force linked mode
#   ROVER_LINK_PREFIX=10.44.0. ./docker/gen-dds-profile.sh
#   ROVER_PEER=10.42.0.1 ./docker/gen-dds-profile.sh
#   DDS_PROFILE_OUT=/opt/dds/fastdds_rover_link.xml ./docker/gen-dds-profile.sh
#   LINK_WAIT_SECS=60 ./docker/gen-dds-profile.sh  # wait for the link first
#
# The defaults below are the ground station's. The rover passes its own through
# the environment in docker/docker-compose.prod.yaml; nothing here is defaulted
# to rover values, so a missing override shows up as the wrong peer rather than
# as a script that silently probes itself.
#
# Run automatically by docker/entrypoint.bash on every container start, so the
# profile cannot outlive the topology it describes. Re-run it by hand (or just
# `docker compose restart indomitus_ground_station`) if the link adapter is
# plugged in or unplugged while the container is up — a DDS participant reads
# this file once, when it is created.
set -euo pipefail

# Address prefix that identifies the rover link. Matched literally against the
# machine's own addresses; nothing is inferred from routes.
LINK_PREFIX="${ROVER_LINK_PREFIX:-10.44.0.}"
# The far end's address across the link — where SPDP announcements are sent now
# that multicast cannot reach it. The rover from the ground station; the ground
# station from the rover.
PEER="${ROVER_PEER:-10.42.0.1}"
# How many participant IDs to probe at the far end. Must exceed the number of
# ROS 2 nodes running there (one participant each), with headroom for the ros2
# CLI and daemon, which take IDs of their own. Each one becomes its own
# explicitly-ported locator below.
#
# A participant landing outside this range is invisible with no fallback: its
# own multicast announcements do not cross the mast Pi, and we never probe its
# port. That is a silent, per-node failure, so leave headroom — the only cost
# of a larger range is a longer file and a slightly wider announcement burst.
PEER_RANGE="${ROVER_PEER_RANGE:-50}"
DOMAIN="${ROS_DOMAIN_ID:-90}"
# Seconds to wait for a link address to appear before settling on a mode.
#
# 0 (the default) decides immediately, which is right on the ground station:
# the container is started by hand, after the link is up.
#
# The rover is the opposite case. Its container is `restart: unless-stopped`
# and comes back on boot, racing rover-ap.service, which is what puts 10.42.0.1
# on wlan0 — so a rover that decided immediately would very often decide
# 'local', come up with no peer list, and never probe the ground station at all
# until someone restarted it by hand. Waiting turns a boot-order race into a
# few seconds of startup delay.
LINK_WAIT_SECS="${LINK_WAIT_SECS:-0}"
# Extra local addresses to whitelist alongside the rover link, space or comma
# separated. Empty by default: the whitelist exists to stop Fast DDS announcing
# locators the rover cannot route to, so widening it is always a deliberate act.
#
# Use it when a second segment must also carry ROS traffic — e.g. a Jetson on
# the campus LAN that the rover path does not reach:
#
#   DDS_EXTRA_ADDRS=10.20.18.46 ./docker/gen-dds-profile.sh
#
# Cost of adding one: the GS then announces locators on that interface too, and
# the rover will try and fail them before falling back to the link address.
# Discovery still completes, but it is slower and noisier — so list only
# addresses that a peer you actually care about can reach.
EXTRA_ADDRS="${DDS_EXTRA_ADDRS:-}"
# Where to write the profile. Defaults to sitting next to this script, which is
# what the ground station wants: the repo is bind-mounted at /work, so the file
# lands in the checkout and is gitignored there.
#
# The rover has no checkout mounted — prod bind-mounts only src/ — so this
# script ships inside the image and the profile is written to /opt/dds instead.
# Setting this is also what arms generation in the rover's entrypoint, so the
# one case that must NOT regenerate (docker-compose.dev.gs.yaml, where the
# rover container reads the ground station's profile from a read-only mount)
# simply leaves it unset.
OUT="${DDS_PROFILE_OUT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fastdds_rover_link.xml}"

# RTPS well-known port for SPDP multicast: PB + DG*domainId + d0,
# with PB=7400, DG=250, d0=0. The port must be spelled out because an initial
# peer with no port makes Fast DDS fan out across participant IDs 0..4, which
# is meaningless for a multicast discovery address.
MCAST_ADDR=239.255.0.1
MCAST_PORT=$((7400 + 250 * DOMAIN))

# One explicit locator per rover participant ID. See the header for why these
# are not left to maxInitialPeersRange.
rover_peer_locators() {
    local id port
    for (( id = 0; id < PEER_RANGE; id++ )); do
        port=$((7400 + 250 * DOMAIN + 10 + 2 * id))
        cat <<LOC
            <locator>
              <udpv4>
                <address>${PEER}</address>
                <port>${port}</port>
              </udpv4>
            </locator>
LOC
    done
}

# Renders DDS_EXTRA_ADDRS as whitelist entries. Accepts commas or spaces, drops
# blanks, and emits nothing at all when the variable is empty — an empty
# <address></address> element is not ignored by Fast DDS, it matches no
# interface and takes the whole transport down with it.
extra_addr_elements() {
    local a
    for a in ${EXTRA_ADDRS//,/ }; do
        [ -n "$a" ] || continue
        printf '          <address>%s</address>\n' "$a"
    done
}

# Enumerating the machine's own IPv4 addresses is the one thing this script
# cannot get wrong. `ip` is the good tool, but it lives in iproute2, which the
# ROS base image does not carry — and inside the container an unguarded `ip`
# call would fail, yield no addresses, and silently select 'local' mode *while
# the rover link was up*. That is the original bug wearing a different hat, so
# the tool is chosen explicitly and its absence is a hard error, never an empty
# result. `hostname -I` is the fallback: it prints exactly the global IPv4
# addresses, space separated, and it is present in the image today.
ADDR_TOOL=""
if command -v ip >/dev/null 2>&1; then
    ADDR_TOOL=ip
elif command -v hostname >/dev/null 2>&1 && hostname -I >/dev/null 2>&1; then
    ADDR_TOOL=hostname
fi

live_addrs() {
    case "$ADDR_TOOL" in
        ip)       ip -o -4 addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1 ;;
        hostname) hostname -I 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9]+(\.[0-9]+){3}$' ;;
    esac
}

if [ -z "$ADDR_TOOL" ]; then
    echo "error: cannot enumerate this machine's IPv4 addresses — neither 'ip'" >&2
    echo "       (iproute2) nor a working 'hostname -I' is available." >&2
    echo "       Refusing to guess: picking a mode blind would either whitelist" >&2
    echo "       nothing while the rover link is up, or whitelist an address" >&2
    echo "       that is gone. Install iproute2, or pass ROVER_LINK_IP=..." >&2
    exit 1
fi

LINK_IP=""
# How the address was chosen, for the generated file's header. Worth spelling
# out: a forced address that does not match ROVER_LINK_PREFIX is legitimate,
# and reporting it as "matched prefix ..." would describe a match that never
# happened.
LINK_IP_SOURCE=""
if [ -n "${ROVER_LINK_IP:-}" ]; then
    LINK_IP="$ROVER_LINK_IP"
    LINK_IP_SOURCE="forced via ROVER_LINK_IP"
    if ! live_addrs | grep -qx "$LINK_IP"; then
        echo "warning: ROVER_LINK_IP=$LINK_IP is not an address on this machine." >&2
        echo "         Writing it anyway, as asked — but if it stays absent, DDS" >&2
        echo "         discovery will come back empty." >&2
    fi
else
    # Poll rather than sample once, so a container that beat the interface up
    # does not settle on 'local' for the rest of the session. With
    # LINK_WAIT_SECS=0 this runs the body exactly once and is identical to the
    # single-sample version it replaced.
    waited=0
    while : ; do
        LINK_IP="$(live_addrs | grep -m1 -- "^${LINK_PREFIX}" || true)"
        [ -n "$LINK_IP" ] && break
        [ "$waited" -ge "$LINK_WAIT_SECS" ] && break
        if [ "$waited" = 0 ]; then
            echo "waiting up to ${LINK_WAIT_SECS}s for an address matching ${LINK_PREFIX}*..."
        fi
        sleep 1
        waited=$((waited + 1))
    done
    if [ -n "$LINK_IP" ] && [ "$waited" -gt 0 ]; then
        echo "link address appeared after ${waited}s"
    fi
    LINK_IP_SOURCE="matched prefix $LINK_PREFIX"
fi

if [ -n "$LINK_IP" ]; then
    MODE=linked
else
    MODE=local
fi

if [ "$MODE" = linked ]; then
cat > "$OUT" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!--
  GENERATED by docker/gen-dds-profile.sh — do not edit by hand.
  Mode:               linked (rover link present)
  Rover link address: $LINK_IP ($LINK_IP_SOURCE)
  Discovery peers:    $MCAST_ADDR:$MCAST_PORT (local, domain $DOMAIN), $PEER (far end)
  Extra whitelisted:  ${EXTRA_ADDRS:-none (set DDS_EXTRA_ADDRS to widen)}
  Peer locators:      participant IDs 0..$((PEER_RANGE - 1)), explicit ports

  Restricts UDP to the rover link so Fast DDS stops announcing locators the far
  end cannot route to, and adds a unicast discovery peer for it because the
  mast Pi routes and multicast SPDP does not cross a router.
  See the header of gen-dds-profile.sh for why each element is load-bearing.
-->
<dds xmlns="http://www.eprosima.com">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>rover_link_udp</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>$LINK_IP</address>
          <!-- Loopback is not optional; see gen-dds-profile.sh. -->
          <address>127.0.0.1</address>
$(extra_addr_elements)
        </interfaceWhiteList>
      </transport_descriptor>
      <transport_descriptor>
        <transport_id>local_shm</transport_id>
        <type>SHM</type>
      </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="rover_link" is_default_profile="true">
      <rtps>
        <builtin>
          <initialPeersList>
            <!-- Keeps node-to-node discovery working on this machine. -->
            <locator>
              <udpv4>
                <address>$MCAST_ADDR</address>
                <port>$MCAST_PORT</port>
              </udpv4>
            </locator>
            <!-- Reaches the rover, which multicast cannot: the mast Pi routes.
                 One locator per participant ID, ports spelled out. -->
$(rover_peer_locators)
          </initialPeersList>
        </builtin>
        <userTransports>
          <transport_id>rover_link_udp</transport_id>
          <transport_id>local_shm</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
EOF
    echo "wrote $OUT (mode: linked, link address: $LINK_IP, discovery peer: $PEER)"
else
cat > "$OUT" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!--
  GENERATED by docker/gen-dds-profile.sh — do not edit by hand.
  Mode: local (no address matching $LINK_PREFIX* on this machine)

  No rover link is up, so there is no second interface to steer away from and
  nothing to whitelist. Deliberately omits both interfaceWhiteList and
  initialPeersList: a whitelist naming an absent address, or a peer list aimed
  at an unreachable rover, leaves Fast DDS with no working SPDP path and makes
  same-host containers invisible to each other.

  UDP and SHM are still declared explicitly rather than falling back to the
  builtin transports, so that this file stays a valid profile target for
  FASTRTPS_DEFAULT_PROFILES_FILE and the two modes differ only in the parts
  that describe the link.

  Re-run gen-dds-profile.sh once the rover link adapter is up to switch back to
  linked mode.
-->
<dds xmlns="http://www.eprosima.com">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>rover_link_udp</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
      <transport_descriptor>
        <transport_id>local_shm</transport_id>
        <type>SHM</type>
      </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="rover_link" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>rover_link_udp</transport_id>
          <transport_id>local_shm</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
EOF
    echo "wrote $OUT (mode: local — no rover link found matching ${LINK_PREFIX}*)"
    echo "  local nodes and same-host containers will discover each other;"
    echo "  re-run this once the rover link adapter is up."
fi
