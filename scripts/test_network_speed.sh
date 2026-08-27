#!/bin/bash
# Rover AP throughput test.
#
# Runs iperf3 in both directions against the rover AP while recording radio
# statistics and kernel messages, so a throughput drop can be correlated with
# retries, signal loss or USB/firmware errors.
#
# Run this on the CLIENT (laptop / mast Pi), not on the rover.
# On the rover, start the server first:   iperf3 -s -B 10.42.0.1
#
# Usage:
#   ./ap-speedtest.sh                 # default 60s per direction
#   ./ap-speedtest.sh -t 300          # 5 min per direction (recommended)
#   ./ap-speedtest.sh -t 300 -u       # also run a UDP loss/jitter test
#   ./ap-speedtest.sh -H rover        # ssh host for rover-side stats

set -uo pipefail

SERVER=10.42.0.1          # rover AP address
ROVER_SSH=rover           # ssh alias for rover-side stats; empty = skip
AP_IF=wlx00c0caba86c1     # rover AP interface
DURATION=60
PARALLEL=4
DO_UDP=0
UDP_RATE=200M
OUTDIR="$HOME/ap-speedtest-$(date +%Y%m%d-%H%M%S)"

usage() {
    sed -n '2,18p' "$0" | sed 's/^# \?//'
    exit 0
}

while getopts "s:t:P:H:i:ub:h" opt; do
    case $opt in
        s) SERVER=$OPTARG ;;
        t) DURATION=$OPTARG ;;
        P) PARALLEL=$OPTARG ;;
        H) ROVER_SSH=$OPTARG ;;
        i) AP_IF=$OPTARG ;;
        u) DO_UDP=1 ;;
        b) UDP_RATE=$OPTARG ;;
        h) usage ;;
        *) usage ;;
    esac
done

command -v iperf3 >/dev/null || { echo "iperf3 not installed"; exit 1; }

mkdir -p "$OUTDIR"
echo "Results -> $OUTDIR"
echo

# --- reachability -----------------------------------------------------------
if ! ping -c2 -W2 "$SERVER" >/dev/null 2>&1; then
    echo "ERROR: cannot reach $SERVER. Is the AP up and are you associated?"
    exit 1
fi

if ! iperf3 -c "$SERVER" -t 1 >/dev/null 2>&1; then
    echo "ERROR: no iperf3 server on $SERVER."
    echo "On the rover run:  iperf3 -s -B $SERVER"
    exit 1
fi

# --- rover-side background collectors --------------------------------------
# Kept as PIDs so they can be stopped in the trap below.
PIDS=()
cleanup() {
    for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

if [ -n "$ROVER_SSH" ]; then
    if ssh -o BatchMode=yes -o ConnectTimeout=5 "$ROVER_SSH" true 2>/dev/null; then
        echo "Collecting rover-side stats via ssh:$ROVER_SSH"

        ssh "$ROVER_SSH" "sudo dmesg -Tw" \
            > "$OUTDIR/rover-dmesg.log" 2>/dev/null &
        PIDS+=($!)

        ssh "$ROVER_SSH" "while :; do
                echo \"--- \$(date -Is)\";
                sudo iw dev $AP_IF station dump |
                  grep -E 'Station|signal:|tx bitrate|rx bitrate|tx retries|tx failed|inactive time';
                sleep 2;
            done" > "$OUTDIR/rover-station.log" 2>/dev/null &
        PIDS+=($!)
    else
        echo "WARNING: ssh to '$ROVER_SSH' failed - rover-side stats will be missing."
        ROVER_SSH=""
    fi
fi

# --- context ----------------------------------------------------------------
{
    echo "date:       $(date -Is)"
    echo "server:     $SERVER"
    echo "duration:   ${DURATION}s per direction"
    echo "parallel:   $PARALLEL"
    [ -n "$ROVER_SSH" ] && {
        echo "--- rover link ---"
        ssh "$ROVER_SSH" "iw dev $AP_IF info; iw dev $AP_IF link" 2>/dev/null
        echo "--- rover driver ---"
        ssh "$ROVER_SSH" "basename \"\$(readlink -f /sys/class/net/$AP_IF/device/driver)\"; lsmod | grep -Ei 'rtw|8812|88XXau'" 2>/dev/null
    }
} > "$OUTDIR/context.txt" 2>&1

# --- tests ------------------------------------------------------------------
run_test() {
    local label=$1 file=$2; shift 2
    echo "== $label (${DURATION}s) =="
    iperf3 -c "$SERVER" -t "$DURATION" -i 5 "$@" \
        | tee "$OUTDIR/$file.txt" \
        | grep -E 'sender|receiver|SUM|Jitter|lost'
    echo
    sleep 5   # let the radio settle between runs
}

run_test "TCP  client -> rover (upload)"   tcp-up     -P "$PARALLEL"
run_test "TCP  rover -> client (download)" tcp-down   -P "$PARALLEL" -R

if [ "$DO_UDP" -eq 1 ]; then
    run_test "UDP  client -> rover" udp-up   -u -b "$UDP_RATE"
    run_test "UDP  rover -> client" udp-down -u -b "$UDP_RATE" -R
fi

cleanup

# --- summary ----------------------------------------------------------------
echo "================ SUMMARY ================"
for f in tcp-up tcp-down udp-up udp-down; do
    [ -f "$OUTDIR/$f.txt" ] || continue
    printf '%-10s ' "$f"
    grep -E 'receiver' "$OUTDIR/$f.txt" | tail -1 | awk '{print $(NF-2), $(NF-1)}'
done

if [ -s "$OUTDIR/rover-dmesg.log" ]; then
    echo
    echo "--- kernel errors during test ---"
    grep -iE 'fail|error|reset|timeout|-71|usb' "$OUTDIR/rover-dmesg.log" \
        | tail -20 || echo "(none)"
fi

if [ -s "$OUTDIR/rover-station.log" ]; then
    echo
    echo "--- retries / failures (first vs last sample) ---"
    grep -E 'tx retries|tx failed' "$OUTDIR/rover-station.log" | head -2
    grep -E 'tx retries|tx failed' "$OUTDIR/rover-station.log" | tail -2
fi

echo
echo "Full results in $OUTDIR"
