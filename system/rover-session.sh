#!/bin/sh
# rover-session - one subsystem's launch in its own tmux session in rover_prod.
# Installed to /usr/local/bin/rover-session by setup.sh; used by rover-launch-*.service
# and by hand in dev (bypasses systemd: `systemctl status` won't know).
#
#   rover-session start SESSION COMMAND...  stop what runs there, then type COMMAND
#   rover-session stop SESSION              Ctrl-C, wait for the shell prompt
#   rover-session status SESSION            foreground command ("bash" = idle, "-" = no session)
#
# Env: ROVER_CONTAINER (rover_prod), CAN_IFACE (start waits until it is UP),
#      ROVER_LOG_DIR (/bags/logs, path inside the container)
set -u

C="${ROVER_CONTAINER:-rover_prod}"
LOG_DIR="${ROVER_LOG_DIR:-/bags/logs}"
IDLE=bash        # docker/tmux.conf: default-shell /bin/bash
STOP_TICKS=60    # 0.5 s each; ros2 launch escalates SIGINT -> SIGTERM -> SIGKILL itself within ~10 s

t() { docker exec "$C" tmux "$@"; }
fg_cmd() { t display -p -t "$1" '#{pane_current_command}' 2>/dev/null || echo -; }

stop_session() {
    t has-session -t "$1" 2>/dev/null || return 0
    [ "$(fg_cmd "$1")" = "$IDLE" ] && return 0
    t send-keys -t "$1" C-c
    i=0
    while [ "$i" -lt "$STOP_TICKS" ]; do
        [ "$(fg_cmd "$1")" = "$IDLE" ] && return 0
        sleep 0.5; i=$((i + 1))
    done
    # Last resort: SIGHUP kills ros2 launch without it stopping its nodes, so
    # leftovers may still hold CAN / cameras - check `ps` in the container.
    echo "[rover-session] '$1' did not stop in $((STOP_TICKS / 2)) s - killing the session" >&2
    t kill-session -t "$1"
}

start_session() {
    s="$1"; shift
    if [ -n "${CAN_IFACE:-}" ]; then
        until ip -o link show dev "$CAN_IFACE" 2>/dev/null | grep -q "[<,]UP[,>]"; do sleep 2; done
    fi
    until docker exec "$C" true 2>/dev/null; do sleep 2; done

    stop_session "$s"    # leftovers of a previous run / something started by hand
    t has-session -t "$s" 2>/dev/null || t new-session -d -s "$s"

    # One log per start. The Jetson has no RTC, so the date repeats across
    # boots - the boot id keeps the names apart.
    log="$LOG_DIR/${s}_$(date +%F_%H%M%S)_$(cut -c1-8 /proc/sys/kernel/random/boot_id).log"
    docker exec "$C" mkdir -p "$LOG_DIR"
    t pipe-pane -t "$s"
    t pipe-pane -t "$s" "cat >> '$log'"

    t send-keys -t "$s" "$*" Enter
}

case "${1:-}" in
    start)  [ "$#" -ge 3 ] || { echo "usage: $0 start SESSION COMMAND..." >&2; exit 2; }
            shift; start_session "$@";;
    stop)   [ "$#" -eq 2 ] || { echo "usage: $0 stop SESSION" >&2; exit 2; }
            docker exec "$C" true 2>/dev/null || exit 0   # no container, nothing runs
            stop_session "$2";;
    status) [ "$#" -eq 2 ] || { echo "usage: $0 status SESSION" >&2; exit 2; }
            fg_cmd "$2";;
    *)      echo "usage: $0 {start SESSION COMMAND...|stop SESSION|status SESSION}" >&2; exit 2;;
esac
