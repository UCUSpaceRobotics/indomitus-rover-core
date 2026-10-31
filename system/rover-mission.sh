#!/bin/bash
# rover-mission - start/stop rover subsystems (rover-launch-*.service) per mission.
# Installed to /usr/local/bin/rover-mission by setup.sh; from a laptop: scripts/mission.sh.
#
# Subsystems = installed rover-launch-<subsystem>.service units.
# Missions   = /etc/rover/missions/<mission>.conf (system/missions/ in the repo):
#              one "subsystem [extra launch args]" per line, in start order.
set -euo pipefail

MISSION_DIR=/etc/rover/missions
RUN_DIR=/run/rover   # until reboot (apply)
ETC_DIR=/etc/rover   # every boot (apply --default)
SESSION=/usr/local/bin/rover-session

MISSION_SUBS=()            # load_mission: subsystems in start order
declare -A MISSION_ARGS=() # load_mission: subsystem -> extra launch args

die() { echo -e "\e[31m[ERROR]\e[0m $*" >&2; exit 1; }
unit() { echo "rover-launch-$1.service"; }

subsystems() {
    systemctl list-unit-files --no-legend 'rover-launch-*.service' \
        | awk '{ sub(/^rover-launch-/, "", $1); sub(/\.service$/, "", $1); print $1 }'
}
missions() {
    local f
    for f in "$MISSION_DIR"/*.conf; do [[ -e $f ]] && basename "$f" .conf; done
}
is_subsystem() { subsystems | grep -qx "$1"; }
is_mission() { [[ -f $MISSION_DIR/$1.conf ]]; }
in_mission() { [[ -v MISSION_ARGS[$1] ]]; }

load_mission() {
    local file="$MISSION_DIR/$1.conf" s args
    is_mission "$1" || die "Unknown mission '$1' ($(missions | xargs))"
    MISSION_SUBS=(); MISSION_ARGS=()
    while read -r s args; do
        [[ -z $s || $s == \#* ]] && continue
        args=${args%%#*}; args=${args%"${args##*[![:space:]]}"}   # drop comment, trailing spaces
        is_subsystem "$s" || die "$file: no unit $(unit "$s")"
        MISSION_SUBS+=("$s"); MISSION_ARGS[$s]=$args
    done < "$file"
}

show_help() {
    cat << EOF
Usage: rover-mission COMMAND

  MISSION [--default]     same as: apply MISSION [--default]
  apply MISSION [--default]
                          start the mission's subsystems, stop the rest; running ones
                          with unchanged args are left alone. --default: also at boot
  stop                    stop every subsystem
  restart SUBSYSTEM...    restart these
  status                  mission, and per subsystem: unit state, boot, what runs, args
  list                    missions ($MISSION_DIR) and their subsystems

Watch / Ctrl-C a subsystem: scripts/enter_container.sh rover <subsystem>
EOF
}

# Args the unit runs (or will run) with: /run overrides /etc, as in the unit.
current_env() {
    cat "$RUN_DIR/$1.env" 2>/dev/null || cat "$ETC_DIR/$1.env" 2>/dev/null || echo "LAUNCH_ARGS="
}

# Unit active and its launch actually up - or still waiting for CAN / the container.
# systemd alone can't tell: a restarted container leaves the units "active".
is_running() {
    local u; u=$(unit "$1")
    systemctl is-active -q "$u" || return 1
    [[ $(systemctl show -P SubState "$u") == running ]] && return 0
    local fg; fg=$("$SESSION" status "$1")
    [[ $fg != - && $fg != bash ]]
}

stop_subsystem() {
    if systemctl is-active -q "$(unit "$1")"; then
        echo ">>> stop $1"; systemctl stop "$(unit "$1")"
    fi
    rm -f "$RUN_DIR/$1.env"
}

apply() {
    local mission=$1 default=$2 s
    load_mission "$mission"
    mkdir -p "$RUN_DIR"

    # Stop first: frees CAN / cameras before anything new grabs them.
    for s in $(subsystems | tac); do
        in_mission "$s" || stop_subsystem "$s"
    done

    for s in "${MISSION_SUBS[@]}"; do
        local env="LAUNCH_ARGS=${MISSION_ARGS[$s]}"
        if is_running "$s" && [[ $(current_env "$s") == "$env" ]]; then
            echo ">>> $s already running"; continue
        fi
        echo "$env" > "$RUN_DIR/$s.env"
        echo ">>> start $s ${MISSION_ARGS[$s]}"
        systemctl restart "$(unit "$s")"   # restart: also picks up new args if it was running
    done
    echo "$mission" > "$RUN_DIR/mission"

    if [[ $default == true ]]; then
        mkdir -p "$ETC_DIR"
        for s in $(subsystems); do
            if in_mission "$s"; then
                cp "$RUN_DIR/$s.env" "$ETC_DIR/$s.env"
                systemctl enable -q "$(unit "$s")"
            else
                rm -f "$ETC_DIR/$s.env"
                systemctl disable -q "$(unit "$s")" 2>/dev/null || true
            fi
        done
        echo "$mission" > "$ETC_DIR/mission"
        echo ">>> '$mission' starts at boot"
    fi
}

stop_all() {
    local s
    for s in $(subsystems | tac); do stop_subsystem "$s"; done
    rm -f "$RUN_DIR/mission"
}

status() {
    local now boot s u
    now=$(cat "$RUN_DIR/mission" 2>/dev/null || echo "-")
    boot=$(cat "$ETC_DIR/mission" 2>/dev/null || echo "-")
    echo "mission: $now   at boot: $boot"
    printf "%-8s %-10s %-9s %-8s %s\n" SUBSYS UNIT BOOT RUNS ARGS
    for s in $(subsystems); do
        u=$(unit "$s")
        printf "%-8s %-10s %-9s %-8s %s\n" "$s" \
            "$(systemctl is-active "$u" 2>/dev/null || true)" \
            "$(systemctl is-enabled "$u" 2>/dev/null || true)" \
            "$("$SESSION" status "$s")" \
            "$(current_env "$s" | sed 's/^LAUNCH_ARGS=//')"
    done
}

list() {
    local m s line
    for m in $(missions); do
        load_mission "$m"
        line="$m:"
        for s in "${MISSION_SUBS[@]}"; do
            line+=" $s"; [[ -n ${MISSION_ARGS[$s]} ]] && line+="(${MISSION_ARGS[$s]})"
        done
        echo "$line"
    done
}

[[ $# -gt 0 ]] || { show_help; exit 1; }
cmd=$1; shift
case $cmd in
    -h|--help|help) show_help; exit 0;;
    status) status; exit 0;;
    list)   list; exit 0;;
esac

[[ $EUID -eq 0 ]] || exec sudo "$0" "$cmd" "$@"

case $cmd in
    stop)
        stop_all;;
    restart)
        [[ $# -ge 1 ]] || die "restart needs SUBSYSTEM(s) ($(subsystems | xargs))"
        for s in "$@"; do
            is_subsystem "$s" || die "Unknown subsystem '$s' ($(subsystems | xargs))"
        done
        for s in "$@"; do echo ">>> restart $s"; systemctl restart "$(unit "$s")"; done;;
    *)
        if [[ $cmd == apply ]]; then
            [[ $# -ge 1 ]] || die "apply needs a MISSION ($(missions | xargs))"
            cmd=$1; shift
        fi
        is_mission "$cmd" || die "Unknown command or mission '$cmd' (missions: $(missions | xargs); -h for help)"
        default=false
        [[ ${1:-} == --default ]] && { default=true; shift; }
        [[ $# -eq 0 ]] || die "Unexpected: $*"
        apply "$cmd" "$default";;
esac
