#!/bin/bash
# system/setup.sh - Runs ON the targeted system (Local Host or Jetson)
#
#   --udev A,B      rules.d/[NN-]A.rules ...: install, reload, re-trigger matching devices
#   --service A,B   systemd/A.service ...:    install, daemon-reload, stop, disable
#   --all           everything in rules.d/ and systemd/
#   --check         only verify the names exist
set -e

SUDO_PASS="${SUDO_PASS:-}"
run_sudo() {
    if [ -n "$SUDO_PASS" ]; then
        echo "$SUDO_PASS" | sudo -S "$@"
    else
        sudo "$@"
    fi
}

CONFIG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

die() { echo "[ERROR] $*" >&2; exit 1; }
need_file() { [ -f "$1" ] || die "Not found: $2"; echo "$1"; }

RULES=()
UNITS=()
CHECK=false
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --udev)
            for n in ${2//,/ }; do
                RULES+=("$(need_file "$(ls "$CONFIG_DIR"/rules.d/{,[0-9][0-9]-}"$n".rules 2>/dev/null | head -1)" "rules.d/[NN-]$n.rules")")
            done; shift 2;;
        --service)
            for n in ${2//,/ }; do
                UNITS+=("$(need_file "$CONFIG_DIR/systemd/$n.service" "systemd/$n.service")")
            done; shift 2;;
        --all)     RULES+=("$CONFIG_DIR"/rules.d/*.rules); UNITS+=("$CONFIG_DIR"/systemd/*.service); shift;;
        --check)   CHECK=true; shift;;
        *) die "Unknown option: $1";;
    esac
done
[ "${#RULES[@]}" -gt 0 ] || [ "${#UNITS[@]}" -gt 0 ] || die "Nothing selected (--udev A,B  --service A,B  --all)."
[ "$CHECK" = true ] && exit 0

# udev and systemd both live on the host. Inside a container this script writes
# to the container's own /etc, where nothing will ever read it — and every step
# still reports success, so the operator walks away believing the rover is
# configured. Refuse instead; a wrong environment is worse than no run at all.
if [ -f /run/.containerenv ] || [ -f /.dockerenv ]; then
    if [ -n "$ROVER_SETUP_ALLOW_CONTAINER" ]; then
        echo "[WARNING] Running inside a container — ROVER_SETUP_ALLOW_CONTAINER is set, continuing."
    else
        echo "[ERROR] Running inside a container. udev rules and systemd units belong to the host."
        echo "        From distrobox: distrobox-host-exec ./scripts/setup_host.sh local <options>"
        echo "        Set ROVER_SETUP_ALLOW_CONTAINER=1 to override (container-only testing)."
        exit 1
    fi
fi

for rule in "${RULES[@]}"; do
    echo ">>> Installing udev rule $(basename "$rule")..."
    run_sudo install -m 0644 "$rule" /etc/udev/rules.d/
    run_sudo udevadm control --reload-rules
    # Re-emit 'add' only for devices this rule matches (its SUBSYSTEM/KERNEL/ATTR keys),
    # so already-connected devices pick it up without replaying unrelated rules.
    declare -A seen=()
    grep -v '^\s*#' "$rule" | grep -o 'SUBSYSTEM=="[^"]*".*' | while read -r line; do
        subsystem=$(grep -oP '(^|[ ,])SUBSYSTEM=="\K[^"]*' <<< "$line")
        kernel=$(grep -oP '(^|[ ,])KERNEL=="\K[^"]*' <<< "$line" || true)
        mapfile -t attrs < <(grep -oP '(^|[ ,])ATTR\{\K[^}]+\}=="[^"]*' <<< "$line" | sed 's/}=="/=/; s/^/--attr-match=/')
        key="$subsystem|$kernel|${attrs[*]}"; [ -n "${seen[$key]}" ] && continue; seen[$key]=1
        run_sudo udevadm trigger --action=add --subsystem-match="$subsystem" ${kernel:+--sysname-match="$kernel"} "${attrs[@]}"
    done
    run_sudo udevadm settle
done

for unit in "${UNITS[@]}"; do
    name=$(basename "$unit")
    echo ">>> Installing systemd unit $name (left stopped and disabled)..."
    run_sudo install -m 0644 "$unit" /etc/systemd/system/
    run_sudo systemctl daemon-reload
    run_sudo systemctl stop "$name" 2>/dev/null || true
    run_sudo systemctl disable "$name" 2>/dev/null || true
done
echo "[SUCCESS] Done."
