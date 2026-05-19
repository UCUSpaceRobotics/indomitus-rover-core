#!/bin/bash

# --- DEFAULT CONFIGURATION ---
JETSON_USER="ros"
JETSON_IP="10.42.0.1"
REMOTE_DIR="/home/ros/Indomitus/indomitus-rover-core"
LOCAL_DIR="./"

# --- HELP MENU ---
show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Safely synchronizes a local Git repository to a Jetson Nano over an SSH connection.
It performs safety checks to ensure both local and remote directories are the 
exact same Git repository before applying any changes or deletions via rsync.

Options:
  -l, --local-dir   Path to the local Git repository on your machine. 
                    (Default: ./)
  -i, --ip          The IP address of the Jetson Nano hotspot/network. 
                    (Default: 10.42.0.1)
  -u, --user        The SSH username for the Jetson Nano. 
                    (Default: jetson_username)
  -d, --remote-dir  The absolute destination path to the repository on the Jetson Nano. 
                    (Default: /home/jetson_username/repository)
  -h, --help        Display this help message and exit.
EOF
}

for arg in "$@"; do
    if [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
        show_help
        exit 0
    fi
done

# --- PARSE TERMINAL ARGUMENTS ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        -u|--user) JETSON_USER="$2"; shift 2;;
        -i|--ip) JETSON_IP="$2"; shift 2;;
        -d|--remote-dir) REMOTE_DIR="$2"; shift 2;;
        -l|--local-dir) LOCAL_DIR="$2"; shift 2;;
        *) show_help; exit 1;;
    esac
done

TARGET="${JETSON_USER}@${JETSON_IP}"

# Navigate to the local directory
cd "${LOCAL_DIR}" || { echo "Fail: Could not access local directory ${LOCAL_DIR}"; exit 1; }

echo "Syncing to: ${TARGET}:${REMOTE_DIR}"
echo "Verifying repositories..."

# 1. Local Check
if ! git rev-parse --is-inside-work-tree > /dev/null 2>&1; then
    echo "Fail: Local path is not a Git repo."
    exit 1
fi

# 2. Remote Check
if ! ssh -q ${TARGET} "cd ${REMOTE_DIR} && git rev-parse --is-inside-work-tree > /dev/null 2>&1"; then
    echo "Fail: Remote path is not a Git repo or is unreachable."
    exit 1
fi

# 3. Lineage Check (Same Project)
LOCAL_ROOT=$(git rev-list --max-parents=0 HEAD 2>/dev/null | tail -n 1)
REMOTE_ROOT=$(ssh -q ${TARGET} "cd ${REMOTE_DIR} && git rev-list --max-parents=0 HEAD 2>/dev/null | tail -n 1")

if [ -z "$LOCAL_ROOT" ] || [ "$LOCAL_ROOT" != "$REMOTE_ROOT" ]; then
    echo "Fail: Repository mismatch. Aborting to protect data."
    exit 1
fi

echo "Checks passed."
echo "-----------------------------------"
echo "Dry Run Preview (Changes to make):"

# 4. Dry-Run
rsync -avz --delete --dry-run ./ ${TARGET}:${REMOTE_DIR}/

echo "-----------------------------------"

# 5. Final Confirmation
read -p "Apply these changes? (y/n): " confirm

if [[ "$confirm" =~ ^[Yy]$ ]]; then
    rsync -avz --delete ./ ${TARGET}:${REMOTE_DIR}/
    echo "Sync complete."
else
    echo "Aborted."
fi