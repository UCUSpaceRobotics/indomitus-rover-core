#!/bin/bash

set -e
set -o pipefail

# PATH RESOLUTION
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
FILTER_FILE="${SCRIPT_DIR}/.rsync-filter"

cd "$REPO_ROOT" || { echo -e "\e[31m[ERROR]\e[0m Failed to navigate to repository root."; exit 1; }

# DEFAULT VARIABLES
JETSON_USER="indomitus-rover"
JETSON_IP="10.42.0.1"
REMOTE_DIR="/home/indomitus-rover/indomitus-rover-core/"
IMAGE_NAME="ghcr.io/ucuspacerobotics/indomitus-rover-core"
IMAGE_TAG=""
CONTAINER_NAME="rover_prod"
DOCKERFILE="docker/Dockerfile"
COMPOSE_FILE="docker/docker-compose.prod.yaml"
WIFI_SSID="IndomitusRover"
WIFI_PASS="12345678"
SYNC_MODE=false
SYNC_SRC_MODE=false
SYNC_COMPOSE_MODE=false
PULL_MODE=false

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Deploys code to the Jetson. 

Modes:
    --sync                      SYNC ALL: Syncs both 'src' and compose file, restarts container, then compiles.
    --sync-src                  SYNC SRC: Syncs ONLY 'src' and auto-compiles inside the running container.
    --sync-docker-compose       SYNC COMPOSE: Syncs ONLY the compose file and restarts the container.
    --pull                      PULL MODE: Laptop pulls image from GHCR, transfers, and loads it.

Options:
    -i, --ip IP                 Jetson IP address (Default: ${JETSON_IP})
    -u, --user USER             Jetson SSH username (Default: ${JETSON_USER})
    -d, --dir DIR               Remote deployment directory on the Jetson. (Default: ${REMOTE_DIR})
    --image-name NAME           Docker image name (Default: ${IMAGE_NAME})
    -t, --tag TAG               Docker image tag (Default: local-prod or develop-prod)
    --container-name            Docker container name (Default: ${CONTAINER_NAME})
    -f, --file FILE             Path to the Dockerfile (Default: ${DOCKERFILE})
    -c, --compose FILE          Path to the Production Compose file (Default: ${COMPOSE_FILE})
    -w, --ssid SSID             Wi-Fi SSID of the Jetson hotspot (Default: ${WIFI_SSID})
    -p, --pass PASS             Wi-Fi password for the Jetson hotspot (Default: ${WIFI_PASS})
    -h, --help                  Display this help message and exit
EOF
}

# HELPER FUNCTIONS
success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
error()   { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step()    { echo -e "\n\e[33m>>> $1\e[0m"; }

spinner() {
    local pid=$1
    local delay=0.1
    local spinstr='|/-\'
    echo -n "  "
    while kill -0 $pid 2>/dev/null; do
        local temp=${spinstr#?}
        printf "\b\b%c " "$spinstr"
        local spinstr=$temp${spinstr%"$temp"}
        sleep $delay
    done
    printf "\b\b \b\b"
}

# PARSE TERMINAL ARGUMENTS
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --sync) SYNC_MODE=true; shift 1;;
        --sync-src) SYNC_SRC_MODE=true; shift 1;;
        --sync-docker-compose) SYNC_COMPOSE_MODE=true; shift 1;;
        --pull) PULL_MODE=true; shift 1;;
        -u|--user) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
        -i|--ip) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
        -d|--dir) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; REMOTE_DIR="$2"; shift 2;;
        --image-name) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; IMAGE_NAME="$2"; shift 2;;
        -t|--tag) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; IMAGE_TAG="$2"; shift 2;;
        --container-name) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; CONTAINER_NAME="$2"; shift 2;;
        -f|--file) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; DOCKERFILE="$2"; shift 2;;
        -c|--compose) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_FILE="$2"; shift 2;;
        -w|--ssid) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
        -p|--pass) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
        -h|--help) show_help; exit 0;;
        *) show_help; exit 1;;
    esac
done

# ENSURE FILTER FILE EXISTS AFTER HELP PARSING
if [ ! -f "$FILTER_FILE" ] && [ "$SYNC_COMPOSE_MODE" = false ]; then 
    error "rsync filter file not found: $FILTER_FILE"
fi

# DYNAMIC TAG LOGIC
if [ -z "$IMAGE_TAG" ]; then
    if [ "$SYNC_MODE" = true ] || [ "$SYNC_SRC_MODE" = true ] || [ "$SYNC_COMPOSE_MODE" = true ] || [ "$PULL_MODE" = true ]; then
        IMAGE_TAG="develop-prod"
    else
        IMAGE_TAG="local-prod"
    fi
fi

TARGET="${JETSON_USER}@${JETSON_IP}"
ARCHIVE_NAME="deploy_temp_archive.tar"

# SSH MULTIPLEXING FLAGS
SSH_OPTS=(
    -o ControlMaster=auto
    -o "ControlPath=/tmp/ssh_mux_%h_%p_%r"
    -o ControlPersist=10m
    -o StrictHostKeyChecking=accept-new
)

# GLOBAL CLEANUP IN CASE OF FAILURES
CLEANUP_FILES=()

cleanup() {
    rm -rf "${CLEANUP_FILES[@]}"
    ssh -q -O exit "${SSH_OPTS[@]}" "${TARGET}" 2>/dev/null || true
}

trap cleanup EXIT
CLEANUP_FILES+=("${ARCHIVE_NAME}")


# ==========================================
# CORE FUNCTIONS
# ==========================================

connect_to_jetson() {
    step "Verifying Jetson Connection..."

    if [ -n "$WIFI_SSID" ]; then
        echo "Attempting to automatically connect to Wi-Fi network: ${WIFI_SSID}..."
        if command -v nmcli >/dev/null 2>&1; then
            if [ -n "$WIFI_PASS" ]; then
                nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASS" >/dev/null 2>&1 || true
            else
                nmcli device wifi connect "$WIFI_SSID" >/dev/null 2>&1 || true
            fi
        elif command -v networksetup >/dev/null 2>&1; then
            local WIFI_IFACE
            WIFI_IFACE=$(networksetup -listallhardwareports | awk '/Hardware Port: Wi-Fi/{getline; print $2}')
            if [ -n "$WIFI_IFACE" ]; then
                if [ -n "$WIFI_PASS" ]; then
                    networksetup -setairportnetwork "$WIFI_IFACE" "$WIFI_SSID" "$WIFI_PASS" >/dev/null 2>&1 || true
                else
                    networksetup -setairportnetwork "$WIFI_IFACE" "$WIFI_SSID" >/dev/null 2>&1 || true
                fi
            fi
        else
            echo -e "\e[33m[WARNING]\e[0m OS not supported for auto-connect. Please switch to '${WIFI_SSID}' manually."
        fi
    fi

    echo -n "Waiting for SSH connection to ${TARGET}..."
    local MAX_RETRIES=15
    local RETRY_COUNT=0
    while ! ssh -q -o BatchMode=yes -o ConnectTimeout=2 "${SSH_OPTS[@]}" "${TARGET}" "echo 'SSH Ready'" > /dev/null 2>&1; do
        sleep 2
        echo -n "."
        RETRY_COUNT=$((RETRY_COUNT+1))
        if [ "$RETRY_COUNT" -ge "$MAX_RETRIES" ]; then
            echo ""
            error "Timeout: Could not connect to Jetson at ${TARGET}."
        fi
    done
    echo ""
    success "Connection established."
    ssh -q "${SSH_OPTS[@]}" "${TARGET}" "mkdir -p -- \"${REMOTE_DIR}\""
}

ensure_container_running() {
    if ! ssh -q "${SSH_OPTS[@]}" "${TARGET}" "docker inspect -f '{{.State.Running}}' ${CONTAINER_NAME} 2>/dev/null | grep -q true"; then
        error "Container '${CONTAINER_NAME}' is not running on the Jetson. Cannot compile."
    fi
}

run_sync_all_mode() {
    connect_to_jetson
    
    step "SYNC ALL: Transferring local 'src' and compose file..."
    if [ ! -d "src" ]; then error "No 'src' directory found in the repository root."; fi
    if [ ! -f "$COMPOSE_FILE" ]; then error "Local compose file not found: $COMPOSE_FILE"; fi
    
    rsync -avz --delete \
        --filter="merge ${FILTER_FILE}" \
        --info=progress2 \
        -e "ssh -q ${SSH_OPTS[*]}" \
        src/ "${TARGET}:${REMOTE_DIR}/src/"
    rsync -az --info=progress2 -e "ssh -q ${SSH_OPTS[*]}" "${COMPOSE_FILE}" "${TARGET}:${REMOTE_DIR}/docker-compose.yaml"
    
    step "Restarting Container with new Compose config (waiting for readiness)..."
    ssh -q "${SSH_OPTS[@]}" "${TARGET}" "cd \"${REMOTE_DIR}\" && \
        IMAGE_NAME=\"${IMAGE_NAME}\" IMAGE_TAG=\"${IMAGE_TAG}\" docker compose down && \
        IMAGE_NAME=\"${IMAGE_NAME}\" IMAGE_TAG=\"${IMAGE_TAG}\" docker compose up -d --wait"
    
    step "Compiling code on Jetson (Inside Docker)..."
    echo -n "Triggering colcon build inside '${CONTAINER_NAME}'..."
    echo ""
    if ssh -q "${SSH_OPTS[@]}" "${TARGET}" "docker exec ${CONTAINER_NAME} bash -c 'source /opt/ros/\$ROS_DISTRO/setup.bash && cd /opt/ws && colcon build --symlink-install'"; then
        success "Code successfully compiled on the Jetson!"
    else
        echo -e "\e[31m[ERROR]\e[0m Compilation failed."
        exit 1
    fi
    
    echo -e "\n\e[32m[DONE]\e[0m Full Sync Complete!"
    exit 0
}


run_sync_src_mode() {
    connect_to_jetson
    
    step "SYNC SRC: Syncing local 'src' directory via rsync..."
    if [ ! -d "src" ]; then error "No 'src' directory found in the repository root."; fi
    
    rsync -avz --delete --info=progress2 \
        --filter="merge ${FILTER_FILE}" \
        -e "ssh -q ${SSH_OPTS[*]}" \
        src/ "${TARGET}:${REMOTE_DIR}/src/"
    
    step "Compiling code on Jetson (Inside Docker container)..."
    ensure_container_running
    
    echo -n "Triggering colcon build inside '${CONTAINER_NAME}'..."
    echo ""
    if ssh -q "${SSH_OPTS[@]}" "${TARGET}" "docker exec ${CONTAINER_NAME} bash -c 'source /opt/ros/\$ROS_DISTRO/setup.bash && cd /opt/ws && colcon build --symlink-install'"; then
        success "Code successfully compiled on the Jetson!"
    else
        echo -e "\e[31m[ERROR]\e[0m Compilation failed."
        exit 1
    fi
    
    echo -e "\n\e[32m[DONE]\e[0m Source Sync Complete!"
    exit 0
}


run_sync_compose_mode() {
    connect_to_jetson
    
    step "SYNC COMPOSE: Transferring local compose file..."
    if [ ! -f "$COMPOSE_FILE" ]; then error "Local compose file not found: $COMPOSE_FILE"; fi
    
    rsync -az --info=progress2 -e "ssh -q ${SSH_OPTS[*]}" "${COMPOSE_FILE}" "${TARGET}:${REMOTE_DIR}/docker-compose.yaml"
    
    step "Restarting Container on Jetson (waiting for readiness)..."
    ssh -q "${SSH_OPTS[@]}" "${TARGET}" "cd \"${REMOTE_DIR}\" && \
        IMAGE_NAME=\"${IMAGE_NAME}\" IMAGE_TAG=\"${IMAGE_TAG}\" docker compose down && \
        IMAGE_NAME=\"${IMAGE_NAME}\" IMAGE_TAG=\"${IMAGE_TAG}\" docker compose up -d --wait"
    
    echo -e "\n\e[32m[DONE]\e[0m Compose Sync Complete!"
    exit 0
}


run_pull_mode() {
    step "PULL MODE: Fetching clean infrastructure from GitHub..."
    
    local TEMP_REPO_DIR
    TEMP_REPO_DIR=$(mktemp -d)

    CLEANUP_FILES+=("$TEMP_REPO_DIR")

    if [[ "$IMAGE_TAG" != *-prod ]]; then
        echo -e "\e[33m[WARNING]\e[0m Tag '${IMAGE_TAG}' has no '-prod' suffix; using it as git ref directly."
    fi

    local GIT_REF=${IMAGE_TAG%-prod} 
    local REPO_URL="https://github.com/ucuspacerobotics/indomitus-rover-core.git"

    echo "Cloning branch/tag '${GIT_REF}' to temporary directory..."
    if git clone --depth 1 --branch "$GIT_REF" "$REPO_URL" "$TEMP_REPO_DIR" > /dev/null 2>&1; then
        success "Clean source code successfully downloaded to laptop."
    else
        error "Failed to clone repository. Check your internet connection or if the tag '${GIT_REF}' exists."
    fi

    step "Pulling ${IMAGE_NAME}:${IMAGE_TAG} (linux/arm64) to local machine..."
    docker rmi -f "${IMAGE_NAME}:${IMAGE_TAG}" >/dev/null 2>&1 || true

    local PYTHON_SCRIPT="
import sys, json
manifests = json.load(sys.stdin).get('manifests', [])
for m in manifests:
    p = m.get('platform', {})
    if p.get('os') == 'linux' and p.get('architecture') == 'arm64':
        print(m['digest'])
        break
"

    local ARM64_DIGEST
    ARM64_DIGEST=$(docker manifest inspect "${IMAGE_NAME}:${IMAGE_TAG}" | python3 -c "$PYTHON_SCRIPT")

    [ -z "$ARM64_DIGEST" ] && error "Could not resolve arm64 digest for ${IMAGE_NAME}:${IMAGE_TAG}"
    
    docker pull "${IMAGE_NAME}@${ARM64_DIGEST}" || error "Docker pull failed."
    docker tag "${IMAGE_NAME}@${ARM64_DIGEST}" "${IMAGE_NAME}:${IMAGE_TAG}"

    step "Exporting Image to ${ARCHIVE_NAME}..."
    echo -n "Exporting archive..."
    (docker save -o "${ARCHIVE_NAME}" "${IMAGE_NAME}:${IMAGE_TAG}") &
    pid=$!
    spinner $pid
    wait $pid || error "Failed to export the Docker image."
    echo ""

    connect_to_jetson

    step "Transferring Clean Payload to ${TARGET}:${REMOTE_DIR}..."
    rsync -avz --delete --info=progress2 \
        --filter="merge ${FILTER_FILE}" \
        -e "ssh -q ${SSH_OPTS[*]}" \
        "${TEMP_REPO_DIR}/src/" "${TARGET}:${REMOTE_DIR}/src/"
    rsync -az --info=progress2 -e "ssh -q ${SSH_OPTS[*]}" "${TEMP_REPO_DIR}/docker/docker-compose.prod.yaml" "${TARGET}:${REMOTE_DIR}/docker-compose.yaml"
    rsync -az --info=progress2 -e "ssh -q ${SSH_OPTS[*]}" "${ARCHIVE_NAME}" "${TARGET}:${REMOTE_DIR}/"

    step "Loading Image on Jetson..."
    ssh -q "${SSH_OPTS[@]}" "${TARGET}" "cd \"${REMOTE_DIR}\" && docker load -i \"${ARCHIVE_NAME}\" && rm \"${ARCHIVE_NAME}\""

    step "Pruning dangling images on Jetson..."
    ssh -q "${SSH_OPTS[@]}" "${TARGET}" "docker image prune -f"

    step "Restarting Container on Jetson (waiting for readiness)..."
    ssh -q "${SSH_OPTS[@]}" "${TARGET}" "cd \"${REMOTE_DIR}\" && \
        IMAGE_NAME=\"${IMAGE_NAME}\" IMAGE_TAG=\"${IMAGE_TAG}\" docker compose down && \
        IMAGE_NAME=\"${IMAGE_NAME}\" IMAGE_TAG=\"${IMAGE_TAG}\" docker compose up -d --wait"
    
    echo -e "\n\e[32m[DONE]\e[0m Pull & Deploy Complete!"
    exit 0
}


run_full_deploy_mode() {
    step "Running Pre-Flight Checks for Full Build..."
    if ! docker info > /dev/null 2>&1; then error "Docker is not running."; fi
    if ! docker buildx version > /dev/null 2>&1; then error "Docker Buildx is missing."; fi
    if [ ! -f "$DOCKERFILE" ]; then error "Dockerfile not found: $DOCKERFILE"; fi
    if [ ! -f "$COMPOSE_FILE" ]; then error "Compose file not found: $COMPOSE_FILE"; fi

    step "Ensuring ARM64 Cross-Compilation Support (QEMU)..."
    echo -n "Checking/downloading emulators..."
    (docker run --rm --privileged multiarch/qemu-user-static --reset -p yes > /dev/null 2>&1) &
    pid=$!
    spinner $pid
    if wait $pid; then echo "" && success "QEMU emulators configured."; else echo "" && echo -e "\e[33m[WARNING]\e[0m QEMU setup failed."; fi

    step "Building ARM64 Image (${IMAGE_NAME}:${IMAGE_TAG})..."
    docker buildx build --platform linux/arm64 --target prod -t "${IMAGE_NAME}:${IMAGE_TAG}" -f "${DOCKERFILE}" .

    step "Exporting Image to ${ARCHIVE_NAME}..."
    echo -n "Exporting archive..."
    (docker save -o "${ARCHIVE_NAME}" "${IMAGE_NAME}:${IMAGE_TAG}") &
    pid=$!
    spinner $pid
    wait $pid || error "Failed to export the Docker image."
    echo ""

    connect_to_jetson

    step "Transferring Payload to ${TARGET}:${REMOTE_DIR}..."
    if [ ! -d "src" ]; then error "No 'src' directory found in the repository root."; fi
    rsync -avz --delete --info=progress2 \
        --filter="merge ${FILTER_FILE}" \
        -e "ssh -q ${SSH_OPTS[*]}" \
        src/ "${TARGET}:${REMOTE_DIR}/src/"
    rsync -az --info=progress2 -e "ssh -q ${SSH_OPTS[*]}" "${ARCHIVE_NAME}" "${TARGET}:${REMOTE_DIR}/"
    rsync -az --info=progress2 -e "ssh -q ${SSH_OPTS[*]}" "${COMPOSE_FILE}" "${TARGET}:${REMOTE_DIR}/docker-compose.yaml"

    step "Loading Image on Jetson..."
    ssh -q "${SSH_OPTS[@]}" "${TARGET}" "cd \"${REMOTE_DIR}\" && docker load -i \"${ARCHIVE_NAME}\" && rm \"${ARCHIVE_NAME}\""

    step "Pruning dangling images on Jetson..."
    ssh -q "${SSH_OPTS[@]}" "${TARGET}" "docker image prune -f"

    step "Restarting Container on Jetson (waiting for readiness)..."
    ssh -q "${SSH_OPTS[@]}" "${TARGET}" "cd \"${REMOTE_DIR}\" && \
        IMAGE_NAME=\"${IMAGE_NAME}\" IMAGE_TAG=\"${IMAGE_TAG}\" docker compose down && \
        IMAGE_NAME=\"${IMAGE_NAME}\" IMAGE_TAG=\"${IMAGE_TAG}\" docker compose up -d --wait"

    echo -e "\n\e[32m[DONE]\e[0m Deployment complete!"
    exit 0
}


# ==========================================
# MAIN EXECUTION ROUTER
# ==========================================

if [ "$SYNC_MODE" = true ]; then
    run_sync_all_mode
elif [ "$SYNC_SRC_MODE" = true ]; then
    run_sync_src_mode
elif [ "$SYNC_COMPOSE_MODE" = true ]; then
    run_sync_compose_mode
elif [ "$PULL_MODE" = true ]; then
    run_pull_mode
else
    run_full_deploy_mode
fi