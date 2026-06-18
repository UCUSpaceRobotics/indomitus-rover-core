#!/bin/bash

set -e
set -o pipefail

# PATH RESOLUTION
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT" || { echo -e "\e[31m[ERROR]\e[0m Failed to navigate to repository root."; exit 1; }

# DEFAULT VARIABLES
JETSON_USER="ros"
JETSON_IP="10.42.0.1"
REMOTE_DIR="/home/ros/Indomitus/indomitus-rover-core/"
IMAGE_NAME="ghcr.io/ucuspacerobotics/indomitus-rover-core"
IMAGE_TAG=""
CONTAINER_NAME="rover_prod"
DOCKERFILE="docker/Dockerfile"
COMPOSE_FILE="docker/docker-compose.prod.yaml"
WIFI_SSID="JetsonRosIndomitus"
WIFI_PASS="jetson1234"
SYNC_MODE=false
PULL_MODE=false

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Deploys code to the Jetson Nano. 

Options:
    -s, --sync            SYNC MODE: Skips Docker build. Syncs 'src' and auto-compiles on the Jetson.
    -P, --pull            PULL MODE: Laptop pulls image from GHCR, transfers, and loads it.
    -i, --ip IP           Jetson Nano IP address (Default: ${JETSON_IP})
    -u, --user USER       Jetson Nano SSH username (Default: ${JETSON_USER})
    -d, --dir DIR         Remote deployment directory on the Jetson. (Default: ${REMOTE_DIR})
    -n, --name NAME       Docker image name (Default: ${IMAGE_NAME})
    -t, --tag TAG         Docker image tag (Default: local-prod or develop-prod)
    -f, --file FILE       Path to the Dockerfile (Default: ${DOCKERFILE})
    -c, --compose FILE    Path to the Production Compose file (Default: ${COMPOSE_FILE})
    -w, --ssid SSID       Wi-Fi SSID of the Jetson hotspot (Default: ${WIFI_SSID})
    -p, --pass PASS       Wi-Fi password for the Jetson hotspot (Default: ${WIFI_PASS})
    -h, --help            Display this help message and exit
EOF
}

# HELPER FUNCTIONS
success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
error()   { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step()    { echo -e "\n\e[33m>>> $1\e[0m"; }

# PARSE TERMINAL ARGUMENTS
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -s|--sync) SYNC_MODE=true; shift 1;;
        -P|--pull) PULL_MODE=true; shift 1;;
        -u|--user) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
        -i|--ip) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
        -d|--dir) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; REMOTE_DIR="$2"; shift 2;;
        -n|--name) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; IMAGE_NAME="$2"; shift 2;;
        -t|--tag) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; IMAGE_TAG="$2"; shift 2;;
        -f|--file) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; DOCKERFILE="$2"; shift 2;;
        -c|--compose) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_FILE="$2"; shift 2;;
        -w|--ssid) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
        -p|--pass) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
        -h|--help) show_help; exit 0;;
        *) show_help; exit 1;;
    esac
done

# DYNAMIC TAG LOGIC
if [ -z "$IMAGE_TAG" ]; then
    if [ "$SYNC_MODE" = true ] || [ "$PULL_MODE" = true ]; then
        IMAGE_TAG="develop-prod"
    else
        IMAGE_TAG="local-prod"
    fi
fi

TARGET="${JETSON_USER}@${JETSON_IP}"
ARCHIVE_NAME="deploy_temp_archive.tar"

# GLOBAL CLEANUP HOOK
trap 'rm -f "${ARCHIVE_NAME}"' EXIT

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

# --- CONNECTION FUNCTION ---
connect_to_jetson() {
    step "Verifying Jetson Nano Connection..."

    if [ -n "$WIFI_SSID" ]; then
        echo "Attempting to automatically connect to Wi-Fi network: ${WIFI_SSID}..."
        if command -v nmcli >/dev/null 2>&1; then
            if [ -n "$WIFI_PASS" ]; then
                nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASS" >/dev/null 2>&1 || true
            else
                nmcli device wifi connect "$WIFI_SSID" >/dev/null 2>&1 || true
            fi
        elif command -v networksetup >/dev/null 2>&1; then
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
    MAX_RETRIES=15
    RETRY_COUNT=0
    while ! ssh -q -o BatchMode=yes -o ConnectTimeout=2 -o StrictHostKeyChecking=accept-new "${TARGET}" "echo 'SSH Ready'" > /dev/null 2>&1; do
        sleep 2
        echo -n "."
        RETRY_COUNT=$((RETRY_COUNT+1))
        if [ "$RETRY_COUNT" -ge "$MAX_RETRIES" ]; then
            echo ""
            error "Timeout: Could not connect to Jetson Nano at ${TARGET}."
        fi
    done
    echo ""
    success "Connection established."
    ssh -q "${TARGET}" "mkdir -p -- \"${REMOTE_DIR}\""
}


# ==========================================
# MODE 1: RAPID CODE SYNC (--sync)
# ==========================================
if [ "$SYNC_MODE" = true ]; then
    connect_to_jetson
    
    step "SYNC MODE: Syncing local 'src' directory via rsync..."
    if [ ! -d "src" ]; then error "No 'src' directory found in the repository root."; fi
    
    rsync -avz --delete -e "ssh -q -o StrictHostKeyChecking=accept-new" src/ "${TARGET}:${REMOTE_DIR}/src/"
    
    step "Compiling code on Jetson (Inside Docker)..."
    echo -n "Triggering colcon build inside '${CONTAINER_NAME}'..."
    
    echo ""
    if ssh -q "${TARGET}" "docker exec ${CONTAINER_NAME} bash -c 'source /opt/ros/\$ROS_DISTRO/setup.bash && cd /opt/ws && colcon build --symlink-install'"; then
        success "Code successfully compiled on the Jetson!"
    else
        echo -e "\e[31m[ERROR]\e[0m Compilation failed, or the container '${CONTAINER_NAME}' is not running."
        exit 1
    fi
    
    echo -e "\n\e[32m[DONE]\e[0m Sync Process Complete!"
    exit 0
fi


# ==========================================
# MODE 3: PULL & BRIDGE (--pull)
# ==========================================
if [ "$PULL_MODE" = true ]; then
    step "PULL MODE: Pulling ${IMAGE_NAME}:${IMAGE_TAG} to local machine..."
    docker pull "${IMAGE_NAME}:${IMAGE_TAG}" || error "Docker pull failed."

    step "Exporting Image to ${ARCHIVE_NAME}..."
    docker save -o "${ARCHIVE_NAME}" "${IMAGE_NAME}:${IMAGE_TAG}"

    connect_to_jetson
    
    step "Transferring Payload to ${TARGET}:${REMOTE_DIR}..."
    if [ ! -d "src" ]; then error "No 'src' directory found in the repository root."; fi
    rsync -avz --delete -e "ssh -q -o StrictHostKeyChecking=accept-new" src/ "${TARGET}:${REMOTE_DIR}/src/"
    scp "${ARCHIVE_NAME}" "${TARGET}:${REMOTE_DIR}/"
    scp "${COMPOSE_FILE}" "${TARGET}:${REMOTE_DIR}/docker-compose.yaml"

    step "Loading Image on Jetson Nano..."
    ssh -q "${TARGET}" "cd \"${REMOTE_DIR}\" && docker load -i \"${ARCHIVE_NAME}\" && rm \"${ARCHIVE_NAME}\""
    
    step "Restarting Container on Jetson..."
    ssh -q "${TARGET}" "cd \"${REMOTE_DIR}\" && IMAGE_NAME=\"${IMAGE_NAME}\" IMAGE_TAG=\"${IMAGE_TAG}\" docker compose up -d"
    
    echo -e "\n\e[32m[DONE]\e[0m Pull & Deploy Complete!"
    exit 0
fi


# ==========================================
# MODE 2: FULL IMAGE DEPLOYMENT (Default)
# ==========================================
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
rsync -avz --delete -e "ssh -q -o StrictHostKeyChecking=accept-new" src/ "${TARGET}:${REMOTE_DIR}/src/"
scp "${ARCHIVE_NAME}" "${TARGET}:${REMOTE_DIR}/"
scp "${COMPOSE_FILE}" "${TARGET}:${REMOTE_DIR}/docker-compose.yaml"

step "Loading Image on Jetson Nano..."
ssh -q "${TARGET}" "cd \"${REMOTE_DIR}\" && docker load -i \"${ARCHIVE_NAME}\" && rm \"${ARCHIVE_NAME}\""

step "Restarting Container on Jetson..."
ssh -q "${TARGET}" "cd \"${REMOTE_DIR}\" && IMAGE_NAME=\"${IMAGE_NAME}\" IMAGE_TAG=\"${IMAGE_TAG}\" docker compose up -d"

echo -e "\n\e[32m[DONE]\e[0m Deployment complete!"