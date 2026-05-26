#!/bin/bash

set -e
set -o pipefail

# PATH RESOLUTION
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$REPO_ROOT" || { echo -e "\e[31m[ERROR]\e[0m Failed to navigate to repository root."; exit 1; }

DEFAULT_JETSON_USER="ros"
DEFAULT_JETSON_IP="10.42.0.1"
DEFAULT_REMOTE_DIR="/home/ros/Indomitus/indomitus-rover-core/"
DEFAULT_IMAGE_NAME="indomitus-rover"
DEFAULT_IMAGE_TAG="humble-prod"
DEFAULT_DOCKERFILE="docker/Dockerfile"
DEFAULT_COMPOSE_FILE="docker/docker-compose.prod.yaml"
DEFAULT_WIFI_SSID="JetsonRosIndomitus"
DEFAULT_WIFI_PASS="jetson1234"

JETSON_USER="$DEFAULT_JETSON_USER"
JETSON_IP="$DEFAULT_JETSON_IP"
REMOTE_DIR="$DEFAULT_REMOTE_DIR"
IMAGE_NAME="$DEFAULT_IMAGE_NAME"
IMAGE_TAG="$DEFAULT_IMAGE_TAG"
DOCKERFILE="$DEFAULT_DOCKERFILE"
COMPOSE_FILE="$DEFAULT_COMPOSE_FILE"
WIFI_SSID="$DEFAULT_WIFI_SSID"
WIFI_PASS="$DEFAULT_WIFI_PASS"

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Cross-compiles the ROS 2 production image, packages it, securely transfers it to 
the offline Jetson Nano, and loads it into the Docker engine, ready for execution.

NOTE: You can run this script from any folder on your computer. All local paths 
provided in the flags MUST be relative to the root of the repository.

Options:
    -i, --ip IP           Jetson Nano IP address (Default: ${DEFAULT_JETSON_IP})
    -u, --user USER       Jetson Nano SSH username (Default: ${DEFAULT_JETSON_USER})
    -d, --dir DIR         Remote deployment directory on the Jetson. Can be absolute (e.g. /opt/rover) or relative to user home (Default: ${DEFAULT_REMOTE_DIR})
    -n, --name NAME       Docker image name (Default: ${DEFAULT_IMAGE_NAME})
    -t, --tag TAG         Docker image tag (Default: ${DEFAULT_IMAGE_TAG})
    -f, --file FILE       Path to the Dockerfile, relative to repo root (Default: ${DEFAULT_DOCKERFILE})
    -c, --compose FILE    Path to the Production Compose file, relative to repo root (Default: ${DEFAULT_COMPOSE_FILE})
    -w, --ssid SSID       Wi-Fi SSID of the Jetson hotspot to auto-connect (Default: ${DEFAULT_WIFI_SSID})
    -p, --pass PASS       Wi-Fi password for the Jetson hotspot (Default: ${DEFAULT_WIFI_PASS})
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

TARGET="${JETSON_USER}@${JETSON_IP}"
ARCHIVE_NAME="${IMAGE_NAME}_${IMAGE_TAG}.tar.gz"

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


# 1. PRE-FLIGHT CHECKS
step "Running Pre-Flight Checks..."

if ! docker info > /dev/null 2>&1; then error "Docker is not running."; fi
if ! docker buildx version > /dev/null 2>&1; then error "Docker Buildx is missing."; fi
if [ ! -f "$DOCKERFILE" ]; then error "Dockerfile not found: $DOCKERFILE"; fi
if [ ! -f "$COMPOSE_FILE" ]; then error "Compose file not found: $COMPOSE_FILE"; fi


# 2. ENSURE CROSS-COMPILATION SUPPORT
step "Ensuring ARM64 Cross-Compilation Support (QEMU)..."
echo -n "Checking/downloading emulators (this may take up to 5 minutes on the first run)..."
(docker run --rm --privileged multiarch/qemu-user-static --reset -p yes > /dev/null 2>&1) &
pid=$!
spinner $pid
if wait $pid; then
    echo ""
    success "QEMU emulators configured."
else
    echo ""
    echo -e "\e[33m[WARNING]\e[0m Could not automatically register QEMU emulators. The build may fail if not already configured."
fi


# 3. CROSS-COMPILE IMAGE (LAPTOP)
step "Building ARM64 Image (${IMAGE_NAME}:${IMAGE_TAG})..."
docker buildx build --platform linux/arm64 --target prod -t "${IMAGE_NAME}:${IMAGE_TAG}" -f "${DOCKERFILE}" .


# 4. EXPORT & COMPRESS (LAPTOP)
step "Compressing Image to ${ARCHIVE_NAME}..."
echo -n "Compressing archive (this may take 1-3 minutes)..."
(docker save "${IMAGE_NAME}:${IMAGE_TAG}" | gzip > "${ARCHIVE_NAME}") &
pid=$!
spinner $pid
wait $pid || error "Failed to export and compress the Docker image."
echo ""
success "Compressed file size: $(du -h "${ARCHIVE_NAME}" | cut -f1)"


# 5. WAIT FOR JETSON CONNECTION
step "Verifying Jetson Nano Connection..."

if [ -n "$WIFI_SSID" ]; then
    echo "Attempting to automatically connect to Wi-Fi network: ${WIFI_SSID}..."
    echo "(Note: You can also switch to this network manually right now if you prefer.)"
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
else
    echo -e "Please switch your Wi-Fi network to the Jetson Nano hotspot now."
fi

echo -n "Waiting for SSH connection to ${TARGET}..."

MAX_RETRIES=60
RETRY_COUNT=0
while ! ssh -q -o BatchMode=yes -o ConnectTimeout=2 -o StrictHostKeyChecking=accept-new "${TARGET}" "echo 'SSH Ready'" > /dev/null 2>&1; do
    sleep 2
    echo -n "."
    RETRY_COUNT=$((RETRY_COUNT+1))
    if [ "$RETRY_COUNT" -ge "$MAX_RETRIES" ]; then
        echo ""
        error "Timeout: Could not connect to Jetson Nano at ${TARGET} after 2 minutes."
    fi
done
echo ""
success "Connection established."


# 6. TRANSFER TO JETSON NANO
step "Transferring Payload to ${TARGET}:${REMOTE_DIR}..."
ssh -q "${TARGET}" "mkdir -p -- \"${REMOTE_DIR}\""
scp "${ARCHIVE_NAME}" "${COMPOSE_FILE}" "${TARGET}:${REMOTE_DIR}/"


# 7. LOAD & CLEANUP (JETSON NANO)
step "Loading Image on Jetson Nano..."
echo -n "Unpacking into Jetson's Docker engine (this may take 2-4 minutes)..."

(ssh -q "${TARGET}" "cd \"${REMOTE_DIR}\" && { gunzip -c \"${ARCHIVE_NAME}\" | docker load; RES=\$?; rm -f \"${ARCHIVE_NAME}\"; exit \$RES; }") &
pid=$!
spinner $pid
wait $pid || error "Failed to load the image into the Jetson's Docker engine."
echo ""
success "Image loaded and remote temporary files cleaned."


# 8. COMPLETION
echo -e "\n\e[32m[DONE]\e[0m Deployment staged successfully!"
echo -e "To start the container on the Jetson, run the following command:"
REMOTE_COMPOSE_FILE=$(basename "${COMPOSE_FILE}")
echo -e "\e[36mssh ${TARGET} 'cd \"${REMOTE_DIR}\" && docker compose -f ${REMOTE_COMPOSE_FILE} up -d'\e[0m\n"