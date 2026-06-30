#!/bin/bash

set -e
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
FILTER_FILE="${SCRIPT_DIR}/.rsync-filter-deploy"

cd "$REPO_ROOT" || { echo -e "\e[31m[ERROR]\e[0m Failed to navigate to repository root."; exit 1; }

# DEFAULT VARIABLES
ROS_DISTRO="humble"
JETSON_USER="indomitus-rover"
JETSON_HOTSPOT_IP="10.42.0.1"
JETSON_ETHERNET_IP="indomitus-rover-computer.local"
JETSON_IP="${JETSON_HOTSPOT_IP}"
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
USE_ETH=false

BUILD_ARGS=(
    --build-arg BASE_IMAGE_HW_BUILDER=stereolabs/zed:5.4-devel-l4t-r36.4
    --build-arg BASE_IMAGE=stereolabs/zed:5.4-runtime-l4t-r36.4
    --build-arg UBUNTU_VERSION=22.04
    --build-arg TARGET_ROS_DISTRO=humble
)

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Deploys code to the Jetson. 

Modes:
    --sync                      SYNC ALL: Syncs both 'src' and compose file, restarts container, then compiles.
    --sync-src                  SYNC SRC: Syncs ONLY 'src' and auto-compiles inside the running container.
    --sync-docker-compose       SYNC COMPOSE: Syncs ONLY the compose file and restarts the container.
    --pull                      PULL MODE: Pulls a published image from GHCR and deploys it.
                                  Valid --tag values:
                                    develop-prod      Pull latest develop branch image (default)
                                    main-prod         Pull latest main branch image
                                    <commit-sha>      Pull a specific commit's image; pass 7+ hex chars
                                                        (e.g. a1b2c3d or a1b2c3d4e5f6)

Options:
    --eth                       Use wired Ethernet connection (${JETSON_ETHERNET_IP}) instead of hotspot.
    -i, --ip IP                 Jetson IP address (Default: ${JETSON_IP})
    -u, --user USER             Jetson SSH username (Default: ${JETSON_USER})
    -d, --dir DIR               Remote deployment directory on the Jetson. (Default: ${REMOTE_DIR})
    --image-name NAME           Docker image name (Default: ${IMAGE_NAME})
    -t, --tag TAG               Docker image tag; for --pull: develop-prod, main-prod, or a commit SHA (Default: local-prod or develop-prod)
    --container-name            Docker container name (Default: ${CONTAINER_NAME})
    -f, --file FILE             Path to the Dockerfile (Default: ${DOCKERFILE})
    -c, --compose FILE          Path to the Production Compose file (Default: ${COMPOSE_FILE})
    -w, --ssid SSID             Wi-Fi SSID of the Jetson hotspot (Default: ${WIFI_SSID})
    -p, --pass PASS             Wi-Fi password for the Jetson hotspot (Default: ${WIFI_PASS})
    -h, --help                  Display this help message and exit
EOF
}

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

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --sync) SYNC_MODE=true; shift 1;;
        --sync-src) SYNC_SRC_MODE=true; shift 1;;
        --sync-docker-compose) SYNC_COMPOSE_MODE=true; shift 1;;
        --pull) PULL_MODE=true; shift 1;;
        --eth) USE_ETH=true; JETSON_IP="${JETSON_ETHERNET_IP}"; shift 1;;
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

if [ ! -f "$FILTER_FILE" ] && [ "$SYNC_COMPOSE_MODE" = false ]; then 
    error "rsync filter file not found: $FILTER_FILE"
fi

if [ -z "$IMAGE_TAG" ]; then
    if [ "$SYNC_MODE" = true ] || [ "$SYNC_SRC_MODE" = true ] || [ "$SYNC_COMPOSE_MODE" = true ] || [ "$PULL_MODE" = true ]; then
        IMAGE_TAG="develop-prod"
    else
        IMAGE_TAG="local-prod"
    fi
fi

TARGET="${JETSON_USER}@${JETSON_IP}"
ARCHIVE_NAME="deploy_temp_archive.tar"

SSH_OPTS=(
    -o ControlMaster=auto
    -o "ControlPath=/tmp/ssh_mux_%h_%p_%r"
    -o ControlPersist=10m
    -o StrictHostKeyChecking=accept-new
)

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

    # Skip automatic Wi-Fi connection if we are using Ethernet
    if [ -n "$WIFI_SSID" ] && [ "$USE_ETH" = false ]; then
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

restart_with_rollback() {
    local remote_dir="$1"
    local image_name="$2"
    local image_tag="$3"

    ssh -q "${SSH_OPTS[@]}" "${TARGET}" bash -s -- \
        "$remote_dir" "$image_name" "$image_tag" << 'REMOTE_EOF'
set -euo pipefail
REMOTE_DIR="$1"
IMAGE_NAME="$2"
IMAGE_TAG="$3"

cd "$REMOTE_DIR"

PREV_TAG=$(
    docker inspect --format='{{index .Config.Image}}' \
        "$(docker compose ps -q 2>/dev/null | head -1)" 2>/dev/null \
    | sed 's/.*://' \
) || PREV_TAG=""

rollback() {
    echo "[ROLLBACK] Startup failed. Attempting to restore previous container..." >&2
    if [ -n "$PREV_TAG" ]; then
        IMAGE_NAME="$IMAGE_NAME" IMAGE_TAG="$PREV_TAG" docker compose up -d --wait \
            && echo "[ROLLBACK] Restored to image tag: ${PREV_TAG}" >&2 \
            || echo "[ROLLBACK] Restore also failed — manual intervention required." >&2
    else
        echo "[ROLLBACK] No previous tag recorded — cannot auto-restore." >&2
    fi
}

trap rollback ERR

IMAGE_NAME="$IMAGE_NAME" IMAGE_TAG="$IMAGE_TAG" docker compose down
IMAGE_NAME="$IMAGE_NAME" IMAGE_TAG="$IMAGE_TAG" docker compose up -d --wait

trap - ERR
echo "[OK] Container is up on tag: ${IMAGE_TAG}"
REMOTE_EOF
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
    restart_with_rollback "${REMOTE_DIR}" "${IMAGE_NAME}" "${IMAGE_TAG}"
    
    step "Compiling code on Jetson (Inside Docker)..."
    echo -n "Triggering colcon build inside '${CONTAINER_NAME}'..."
    echo ""
    if ssh -q "${SSH_OPTS[@]}" "${TARGET}" "docker exec ${CONTAINER_NAME} bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && cd /opt/ws && colcon build --symlink-install'"; then
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
    if ssh -q "${SSH_OPTS[@]}" "${TARGET}" "docker exec ${CONTAINER_NAME} bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && cd /opt/ws && colcon build --symlink-install'"; then
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
    restart_with_rollback "${REMOTE_DIR}" "${IMAGE_NAME}" "${IMAGE_TAG}"
    
    echo -e "\n\e[32m[DONE]\e[0m Compose Sync Complete!"
    exit 0
}


resolve_pull_ref() {
    case "$IMAGE_TAG" in

      develop-prod|main-prod)
          local BRANCH="${IMAGE_TAG%-prod}"
          echo "Tag '${IMAGE_TAG}' → tracking HEAD of branch '${BRANCH}'."
          PULL_GIT_REF="$BRANCH"
          PULL_IMAGE_TAG="$IMAGE_TAG"
          ;;

      [0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f]*)

          if [[ ! "$IMAGE_TAG" =~ ^[0-9a-f]+$ ]]; then
              error "Invalid --tag '${IMAGE_TAG}': looks like a commit hash but contains non-hex characters."
          fi

          local SHORT_SHA="${IMAGE_TAG:0:7}"
          PULL_IMAGE_TAG="sha-${SHORT_SHA}-prod"

          echo "Resolving commit via GitHub API..."
          local API_URL="https://api.github.com/repos/ucuspacerobotics/indomitus-rover-core/commits/${SHORT_SHA}"
          local FULL_SHA
          FULL_SHA=$(curl -sf "$API_URL" | python3 -c "import sys,json; print(json.load(sys.stdin)['sha'])" 2>/dev/null) \
              || error "Could not resolve SHA '${SHORT_SHA}' via GitHub API. Check your internet connection or that the commit exists."

          PULL_GIT_REF="$FULL_SHA"
          ;;

      *)
          error "Invalid --tag '${IMAGE_TAG}' for --pull mode.
Valid values are:
  develop-prod        Latest image from the develop branch
  main-prod           Latest image from the main branch
  <commit-sha>        Specific commit (7+ hex chars, e.g. a1b2c3d or a1b2c3d4e5f6)"
          ;;
    esac
}


run_pull_mode() {
    step "PULL MODE: Validating tag..."
    resolve_pull_ref

    local TEMP_REPO_DIR
    TEMP_REPO_DIR=$(mktemp -d)
    CLEANUP_FILES+=("$TEMP_REPO_DIR")

    local REPO_URL="https://github.com/ucuspacerobotics/indomitus-rover-core.git"

    step "Fetching clean source (ref: ${PULL_GIT_REF}) from GitHub..."
    if git clone --depth 1 --branch "$PULL_GIT_REF" "$REPO_URL" "$TEMP_REPO_DIR" > /dev/null 2>&1; then
        success "Source cloned successfully."
    else
        local BARE_DIR
        BARE_DIR=$(mktemp -d)
        CLEANUP_FILES+=("$BARE_DIR")
        git clone --no-checkout "$REPO_URL" "$BARE_DIR" > /dev/null 2>&1 \
            || error "Failed to clone repository. Check internet connection."
        git -C "$BARE_DIR" checkout "$PULL_GIT_REF" -- . > /dev/null 2>&1 \
            || error "Commit '${PULL_GIT_REF}' not found in repository."
        cp -a "$BARE_DIR/." "$TEMP_REPO_DIR/"
        success "Source checked out at commit ${PULL_GIT_REF}."
    fi

    step "Pulling ${IMAGE_NAME}:${PULL_IMAGE_TAG} (linux/arm64) from GHCR..."
    docker rmi -f "${IMAGE_NAME}:${PULL_IMAGE_TAG}" >/dev/null 2>&1 || true

    local PYTHON_SCRIPT="
import sys, json
manifests = json.load(sys.stdin).get('manifests', [])
for m in manifests:
    p = m.get('platform', {})
    if p.get('os') == 'linux' and p.get('architecture') == 'arm64':
        print(m['digest'])
        break
"

    local ARM64_DIGEST MANIFEST_JSON
    if ! MANIFEST_JSON=$(docker manifest inspect "${IMAGE_NAME}:${PULL_IMAGE_TAG}" 2>/dev/null); then
        error "Image '${IMAGE_NAME}:${PULL_IMAGE_TAG}' was not found in GHCR.
Images are only published on pushes to main/develop branches.
Make sure a CI run completed successfully for this commit."
    fi

    ARM64_DIGEST=$(echo "$MANIFEST_JSON" | python3 -c "$PYTHON_SCRIPT" 2>/dev/null)
    [ -z "$ARM64_DIGEST" ] && error "Image '${IMAGE_NAME}:${PULL_IMAGE_TAG}' exists in GHCR but has no linux/arm64 manifest."

    docker pull "${IMAGE_NAME}@${ARM64_DIGEST}" || error "Docker pull failed."
    docker tag "${IMAGE_NAME}@${ARM64_DIGEST}" "${IMAGE_NAME}:${PULL_IMAGE_TAG}"

    step "Exporting Image to ${ARCHIVE_NAME}..."
    echo -n "Exporting archive..."
    (docker save -o "${ARCHIVE_NAME}" "${IMAGE_NAME}:${PULL_IMAGE_TAG}") &
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
    restart_with_rollback "${REMOTE_DIR}" "${IMAGE_NAME}" "${PULL_IMAGE_TAG}"

    echo -e "\n\e[32m[DONE]\e[0m Pull & Deploy Complete! (ref: ${PULL_GIT_REF}, image: ${PULL_IMAGE_TAG})"
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
    docker buildx build --platform linux/arm64 --target prod \
        -t "${IMAGE_NAME}:${IMAGE_TAG}" \
        -f "${DOCKERFILE}" \
        "${BUILD_ARGS[@]}" \
        .

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
    restart_with_rollback "${REMOTE_DIR}" "${IMAGE_NAME}" "${IMAGE_TAG}"

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