#!/bin/bash

set -e
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
FILTER_FILE="${SCRIPT_DIR}/.rsync-filter-deploy"

# IMPORT SHARED FUNCTIONS
source "${SCRIPT_DIR}/utils.sh"

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
IMAGE_COMMIT=""
CONTAINER_NAME="rover_prod"
DOCKERFILE="docker/Dockerfile"
COMPOSE_FILE="docker/docker-compose.prod.yaml"
WIFI_SSID="ERC_UCUSpaceRobotics_A"
WIFI_PASS="19283746"

# ACTION MODES
REMOTE_BUILD_MODE=false
SYNC_SRC_MODE=false
SYNC_COMPOSE_MODE=false
PULL_MODE=false
LOCAL_BUILD_MODE=false

USE_ETH=false

show_help() {
    cat << EOF
Usage: $0 [MODE] [OPTIONS]

Deploys code to the Jetson. 

Modes (REQUIRED - You must specify exactly one):
    remote-build                REMOTE BUILD: Syncs the entire local repository to the Jetson, builds the image natively, and restarts.
    sync-src                    SYNC SRC: Syncs ONLY 'src' and auto-compiles inside the running container.
    sync-docker-compose         SYNC COMPOSE: Syncs ONLY the compose file and restarts the container.
    pull                        PULL MODE: Pulls a published image from GHCR and deploys it.
    local-build                 LOCAL BUILD: Cross-compiles a new ARM64 image locally, then deploys it.
                                    ATTENTION: This is a deprecated mode and is not guaranteed to work. If you still want to use it, 
                                    turn off address space randomization before deployment with command "sudo sysctl kernel.randomize_va_space=0". 
                                    After deployment, turn it back on with "sudo sysctl kernel.randomize_va_space=2".

Options:
    --eth                       Use wired Ethernet connection (${JETSON_ETHERNET_IP}) instead of hotspot.
    --ip IP                     Jetson IP address (Default: ${JETSON_IP})
    --user USER                 Jetson SSH username (Default: ${JETSON_USER})
    --dir DIR                   Remote deployment directory on the Jetson. (Default: ${REMOTE_DIR})
    --image-name NAME           Docker image name (Default: ${IMAGE_NAME})
    --tag TAG                   Docker image tag (e.g., develop-prod, feature-branch-prod).
    --commit SHA                Git commit SHA (7+ hex chars) to pull. Overrides --tag in pull mode.
    --container-name NAME       Docker container name (Default: ${CONTAINER_NAME})
    --dockerfile FILE           Path to the Dockerfile (Default: ${DOCKERFILE})
    --compose FILE              Path to the Production Compose file (Default: ${COMPOSE_FILE})
    --ssid SSID                 Wi-Fi SSID of the Jetson hotspot (Default: ${WIFI_SSID})
    --pass PASS                 Wi-Fi password for the Jetson hotspot (Default: ${WIFI_PASS})
    -h, --help                  Display this help message and exit
EOF
}

success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
warning() { echo -e "\e[33m[WARNING]\e[0m $1"; }
error() { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step() { echo -e "\n\e[34m>>> $1\e[0m"; }

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
        remote-build) REMOTE_BUILD_MODE=true; shift 1;;
        sync-src) SYNC_SRC_MODE=true; shift 1;;
        sync-docker-compose) SYNC_COMPOSE_MODE=true; shift 1;;
        pull) PULL_MODE=true; shift 1;;
        local-build) LOCAL_BUILD_MODE=true; shift 1;;
        --eth) USE_ETH=true; JETSON_IP="${JETSON_ETHERNET_IP}"; shift 1;;
        --user) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_USER="$2"; shift 2;;
        --ip) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; JETSON_IP="$2"; shift 2;;
        --dir) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; REMOTE_DIR="$2"; shift 2;;
        --image-name) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; IMAGE_NAME="$2"; shift 2;;
        --tag) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; IMAGE_TAG="$2"; shift 2;;
        --commit) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; IMAGE_COMMIT="$2"; shift 2;;
        --container-name) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; CONTAINER_NAME="$2"; shift 2;;
        --dockerfile) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; DOCKERFILE="$2"; shift 2;;
        --compose) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; COMPOSE_FILE="$2"; shift 2;;
        --ssid) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_SSID="$2"; shift 2;;
        --pass) [[ "$#" -ge 2 ]] || error "$1 requires an argument."; WIFI_PASS="$2"; shift 2;;
        -h|--help) show_help; exit 0;;
        *) error "Unknown command or option: $1\nRun '$0 --help' for usage." ;;
    esac
done


# ==========================================
# MODE VALIDATION
# ==========================================

MODE_COUNT=0
for mode in "$REMOTE_BUILD_MODE" "$SYNC_SRC_MODE" "$SYNC_COMPOSE_MODE" "$PULL_MODE" "$LOCAL_BUILD_MODE"; do
    if [ "$mode" = true ]; then
        MODE_COUNT=$((MODE_COUNT + 1))
    fi
done

if [ "$MODE_COUNT" -eq 0 ]; then
    error "No deployment mode specified. You must explicitly select a command (e.g., remote-build, pull).\nRun '$0 --help' for options."
elif [ "$MODE_COUNT" -gt 1 ]; then
    error "Multiple deployment modes specified. Please select only one command."
fi


# ==========================================
# PRE-FLIGHT CHECKS & VARS
# ==========================================

if [ ! -f "$FILTER_FILE" ] && [ "$SYNC_COMPOSE_MODE" = false ]; then 
    error "rsync filter file not found: $FILTER_FILE"
fi

# ENFORCE TAG OR COMMIT IN PULL MODE
if [ "$PULL_MODE" = true ] && [ -z "$IMAGE_TAG" ] && [ -z "$IMAGE_COMMIT" ]; then
    error "The --tag (or --commit) flag is mandatory when using pull mode."
fi

if [ -z "$IMAGE_TAG" ] && [ -z "$IMAGE_COMMIT" ]; then
    if [ "$LOCAL_BUILD_MODE" = true ] || [ "$REMOTE_BUILD_MODE" = true ]; then
        IMAGE_TAG="local-prod"
    else
        IMAGE_TAG="develop-prod"
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
    ensure_wifi_connection "$WIFI_SSID" "$WIFI_PASS" "$USE_ETH"
    wait_for_ssh "$TARGET" 30 "$USE_ETH"
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
        IMAGE_NAME="$IMAGE_NAME" IMAGE_TAG="$PREV_TAG" docker compose up -d --wait --no-build --pull never \
            && echo "[ROLLBACK] Restored to image tag: ${PREV_TAG}" >&2 \
            || echo "[ROLLBACK] Restore also failed — manual intervention required." >&2
    else
        echo "[ROLLBACK] No previous tag recorded — cannot auto-restore." >&2
    fi
}

trap rollback ERR

IMAGE_NAME="$IMAGE_NAME" IMAGE_TAG="$IMAGE_TAG" docker compose down
IMAGE_NAME="$IMAGE_NAME" IMAGE_TAG="$IMAGE_TAG" docker compose up -d --wait --no-build --pull never

trap - ERR
echo "[OK] Container is up on tag: ${IMAGE_TAG}"
REMOTE_EOF
}

run_remote_build_mode() {
    connect_to_jetson

    step "SYNC ALL: Transferring entire local repository to Jetson..."

    rsync -avz --delete \
        --filter="merge ${FILTER_FILE}" \
        --info=progress2 \
        -e "ssh -q ${SSH_OPTS[*]}" \
        ./ "${TARGET}:${REMOTE_DIR}/"

    rsync -az --info=progress2 -e "ssh -q ${SSH_OPTS[*]}" "${COMPOSE_FILE}" "${TARGET}:${REMOTE_DIR}/docker-compose.yaml"

    step "Building Docker Image natively on Jetson (${IMAGE_NAME}:${IMAGE_TAG})..."

    if ssh -t -q "${SSH_OPTS[@]}" "${TARGET}" "cd \"${REMOTE_DIR}\" && IMAGE_NAME=\"${IMAGE_NAME}\" IMAGE_TAG=\"${IMAGE_TAG}\" docker compose --progress=tty build"; then
        success "Image successfully built on the Jetson."
    else
        error "Remote Docker Compose build failed."
    fi

    step "Restarting Container on Jetson (Safe Mode with Rollback)..."

    restart_with_rollback "${REMOTE_DIR}" "${IMAGE_NAME}" "${IMAGE_TAG}"

    step "Pruning dangling images on Jetson..."
    ssh -q "${SSH_OPTS[@]}" "${TARGET}" "docker image prune -f"

    echo -e "\n\e[32m[DONE]\e[0m Full Repo Sync & Native Build Complete!"
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


resolve_pull_target() {
    if [ -n "$IMAGE_COMMIT" ]; then
        if [[ ! "$IMAGE_COMMIT" =~ ^[0-9a-f]+$ ]]; then
            error "Invalid --commit '${IMAGE_COMMIT}': contains non-hex characters."
        fi

        local SHORT_SHA="${IMAGE_COMMIT:0:7}"
        PULL_IMAGE_TAG="sha-${SHORT_SHA}-prod"

        echo "Resolving commit via GitHub API..."
        local API_URL="https://api.github.com/repos/ucuspacerobotics/indomitus-rover-core/commits/${SHORT_SHA}"
        local FULL_SHA
        FULL_SHA=$(curl -sf "$API_URL" | python3 -c "import sys,json; print(json.load(sys.stdin)['sha'])" 2>/dev/null) \
            || error "Could not resolve SHA '${SHORT_SHA}' via GitHub API. Check your internet connection or that the commit exists."

        PULL_GIT_REF="$FULL_SHA"
    else
        PULL_IMAGE_TAG="$IMAGE_TAG"
        PULL_GIT_REF=""
    fi
}


run_pull_mode() {
    step "PULL MODE: Validating target..."
    resolve_pull_target

    step "Pulling ${IMAGE_NAME}:${PULL_IMAGE_TAG} (linux/arm64) from GHCR..."

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
Images are only published on pushes to main/develop branches or manual trigger of the publish image workflow.
Make sure a CI run completed successfully for this tag."
    fi

    ARM64_DIGEST=$(echo "$MANIFEST_JSON" | python3 -c "$PYTHON_SCRIPT" 2>/dev/null)
    [ -z "$ARM64_DIGEST" ] && error "Image '${IMAGE_NAME}:${PULL_IMAGE_TAG}' exists in GHCR but has no linux/arm64 manifest."

    local OLD_IMAGE_ID
    OLD_IMAGE_ID=$(docker images -q "${IMAGE_NAME}:${PULL_IMAGE_TAG}" 2>/dev/null)

    docker pull "${IMAGE_NAME}@${ARM64_DIGEST}" || error "Docker pull failed."
    docker tag "${IMAGE_NAME}@${ARM64_DIGEST}" "${IMAGE_NAME}:${PULL_IMAGE_TAG}"

    local NEW_IMAGE_ID
    NEW_IMAGE_ID=$(docker images -q "${IMAGE_NAME}:${PULL_IMAGE_TAG}" 2>/dev/null)
    if [ -n "$OLD_IMAGE_ID" ] && [ "$OLD_IMAGE_ID" != "$NEW_IMAGE_ID" ]; then
        docker rmi -f "$OLD_IMAGE_ID" >/dev/null 2>&1 || true
    fi

    if [ -z "$PULL_GIT_REF" ]; then
        step "Extracting Git Commit SHA from Docker image metadata..."
        local EXTRACTED_SHA
        EXTRACTED_SHA=$(docker inspect --format '{{ index .Config.Labels "org.opencontainers.image.revision" }}' "${IMAGE_NAME}:${PULL_IMAGE_TAG}" 2>/dev/null || echo "")

        if [ -n "$EXTRACTED_SHA" ] && [ "$EXTRACTED_SHA" != "<no value>" ]; then
            PULL_GIT_REF="$EXTRACTED_SHA"
            success "Image was built from commit: ${PULL_GIT_REF:0:7}"
        else
            warning "No standard OCI revision label found in the image."
            PULL_GIT_REF="${IMAGE_TAG%-prod}"
            echo "Falling back to dynamic branch inference: '${PULL_GIT_REF}'"
        fi
    fi

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
        success "Source checked out at commit ${PULL_GIT_REF:0:7}."
    fi

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

    echo -e "\n\e[32m[DONE]\e[0m Pull & Deploy Complete! (ref: ${PULL_GIT_REF:0:7}, image: ${PULL_IMAGE_TAG})"
    exit 0
}


run_local_build_mode() {
    warning "ATTENTION: This is a deprecated mode and is not guaranteed to work. If you still want to use it, turn off address space randomization with command 'sudo sysctl kernel.randomize_va_space=0'. After deployment, turn it back on with 'sudo sysctl kernel.randomize_va_space=2'."

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

    step "Building ARM64 Image locally via Compose (${IMAGE_NAME}:${IMAGE_TAG})..."
    
    IMAGE_NAME="${IMAGE_NAME}" \
    IMAGE_TAG="${IMAGE_TAG}" \
    DOCKER_DEFAULT_PLATFORM=linux/arm64 \
    docker compose -f "${COMPOSE_FILE}" build

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

if [ "$REMOTE_BUILD_MODE" = true ]; then
    run_remote_build_mode
elif [ "$LOCAL_BUILD_MODE" = true ]; then
    run_local_build_mode
elif [ "$SYNC_SRC_MODE" = true ]; then
    run_sync_src_mode
elif [ "$SYNC_COMPOSE_MODE" = true ]; then
    run_sync_compose_mode
elif [ "$PULL_MODE" = true ]; then
    run_pull_mode
fi