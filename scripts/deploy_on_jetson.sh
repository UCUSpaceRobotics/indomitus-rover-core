#!/bin/bash

set -e

JETSON_USER="ros"
JETSON_IP="10.42.0.1"
REMOTE_DIR="/home/ros/Indomitus/indomitus-rover-core/"
IMAGE_NAME="indomitus-rover"
IMAGE_TAG="humble-prod"
DOCKERFILE="docker/Dockerfile"
COMPOSE_FILE="docker/docker-compose.prod.yaml"

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Cross-compiles the ROS 2 production image, packages it, securely transfers it to 
the offline Jetson Nano, and loads it into the Docker engine, ready for execution.

Options:
  -i IP           Jetson Nano IP address (Default: 10.42.0.1)
  -u USER         Jetson Nano SSH username (Default: ros)
  -d DIR          Remote deployment directory on the Jetson (Default: /home/ros/Indomitus/indomitus-rover-core/)
  -n NAME         Docker image name (Default: indomitus-rover)
  -t TAG          Docker image tag (Default: humble-prod)
  -f FILE         Path to the Dockerfile (Default: docker/Dockerfile)
  -c FILE         Path to the Production Compose file (Default: docker/docker-compose.prod.yaml)
  -h, --help      Display this help message and exit
EOF
}

for arg in "$@"; do
    if [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
        show_help
        exit 0
    fi
done

while getopts u:i:d:n:t:f:c: flag; do
    case "${flag}" in
        u) JETSON_USER=${OPTARG};;
        i) JETSON_IP=${OPTARG};;
        d) REMOTE_DIR=${OPTARG};;
        n) IMAGE_NAME=${OPTARG};;
        t) IMAGE_TAG=${OPTARG};;
        f) DOCKERFILE=${OPTARG};;
        c) COMPOSE_FILE=${OPTARG};;
        *) show_help; exit 1;;
    esac
done

TARGET="${JETSON_USER}@${JETSON_IP}"
ARCHIVE_NAME="${IMAGE_NAME}_${IMAGE_TAG}.tar.gz"

# HELPER FUNCTIONS
success() { echo -e "\e[32m[SUCCESS]\e[0m $1"; }
error()   { echo -e "\e[31m[ERROR]\e[0m $1"; exit 1; }
step()    { echo -e "\n\e[33m>>> $1\e[0m"; }


# 1. PRE-FLIGHT CHECKS
step "Running Pre-Flight Checks..."

if ! docker info > /dev/null 2>&1; then error "Docker is not running."; fi
if ! docker buildx version > /dev/null 2>&1; then error "Docker Buildx is missing."; fi
if [ ! -f "$DOCKERFILE" ]; then error "Dockerfile not found: $DOCKERFILE"; fi
if [ ! -f "$COMPOSE_FILE" ]; then error "Compose file not found: $COMPOSE_FILE"; fi

if ! ssh -q -o BatchMode=yes -o ConnectTimeout=5 "${TARGET}" "echo 'SSH Ready'" > /dev/null 2>&1; then
    error "Cannot connect to Jetson Nano at ${TARGET}. Check connection."
fi


# 2. CROSS-COMPILE IMAGE (LAPTOP)
step "Building ARM64 Image (${IMAGE_NAME}:${IMAGE_TAG})..."
docker buildx build --platform linux/arm64 --target prod -t "${IMAGE_NAME}:${IMAGE_TAG}" -f "${DOCKERFILE}" .


# 3. EXPORT & COMPRESS (LAPTOP)
step "Compressing Image to ${ARCHIVE_NAME}..."
docker save "${IMAGE_NAME}:${IMAGE_TAG}" | gzip > "${ARCHIVE_NAME}"
success "Compressed file size: $(du -h "${ARCHIVE_NAME}" | cut -f1)"


# 4. TRANSFER TO JETSON NANO
step "Transferring Payload to ${TARGET}:~/${REMOTE_DIR}..."
ssh -q "${TARGET}" "mkdir -p ~/${REMOTE_DIR}"
scp "${ARCHIVE_NAME}" "${COMPOSE_FILE}" "${TARGET}:~/${REMOTE_DIR}/"


# 5. LOAD & CLEANUP (JETSON NANO)
step "Loading Image on Jetson Nano (this takes a moment)..."
ssh -q "${TARGET}" << EOF
    cd ~/${REMOTE_DIR}
    gunzip -c ${ARCHIVE_NAME} | docker load
    rm ${ARCHIVE_NAME}
EOF
success "Image loaded and remote temporary files cleaned."


# 6. LOCAL CLEANUP & COMPLETION
rm "${ARCHIVE_NAME}"

echo -e "\n\e[32m[DONE]\e[0m Deployment staged successfully!"
echo -e "To start the rover, run the following command:"
echo -e "\e[36mssh ${TARGET} \"cd ~/${REMOTE_DIR} && docker compose -f ${COMPOSE_FILE} up -d\"\e[0m\n"