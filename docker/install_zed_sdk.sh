#!/bin/bash
# Installs the ZED SDK from a Stereolabs .run installer URL, for base images
# that ship without it (Jetson on Ubuntu 24.04). No-op if the URL is empty or
# the SDK is already there. Same steps as Stereolabs' own L4T Dockerfiles.
set -euo pipefail

url="${1:-}"
if [ -z "${url}" ] || [ -d /usr/local/zed/lib ]; then
    exit 0
fi

apt-get update
apt-get install -y --no-install-recommends lsb-release wget less zstd udev sudo apt-transport-https

# The installer reads the L4T release from here; containers do not have it.
if [ ! -f /etc/nv_tegra_release ]; then
    echo "# R36 (release), REVISION: 4.0" > /etc/nv_tegra_release
fi

wget -q -O /tmp/ZED_SDK_Linux.run "${url}"
chmod +x /tmp/ZED_SDK_Linux.run
/tmp/ZED_SDK_Linux.run silent skip_tools skip_drivers skip_python
rm -rf /tmp/ZED_SDK_Linux.run /usr/local/zed/resources/* /var/lib/apt/lists/*
