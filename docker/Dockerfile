ARG UBUNTU_VERSION=24.04
ARG TARGET_ROS_DISTRO=jazzy
ARG BASE_IMAGE="ros:jazzy-ros-base"
# arm_tasks is not simulation code — it's excluded here (despite the var name)
# because its remaining nodes are pure network clients against /arm, meant to
# run from a dev/ground-station container rather than build into the Jetson
# production image.
ARG SIMULATION_PKGS="rover_sim rover_viz arm_sim arm_viz panel_sim arm_tasks"
ARG WS="/opt/ws"

####################
# CACHER
####################
FROM ubuntu:${UBUNTU_VERSION} AS cacher
ARG INSTALL_SIM_TOOLS=false
ARG SIMULATION_PKGS

WORKDIR /tmp/src
COPY src/ .

RUN if [ "$INSTALL_SIM_TOOLS" != "true" ]; then \
        for pkg in ${SIMULATION_PKGS}; do find . -name "$pkg" -type d -exec rm -rf {} + ; done; \
    fi && \
    mkdir -p /extracted_xmls && \
    find . -name "package.xml" -exec cp --parents {} /extracted_xmls/ \;


####################
# BASE
####################
FROM ${BASE_IMAGE} AS base
ARG TARGET_ROS_DISTRO
ARG WS

ENV WS=${WS}

ENV DEBIAN_FRONTEND=noninteractive
ENV TARGET_ROS_DISTRO=${TARGET_ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV TZ=Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# The Jetson (prod) base is jetson-containers' CUDA image (CUDA, cuDNN, TensorRT,
# onnxruntime for L4T r36 / JetPack 6, Ubuntu 24.04), which has no ROS: install
# ROS from the official apt repository, so rosdep and apt ROS packages work as on
# ros:jazzy-ros-base. (Its sibling dustynv/ros:jazzy-* builds ROS from source,
# which rosdep and apt ROS packages do not mix with.)
RUN if [ ! -f /opt/ros/${TARGET_ROS_DISTRO}/setup.bash ]; then \
        apt-get update && apt-get install -y --no-install-recommends curl ca-certificates && \
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg && \
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
            > /etc/apt/sources.list.d/ros2.list && \
        apt-get update && apt-get install -y --no-install-recommends ros-${TARGET_ROS_DISTRO}-ros-base && \
        rm -rf /var/lib/apt/lists/*; \
    fi

# jetson-containers keep their Python packages (onnxruntime, numpy, ...) in
# /opt/venv, first in PATH; ROS nodes and tools run on /usr/bin/python3. Make the
# venv's packages visible to the system Python (after its own, so apt's numpy
# etc. that ROS was built against win), and take the venv off PATH. The base
# image also points pip at its own mirror; use PyPI. No-ops on ros:* images.
RUN for sp in /opt/venv/lib/python3*/site-packages; do \
        [ -d "$sp" ] && echo "$sp" > /usr/lib/python3/dist-packages/zz-jetson-venv.pth; \
    done; true
ENV PATH=/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV PIP_INDEX_URL=https://pypi.org/simple
RUN if [ -d /opt/venv ]; then /usr/bin/python3 -c "import onnxruntime; print('onnxruntime', onnxruntime.__version__, onnxruntime.get_available_providers())"; fi

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales git curl wget ca-certificates \
    build-essential python3-pip \
    python3-colcon-common-extensions python3-rosdep \
    iputils-ping pkg-config usbutils iproute2 g++ make cmake \
    can-utils \
    vim && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

RUN rosdep init || true && rosdep update

RUN mkdir -p ${WS}/src
WORKDIR ${WS}

RUN echo "if [ -f /opt/ros/\${TARGET_ROS_DISTRO}/setup.bash ]; then source /opt/ros/\${TARGET_ROS_DISTRO}/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /root/.bashrc && \
    echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /root/.bash_aliases ]; then source /root/.bash_aliases; fi" >> /root/.bashrc

COPY docker/entrypoint.bash /entrypoint.bash
COPY docker/.bash_aliases /root/.bash_aliases
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]


####################
# DEV
####################
FROM base AS dev
ARG INSTALL_SIM_TOOLS=false
ARG TARGET_ROS_DISTRO

COPY --from=cacher /extracted_xmls/ ${WS}/src/

RUN apt-get update && \ 
    if [ "${INSTALL_SIM_TOOLS}" = "true" ]; then \
        rosdep install --from-paths ${WS}/src/ --ignore-src --rosdistro ${TARGET_ROS_DISTRO} -r -y ; \
    else \
        rosdep install --from-paths ${WS}/src/ --ignore-src --rosdistro ${TARGET_ROS_DISTRO} -r -y --skip-keys "${SIMULATION_PKGS}" ; \
    fi && \
    rm -rf /var/lib/apt/lists/*
        
RUN if [ "$INSTALL_SIM_TOOLS" = "true" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0 && \
        rm -rf /var/lib/apt/lists/*; \
    fi

# Debugging convenience tools, not any package's actual runtime dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
    gstreamer1.0-tools ros-${TARGET_ROS_DISTRO}-rqt-image-view && \
    rm -rf /var/lib/apt/lists/*

CMD ["bash"]


####################
# PROD
####################
FROM base AS prod
ARG SIMULATION_PKGS

COPY --from=cacher /extracted_xmls/ ${WS}/src/

RUN apt-get update && rosdep install --from-paths ${WS}/src --ignore-src --rosdistro ${TARGET_ROS_DISTRO} -r -y \
    --skip-keys "${SIMULATION_PKGS}" \
    -t build \
    -t buildtool \
    -t build_export \
    -t buildtool_export \
    -t exec \
    && rm -rf /var/lib/apt/lists/*

COPY src/ ${WS}/src/

RUN cd ${WS}/src && \
    for pkg in ${SIMULATION_PKGS}; do find . -name "$pkg" -type d -exec rm -rf {} + ; done

COPY docker/colcon_defaults.prod.yaml ${WS}/colcon_defaults.yaml
ENV COLCON_DEFAULTS_FILE=${WS}/colcon_defaults.yaml

COPY docker/colcon_defaults.prod.yaml colcon_defaults.yaml
RUN /bin/bash -c "source /opt/ros/${TARGET_ROS_DISTRO}/setup.bash && colcon build"

RUN rm -rf ${WS}/log/

CMD ["bash"]
