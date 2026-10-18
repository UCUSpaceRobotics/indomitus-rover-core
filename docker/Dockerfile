ARG UBUNTU_VERSION=22.04
ARG TARGET_ROS_DISTRO=humble
ARG BASE_IMAGE="ros:humble-ros-base"

ARG ROBOT_META="indomitus_robot"
ARG DESKTOP_META="indomitus_desktop"
ARG WS="/opt/ws"

####################
# CACHER
####################
FROM ubuntu:${UBUNTU_VERSION} AS cacher

WORKDIR /tmp/src
COPY src/ .

RUN mkdir -p /extracted_xmls && \
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
ARG DESKTOP_META

COPY --from=cacher /extracted_xmls/ ${WS}/src/

# Dependencies of indomitus_desktop and everything it pulls in.
RUN apt-get update && \
    rosdep install --from-paths $(colcon list --packages-up-to ${DESKTOP_META} --paths-only) \
        --ignore-src --rosdistro ${TARGET_ROS_DISTRO} -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN if [ "$INSTALL_SIM_TOOLS" = "true" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0 && \
        rm -rf /var/lib/apt/lists/*; \
    fi

# Debugging convenience tools, not any package's actual runtime dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
    gstreamer1.0-tools ros-humble-rqt-image-view && \
    rm -rf /var/lib/apt/lists/*

CMD ["bash"]


####################
# PROD
####################
FROM base AS prod
ARG ROBOT_META

COPY --from=cacher /extracted_xmls/ ${WS}/src/

# Dependencies of indomitus_robot only: no simulation, no viz.
RUN apt-get update && rosdep install --from-paths $(colcon list --packages-up-to ${ROBOT_META} --paths-only) \
    --ignore-src --rosdistro ${TARGET_ROS_DISTRO} -r -y \
    -t build \
    -t buildtool \
    -t build_export \
    -t buildtool_export \
    -t exec \
    && rm -rf /var/lib/apt/lists/*

COPY src/ ${WS}/src/

COPY docker/colcon_defaults.prod.yaml ${WS}/colcon_defaults.yaml
ENV COLCON_DEFAULTS_FILE=${WS}/colcon_defaults.yaml

COPY docker/colcon_defaults.prod.yaml colcon_defaults.yaml
RUN /bin/bash -c "source /opt/ros/${TARGET_ROS_DISTRO}/setup.bash && colcon build --packages-up-to ${ROBOT_META}"

RUN rm -rf ${WS}/log/

CMD ["bash"]
