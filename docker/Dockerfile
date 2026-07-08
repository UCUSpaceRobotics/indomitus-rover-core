##### TEMP CHANGE

ARG UBUNTU_VERSION=22.04
ARG ROS_DISTRO=humble
ARG SIMULATION_PKGS="rover_sim rover_viz arm_sim arm_viz" 


####################
# CACHER
####################
FROM ubuntu:${UBUNTU_VERSION} AS cacher
ARG INSTALL_SIM_TOOLS=false
ARG SIMULATION_PKGS

COPY src/ /tmp/src/

RUN cd /tmp/src && \
    if [ "$INSTALL_SIM_TOOLS" != "true" ]; then \
        for pkg in ${SIMULATION_PKGS}; do find . -name "$pkg" -type d -exec rm -rf {} + ; done; \
    fi && \
    mkdir -p /extracted_xmls && \
    find . -name "package.xml" -exec cp --parents {} /extracted_xmls/ \;


####################
# BASE
####################
FROM ros:${ROS_DISTRO}-ros-base AS base

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

ENV HW_WS=/opt/hw_ws
RUN mkdir -p ${HW_WS}/src
WORKDIR ${HW_WS}

# Clone here repositories with packages for the modules like LiDAR or stereo camera

RUN apt-get update && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release" && \
    rm -rf build/ log/


ENV WS=/opt/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}

RUN echo "if [ -f /opt/ros/\${ROS_DISTRO}/setup.bash ]; then source /opt/ros/\${ROS_DISTRO}/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /root/.bashrc && \
    echo "if [ -f ${HW_WS}/install/setup.bash ]; then source ${HW_WS}/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> /root/.bashrc

COPY docker/entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]


####################
# DEV
####################
FROM base AS dev
ARG INSTALL_SIM_TOOLS=false

COPY --from=cacher /extracted_xmls/ ${WS}/src/

RUN apt-get update && \ 
    rosdep install --from-paths ${WS}/src/ ${HW_WS}/src/ --ignore-src --rosdistro ${ROS_DISTRO} -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN if [ "$INSTALL_SIM_TOOLS" = "true" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0 && \
        rm -rf /var/lib/apt/lists/*; \
    fi

CMD ["bash"]


####################
# PROD
####################
FROM base AS prod
ARG SIMULATION_PKGS

COPY --from=cacher /extracted_xmls/ ${WS}/src/

RUN apt-get update && rosdep install --from-paths ${WS}/src ${HW_WS}/src --ignore-src --rosdistro ${ROS_DISTRO} -r -y \
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
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

RUN rm -rf ${WS}/log/

CMD ["bash"]