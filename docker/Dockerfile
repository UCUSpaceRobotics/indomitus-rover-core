ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base AS base

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales git curl wget ca-certificates \
    build-essential python3-pip \
    python3-colcon-common-extensions python3-rosdep \
    iputils-ping pkg-config usbutils iproute2 g++ make cmake \
    can-utils \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

RUN rosdep init || true && rosdep update

ENV WS=/opt/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}
ENV ROS_DISTRO=${ROS_DISTRO}

RUN echo "if [ -f /opt/ros/\${ROS_DISTRO}/setup.bash ]; then source /opt/ros/\${ROS_DISTRO}/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /root/.bashrc && \
    echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> /root/.bashrc

COPY docker/entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]


FROM base AS dev

ARG INSTALL_SIM_TOOLS=false

COPY src/ /tmp/src_packages/src/
RUN mkdir -p ${WS}/src && \
    if [ "$INSTALL_SIM_TOOLS" = "true" ]; then \
        cp -r /tmp/src_packages/src/* ${WS}/src/; \
    else \
        cd /tmp/src_packages/src && \
        for pkg in *; do \
            case "$pkg" in \
                rover_sim|rover_viz) ;; \
                *) cp -r "$pkg" ${WS}/src/ ;; \
            esac; \
        done; \
    fi

RUN apt-get update && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y && rm -rf /var/lib/apt/lists/*

RUN if [ "$INSTALL_SIM_TOOLS" = "true" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0 \
        && rm -rf /var/lib/apt/lists/*; \
    fi

CMD ["bash"]


FROM base AS prod

COPY src/ /tmp/src_packages/src/
RUN mkdir -p ${WS}/src && \
    cd /tmp/src_packages/src && \
    for pkg in *; do \
        case "$pkg" in \
            rover_sim|rover_viz) ;; \
            *) find "$pkg" -name "package.xml" -exec cp --parents {} ${WS}/src/ \; ;; \
        esac; \
    done

RUN apt-get update && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y && rm -rf /var/lib/apt/lists/*
# RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
#     --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
#     apt-get update && rosdep install --from-paths ${WS}/src --ignore-src --rosdistro ${ROS_DISTRO} -r -y

COPY src/ /tmp/src_packages/src/
RUN mkdir -p ${WS}/src && \
    cd /tmp/src_packages/src && \
    for pkg in *; do \
        case "$pkg" in \
            rover_sim|rover_viz) ;; \
            *) cp -r "$pkg" ${WS}/src/ ;; \
        esac; \
    done


COPY docker/colcon_defaults.prod.yaml colcon_defaults.yaml
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && CXXFLAGS='-g0 -O1' colcon build --parallel-workers 1"

CMD ["bash"]