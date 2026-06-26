ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base AS base

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales git curl wget ca-certificates \
    build-essential python3-pip \
    python3-colcon-common-extensions python3-rosdep \
    iputils-ping pkg-config usbutils iproute2 g++ make cmake \
    ros-${ROS_DISTRO}-rcpputils \
    ros-${ROS_DISTRO}-class-loader \
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
ARG RPLIDAR_REPO=https://github.com/Slamtec/rplidar_sdk
ARG RPLIDAR_COMMIT=99478e5fb90de3b4a6db0080acacd373f8b36869

RUN git clone ${RPLIDAR_REPO} src/sllidar_ros2 && \
    cd src/sllidar_ros2 && \
    git checkout ${RPLIDAR_COMMIT}



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

RUN apt-get update && \
    rosdep install --from-paths ${WS}/src ${HW_WS}/src --ignore-src --rosdistro ${ROS_DISTRO} -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN if [ "$INSTALL_SIM_TOOLS" = "true" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglib2.0-0 && \
        rm -rf /var/lib/apt/lists/*; \
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

RUN apt-get update && rosdep install --from-paths ${WS}/src ${HW_WS}/src --ignore-src --rosdistro ${ROS_DISTRO} -r -y \
    -t build \
    -t buildtool \
    -t build_export \
    -t buildtool_export \
    -t exec \
    && rm -rf /var/lib/apt/lists/*

COPY src/ /tmp/src_packages/src/
RUN mkdir -p ${WS}/src && \
    cd /tmp/src_packages/src && \
    for pkg in *; do \
        case "$pkg" in \
            rover_sim|rover_viz) ;; \
            *) cp -r "$pkg" ${WS}/src/ ;; \
        esac; \
    done

# IMPORTANT: we had really a lot of troubles with segmentation fault
# evething below is done to prevent this fault. DONT TOUCH IT!
COPY docker/colcon_defaults.prod.yaml colcon_defaults.yaml
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
    --parallel-workers 1 \
    --packages-select indomitus_interfaces \
    --cmake-args \
      -DCMAKE_BUILD_PARALLEL_LEVEL=1 \
      -DCMAKE_C_FLAGS='-O0' \
      -DCMAKE_CXX_FLAGS='-O0'"

# Then build the rest normally
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${HW_WS}/install/setup.bash && \
    colcon build \
    --parallel-workers 1 \
    --packages-skip indomitus_interfaces \
    --cmake-args \
      -DCMAKE_BUILD_PARALLEL_LEVEL=1 \
      -DCMAKE_C_FLAGS='-O0' \
      -DCMAKE_CXX_FLAGS='-g0 -O1'"

CMD ["bash"]
