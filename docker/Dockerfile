ARG BASE_IMAGE=stereolabs/zed:5.4-gl-devel-cuda12.8-ubuntu22.04

FROM ${BASE_IMAGE} AS base

ARG TARGET_ROS_DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive
ENV TARGET_ROS_DISTRO=${TARGET_ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common curl \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${TARGET_ROS_DISTRO}-ros-base \
    locales git curl wget ca-certificates \
    build-essential python3-pip \
    python3-colcon-common-extensions python3-rosdep \
    iputils-ping pkg-config usbutils iproute2 g++ make cmake \
    can-utils vim \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

RUN rosdep init || true && rosdep update


ENV HW_WS=/opt/hw_ws
RUN mkdir -p ${HW_WS}/src
WORKDIR ${HW_WS}

RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git ${HW_WS}/src/zed-ros2-wrapper \
    && find ${HW_WS}/src -name ".git" -type d -prune -exec rm -rf '{}' + \
    && find ${HW_WS}/src -name ".gitignore" -type f -delete \
    && find ${HW_WS}/src -name ".gitmodules" -type f -delete

RUN apt-get update && rosdep install --from-paths src --ignore-src --rosdistro ${TARGET_ROS_DISTRO} -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp/fake_gpu \
    && cp /usr/local/cuda/lib64/stubs/libcuda.so /tmp/fake_gpu/libcuda.so.1 \
    && cp /usr/local/cuda/lib64/stubs/libcuda.so /tmp/fake_gpu/libnvcuvid.so.1 \
    && cp /usr/local/cuda/lib64/stubs/libcuda.so /tmp/fake_gpu/libnvidia-encode.so.1

RUN /bin/bash -c "export LD_LIBRARY_PATH=/tmp/fake_gpu:\${LD_LIBRARY_PATH} && \
    source /opt/ros/${TARGET_ROS_DISTRO}/setup.bash && \
    colcon build --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_SHARED_LINKER_FLAGS='-Wl,--allow-shlib-undefined' \
    -DCMAKE_EXE_LINKER_FLAGS='-Wl,--allow-shlib-undefined'" \
    && rm -rf build/ log/ \
    && rm -rf /tmp/fake_gpu

ENV WS=/opt/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}

RUN echo "if [ -f /opt/ros/\${TARGET_ROS_DISTRO}/setup.bash ]; then source /opt/ros/\${TARGET_ROS_DISTRO}/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f ${HW_WS}/install/setup.bash ]; then source ${HW_WS}/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /root/.bashrc && \
    echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> /root/.bashrc

COPY docker/entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]


# ==========================================
# DEV STAGE
# ==========================================
FROM base AS dev

ARG INSTALL_SIM_TOOLS=false
COPY src/ /tmp/src_packages/src/
RUN if [ "$INSTALL_SIM_TOOLS" = "true" ]; then \
        cp -r /tmp/src_packages/src/* ${WS}/src/; \
    else \
        cd /tmp/src_packages/src && \
        for pkg in *; do \
            case "$pkg" in rover_sim|rover_viz) ;; *) cp -r "$pkg" ${WS}/src/ ;; esac; \
        done; \
    fi

RUN apt-get update && rosdep install --from-paths ${WS}/src ${HW_WS}/src --ignore-src --rosdistro ${TARGET_ROS_DISTRO} -r -y && rm -rf /var/lib/apt/lists/*

RUN if [ "$INSTALL_SIM_TOOLS" = "true" ]; then \
        apt-get update && apt-get install -y --no-install-recommends libgl1 libglib2.0-0 && rm -rf /var/lib/apt/lists/*; \
    fi

CMD ["bash"]


# ==========================================
# PROD STAGE
# ==========================================
FROM base AS prod

COPY src/ /tmp/src_packages/src/
RUN cd /tmp/src_packages/src && \
    for pkg in *; do \
        case "$pkg" in rover_sim|rover_viz) ;; *) cp -r "$pkg" ${WS}/src/ ;; esac; \
    done

RUN apt-get update && rosdep install --from-paths ${WS}/src ${HW_WS}/src --ignore-src --rosdistro ${TARGET_ROS_DISTRO} -r -y \
    -t build \
    -t buildtool \
    -t build_export \
    -t buildtool_export \
    -t exec \
    && rm -rf /var/lib/apt/lists/*

COPY docker/colcon_defaults.prod.yaml colcon_defaults.yaml

RUN /bin/bash -c "source /opt/ros/${TARGET_ROS_DISTRO}/setup.bash && source ${HW_WS}/install/setup.bash && colcon build" \
    && rm -rf build/ log/

CMD ["bash"]