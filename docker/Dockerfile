ARG UBUNTU_VERSION=22.04
ARG TARGET_ROS_DISTRO=humble
ARG BASE_IMAGE_HW_BUILDER
ARG BASE_IMAGE
ARG SIMULATION_PKGS="rover_sim rover_viz"


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
# HW-BUILDER
####################
FROM ${BASE_IMAGE_HW_BUILDER} AS hw-builder
ARG TARGET_ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_PYTHON_VERSION=3

RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common curl \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${TARGET_ROS_DISTRO}-ros-base \
    build-essential python3-colcon-common-extensions python3-rosdep git \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

ENV HW_WS=/opt/hw_ws
RUN mkdir -p ${HW_WS}/src
WORKDIR ${HW_WS}

########### Stereo Camera Setup ###########
ARG ZED_WRAPPER_REPOSITORY="https://github.com/stereolabs/zed-ros2-wrapper.git"
ARG ZED_WRAPPER_COMMIT="6545933af94d70922881654e6fb29d95e3a8f14f"

RUN mkdir -p ${HW_WS}/src/zed-ros2-wrapper \
    && cd ${HW_WS}/src/zed-ros2-wrapper \
    && git init \
    && git remote add origin ${ZED_WRAPPER_REPOSITORY} \
    && git fetch --depth 1 origin ${ZED_WRAPPER_COMMIT} \
    && git checkout FETCH_HEAD \
    && git submodule update --init --recursive \
    && cd ${HW_WS} \
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


####################
# BASE
####################
FROM ${BASE_IMAGE} AS base
ARG TARGET_ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV TARGET_ROS_DISTRO=${TARGET_ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV TZ=Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

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

ARG RPLIDAR_REPO=https://github.com/Slamtec/sllidar_ros2
ARG RPLIDAR_COMMIT=34300099fadfc772965962dec837bf436706188f

RUN git clone ${RPLIDAR_REPO} src/sllidar_ros2 && \
    cd src/sllidar_ros2 && \
    git checkout ${RPLIDAR_COMMIT}

RUN apt-get update && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --cmake-args=-DCMAKE_BUILD_TYPE=Release" && \
    rm -rf build/ log/

COPY --from=hw-builder ${HW_WS}/install ${HW_WS}/install
COPY --from=hw-builder ${HW_WS}/src ${HW_WS}/src

ENV WS=/opt/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}

RUN echo "if [ -f /opt/ros/\${TARGET_ROS_DISTRO}/setup.bash ]; then source /opt/ros/\${TARGET_ROS_DISTRO}/setup.bash; fi" >> /root/.bashrc && \
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
    rosdep install --from-paths ${WS}/src/ ${HW_WS}/src/ --ignore-src --rosdistro ${TARGET_ROS_DISTRO} -r -y && \
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

RUN apt-get update && rosdep install --from-paths ${WS}/src ${HW_WS}/src --ignore-src --rosdistro ${TARGET_ROS_DISTRO} -r -y \
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

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build"

RUN rm -rf ${WS}/log/

CMD ["bash"]