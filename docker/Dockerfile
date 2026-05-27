ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base AS base

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales git curl wget ca-certificates \
    build-essential python3-pip \
    python3-colcon-common-extensions python3-rosdep \
    iputils-ping pkg-config usbutils iproute2 g++ make cmake \
    can-utils \
    ros-${ROS_DISTRO}-ros2launch \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-ros2-socketcan \
    ros-${ROS_DISTRO}-can-msgs \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-action-msgs \
    ros-${ROS_DISTRO}-std-msgs \
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

RUN if [ "$INSTALL_SIM_TOOLS" = "true" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-rosbag2 \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        libgl1 libglib2.0-0 \
        && rm -rf /var/lib/apt/lists/*; \
    fi

CMD ["bash"]


FROM base AS prod
COPY ./src ${WS}/src

RUN apt-get update && rosdep install -y -r -i --from-paths src --rosdistro ${ROS_DISTRO} && rm -rf /var/lib/apt/lists/*

RUN touch ${WS}/src/indomitus_rover_sim/COLCON_IGNORE

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

CMD ["bash"]