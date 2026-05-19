ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    git curl wget ca-certificates \
    build-essential python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    iputils-ping pkg-config \
    ros-${ROS_DISTRO}-ros2launch \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-ros2-socketcan \
    ros-${ROS_DISTRO}-can-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-diagnostic-msgs \
    can-utils \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    usbutils \
    iproute2 \
    g++ make cmake \
    && rm -rf /var/lib/apt/lists/*
    
    # ros-${ROS_DISTRO}-joint-state-publisher-gui \
    # ros-${ROS_DISTRO}-rviz2 \
    # ros-${ROS_DISTRO}-rosbag2 \
    # ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-ros-gz-sim \
    # ros-${ROS_DISTRO}-ros-gz-bridge ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    # libgl1 libglib2.0-0 \

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

CMD ["bash"]