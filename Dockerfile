# Dockerfile
FROM osrf/ros:jazzy-desktop-noble

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Basic dev tools + ROS build tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl wget nano vim tmux \
    build-essential cmake pkg-config \
    python3-pip python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    # helpful ROS tools
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-tf2-tools \

 && rm -rf /var/lib/apt/lists/*

# rosdep init/update (safe to run even if already initialized)
RUN rosdep init || true && rosdep update

# Create a default workspace location
ENV WS=/inspection_ws
RUN mkdir -p ${WS}/src

# Copy entrypoint that sources ROS + overlay
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR ${WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
