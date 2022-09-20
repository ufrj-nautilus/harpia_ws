# Uses the ROS Humble Hawksbill as base image
FROM ros:humble-ros-base

# Shell to be used during the build process and the container's default.
SHELL ["/bin/bash", "-c"]

# Update the system.
RUN apt update && apt upgrade -y

# Install ROS Humble Hawksbill desktop
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install ros-humble-desktop-full -y

# Install mavros and mavlink.
RUN apt update && DEBIAN_FRONTEND=noninteractive \
    && apt install -y ros-humble-mavros ros-humble-mavros-extras ros-humble-mavlink \
    && wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh

# Install some tools.
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install tmux htop vim -y

# Configure the environment.
RUN echo "set -g mouse on" >> /root/.tmux.conf
RUN echo "set-option -g history-limit 20000" >> /root/.tmux.conf
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
