# Uses the ROS Humble as base image
FROM osrf/ros:humble-desktop-full

# Shell to be used during the build process and the container's default.
SHELL ["/bin/bash", "-c"]

# Update the system.
RUN apt update && apt upgrade -y

# Install mavros and mavlink.
RUN apt update && DEBIAN_FRONTEND=noninteractive \
    && apt install -y ros-humble-rqt ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs ros-humble-mavlink \
    && wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm -rf install_geographiclib_datasets.sh

# Install robot_localization
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-robot-localization

# Install Gazebo deps
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros ros-humble-gazebo-plugins

# Python deps
# COPY ./requirements.txt requirements.txt
# RUN pip install -r requirements.txt

# Configure environment
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install tmux htop vim -y
RUN echo 'source /opt/ros/humble/setup.bash' >> $HOME/.bashrc
RUN echo 'source /usr/share/gazebo/setup.bash' >> $HOME/.bashrc
RUN echo "set -g mouse on" >> /root/.tmux.conf
RUN echo "set-option -g history-limit 20000" >> /root/.tmux.conf
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
