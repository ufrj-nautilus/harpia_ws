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
    && apt install -y ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs ros-humble-mavlink \
    && wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm -rf install_geographiclib_datasets.sh

# Install ArduPilot
RUN cd /root \
    && git clone https://github.com/ufrj-nautilus/ardupilot.git \
    && cd ardupilot \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && . /root/.profile \
    && git checkout $(git tag -l | grep Copter | tail -n1) \
    && git submodule update --init --recursive \
    && cd Tools/autotest \
    # && cd ArduCopter \
    && sim_vehicle.py -w

# Install ardupilot_gazebo
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y rapidjson-dev \
    && cd $HOME \
    && git clone https://github.com/ArduPilot/ardupilot_gazebo \
    && cd ardupilot_gazebo \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j4 \
    && export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH \
    && export IGN_GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:IGN_GAZEBO_RESOURCE_PATH \
    && echo 'export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}' >> /root/.bashrc \
    && echo 'export IGN_GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${IGN_GAZEBO_RESOURCE_PATH}' >> /root/.bashrc

# Install some tools.
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install tmux htop vim -y

# Configure the environment.
RUN echo "set -g mouse on" >> /root/.tmux.conf
RUN echo "set-option -g history-limit 20000" >> /root/.tmux.conf
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
