# Uses the ROS Humble as base image
FROM osrf/ros:humble-desktop-full

# Shell to be used during the build process and the container's default.
SHELL ["/bin/bash", "-c"]

# Update the system.
RUN apt update && apt upgrade -y

# Install mavros and mavlink.
RUN apt update && DEBIAN_FRONTEND=noninteractive \
    && apt install -y ros-humble-rq tros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs ros-humble-mavlink \
    && wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm -rf install_geographiclib_datasets.sh

# Install ArduPilot
RUN cd /root \
    && git clone https://github.com/ufrj-nautilus/ardupilot.git \
    && cd ardupilot \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && git checkout $(git tag -l | grep Copter | tail -n1) \
    && git submodule update --init --recursive \
    && cd ArduCopter \
    && echo 'export PATH=$PATH:/root/ardupilot/Tools/autotest' >> /root/.profile \
    && echo 'export PATH=/usr/lib/ccache:$PATH' >> /root/.profile \
    && echo 'export PYTHONPATH=:/usr/local/lib/python3.8/dist-packages:$PYTHONPATH' >> /root/.profile \
    && echo 'export PATH=/root/.local/bin:$PATH' >> /root/.profile \
    && . /root/.profile \
    && python3 ../Tools/autotest/sim_vehicle.py -w

# Install ardupilot_gazebo
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y libignition-gazebo6-dev rapidjson-dev \
    && cd $HOME \
    && git clone https://github.com/ArduPilot/ardupilot_gazebo -b fortress \
    && cd ardupilot_gazebo \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j4 \
    && echo 'export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc \
    && echo 'export IGN_GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${IGN_GAZEBO_RESOURCE_PATH}' >> ~/.bashrc

# Install robot_localization
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-robot-localization

# Install Gazebo deps
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros ros-humble-gazebo-plugins

# Python deps
COPY ./requirements.txt requirements.txt
RUN pip install -r requirements.txt

# Configure environment
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install tmux htop vim -y
RUN echo 'source /opt/ros/humble/setup.bash' >> $HOME/.bashrc
RUN echo 'source /usr/share/gazebo/setup.bash' >> $HOME/.bashrc
RUN echo "set -g mouse on" >> /root/.tmux.conf
RUN echo "set-option -g history-limit 20000" >> /root/.tmux.conf
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
