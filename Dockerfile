# Uses the ROS Foxy as base image
FROM osrf/ros:foxy-desktop

# Shell to be used during the build process and the container's default.
SHELL ["/bin/bash", "-c"]

# Update the system.
RUN apt update && apt upgrade -y

# Install mavros and mavlink.
RUN apt update && DEBIAN_FRONTEND=noninteractive \
    && apt install -y ros-foxy-mavros ros-foxy-mavros-extras ros-foxy-mavros-msgs ros-foxy-mavlink \
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
    
# Install gazebo garden
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y lsb-release wget gnupg \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y gz-garden 

# Install ardupilot_gazebo
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y libgz-sim7-dev rapidjson-dev \
    && cd $HOME \
    && git clone https://github.com/ArduPilot/ardupilot_gazebo \
    && cd ardupilot_gazebo \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j4 \
    && export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH \
    && export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:GZ_SIM_RESOURCE_PATH \
    && echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> /root/.bashrc \
    && echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> /root/.bashrc

# Install robot_localization
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-foxy-robot-localization

# Install some tools.
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install tmux htop vim -y

# Python deps
COPY ./requirements.txt requirements.txt
RUN pip install -r requirements.txt

# Configure the environment.
RUN echo "set -g mouse on" >> /root/.tmux.conf
RUN echo "set-option -g history-limit 20000" >> /root/.tmux.conf
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
