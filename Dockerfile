# Uses the ROS Foxy as base image
FROM ros:foxy-ros-base

# Shell to be used during the build process and the container's default.
SHELL ["/bin/bash", "-c"]

# Update the system.
RUN apt update && apt upgrade -y

# Install ROS Foxy desktop
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install ros-foxy-desktop ignition-edifice -y

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

# Install ardupilot_gazebo
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y rapidjson-dev libignition-gazebo5-dev \
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

# Harpia simulator
RUN apt install gazebo11 libgazebo11-dev -y \
    && echo 'source /opt/ros/foxy/setup.bash' >> /root/.bashrc \
    && echo 'source $HOME/catkin_ws/install/setup.bash' >> /root/.bashrc \
    && echo 'source /usr/share/gazebo/setup.bash' >> /root/.bashrc \
    && echo 'export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${PWD}/src/harpia_simulator' >> /root/.bashrc \
    && echo 'export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/harpia_simulator/models' >> /root/.bashrc

# Install some tools.
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install tmux htop vim -y

# Configure the environment.
RUN echo "set -g mouse on" >> /root/.tmux.conf
RUN echo "set-option -g history-limit 20000" >> /root/.tmux.conf
RUN mkdir -p /root/catkin_ws/src
RUN pip install -r requirements.txt
WORKDIR /root/catkin_ws
