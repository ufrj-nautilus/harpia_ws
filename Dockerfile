# Uses the ROS Humble as base image
FROM ros:humble-ros-base-jammy

# Shell to be used during the build process and the container's default.
SHELL ["/bin/bash", "-c"]

# Update and upgrade system.
RUN apt update && DEBIAN_FRONTEND=noninteractive apt upgrade -y

# Python deps
COPY ./requirements.txt requirements.txt
RUN apt update && DEBIAN_FRONTEND=noninteractive \
    && apt install -y python3-pip \
    && pip install -r requirements.txt

# Gazebo Garden
RUN apt update && DEBIAN_FRONTEND=noninteractive \
    && apt upgrade -y \
    && apt install -y lsb-release wget gnupg \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt update \
    && apt install -y gz-garden

# Install robot_localization
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-robot-localization

# Install rtabmap
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-humble-rtabmap-ros

# Install mavros and mavlink.
RUN apt update && DEBIAN_FRONTEND=noninteractive \
    && apt upgrade -y \
    && apt install -y ros-humble-rqt ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs ros-humble-mavlink \
    && wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm -rf install_geographiclib_datasets.sh

# Install ardupilot and ardupilot_gz
RUN apt update && DEBIAN_FRONTEND=noninteractive \
    && apt upgrade -y \
    && apt install -y default-jre socat \
    && cd /root \
    && git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git \
    && cd /root/Micro-XRCE-DDS-Gen \
    && ./gradlew assemble \
    && export PATH=$PATH:/root/Micro-XRCE-DDS-Gen/scripts \
    && echo "export PATH=$PATH:/root/Micro-XRCE-DDS-Gen/scripts" >> /root/.bashrc \
    && cd /root \
    && apt install -y python3-future python3-serial python-is-python3 gdb \
    && pip install pexpect PyYAML mavproxy waftools pexpect \
    && mkdir -p /root/catkin_ws/src \
    && cd /root/catkin_ws/src \
    && wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos \
    && vcs import --recursive < ros2.repos \
    && cd /root/catkin_ws \
    && rosdep update \
    && rosdep install --rosdistro humble --from-paths src -i -r -y \
    && source /opt/ros/humble/setup.sh \
    && colcon build \
    && export PATH=$PATH:/root/catkin_ws/src/ardupilot/Tools/autotest \
    && echo "export PATH=$PATH:/root/catkin_ws/src/ardupilot/Tools/autotest" >> /root/.bashrc \
    && cd /root/catkin_ws/src \
    && wget https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos \
    && vcs import --recursive < ros2_gz.repos \
    && export GZ_VERSION=garden \
    && echo "export GZ_VERSION=garden" >> /root/.bashrc \
    && cd /root/catkin_ws \
    && apt update \
    && rosdep update \
    && rosdep install --rosdistro humble --from-paths src -i -r -y \
    && source /opt/ros/humble/setup.sh \
    && colcon build \
    && sim_vehicle.py -w -v ArduCopter

# Configure environment
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y tmux htop vim
RUN echo 'source /opt/ros/humble/setup.bash' >> root/.bashrc
RUN echo 'source /root/catkin_ws/install/setup.bash' >> root/.bashrc
RUN echo "set -g mouse on" >> /root/.tmux.conf
RUN echo "set-option -g history-limit 20000" >> /root/.tmux.conf
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
