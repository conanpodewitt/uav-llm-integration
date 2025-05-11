########################################
# Stage 1: Builder Stage (Heavy Build Steps)
########################################
FROM ubuntu:24.04 AS builder
ENV DEBIAN_FRONTEND=noninteractive

########################################
# Install OS Dependencies & Tools
########################################
RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    sudo \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

########################################
# Setup ROS 2 Jazzy Repository & Key
########################################
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

########################################
# Setup Gazebo Harmonic Repository & Key
########################################
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg > /dev/null
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

########################################
# Install ROS, Gazebo, and ROS Tools
########################################
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    gz-harmonic \
    python3-colcon-common-extensions \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-sick-scan-xd \
    && rm -rf /var/lib/apt/lists/*

########################################
# Install Additional Packages
########################################
RUN pip3 install --break-system-packages evdev numpy opencv-python requests

########################################
# Install and Build AriaCoda
########################################
RUN apt-get update && apt-get install -y \
    git \
    make \
    g++ \
    doxygen \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/reedhedges/AriaCoda.git /opt/AriaCoda && \
    cd /opt/AriaCoda && \
    make -j6 && \
    sudo make install && \
    ldconfig

# Ensure shared libraries are found
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> /etc/bash.bashrc

########################################
# Global ROS Environment Setup
########################################
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

########################################
# Create and Configure User for GUI Applications
########################################
RUN useradd -m pioneer-container && \
    echo "pioneer-container:password" | chpasswd && \
    adduser pioneer-container sudo
RUN echo 'echo "Usage: ros2 launch master_launch [sim.launch.py|actual.launch.py]"; history -s "ros2 launch master_launch "' \
    >> /home/pioneer-container/.bashrc

########################################
# Switch to New User and Setup Workspace
########################################
USER pioneer-container
WORKDIR /home/pioneer-container

# Create ROS Workspace and copy in project files
RUN mkdir -p ~/uav-llm-integration/src
COPY --chown=pioneer-container:pioneer-container src/ /home/pioneer-container/uav-llm-integration/src/
COPY --chown=pioneer-container:pioneer-container conf/system_prompt.txt /home/pioneer-container/uav-llm-integration/

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && cd ~/uav-llm-integration && colcon build"
RUN echo "source ~/uav-llm-integration/install/setup.bash" >> ~/.bashrc

########################################
# Stage 2: Final Runtime Stage
########################################
FROM builder

# Set simulation-specific environment variables
ENV GZ_SIM_RESOURCE_PATH=/home/pioneer-container/uav-llm-integration/install/uav_sim/share/ \
    XDG_RUNTIME_DIR=/tmp/runtime-pioneer-container

# Set variables
ARG SAFETY_STOP_DISTANCE
ARG MAX_FORWARD_SPEED
ARG MAX_REVERSE_SPEED
ARG MAX_TURN_LEFT_SPEED
ARG MAX_TURN_RIGHT_SPEED
ARG AREA_THRESHOLD
ARG AREA_MATCH_TOL
ARG MAX_TRACKED
ARG LLM_URL
ARG LLM_MODEL
ARG LLM_TEMPERATURE
ARG LLM_MAX_RETRIES
ARG TIME_DIFF_THRESHOLD
ARG SYSTEM_INTERVAL
ARG POISON_RATE
ARG MIRAGE_RATE
ENV SAFETY_STOP_DISTANCE=${SAFETY_STOP_DISTANCE} \
    MAX_FORWARD_SPEED=${MAX_FORWARD_SPEED} \
    MAX_REVERSE_SPEED=${MAX_REVERSE_SPEED} \
    MAX_TURN_LEFT_SPEED=${MAX_TURN_LEFT_SPEED} \
    MAX_TURN_RIGHT_SPEED=${MAX_TURN_RIGHT_SPEED} \
    AREA_THRESHOLD=${AREA_THRESHOLD} \
    AREA_MATCH_TOL=${AREA_MATCH_TOL} \
    MAX_TRACKED=${MAX_TRACKED} \
    LLM_URL=${LLM_URL} \
    LLM_MODEL=${LLM_MODEL} \
    LLM_TEMPERATURE=${LLM_TEMPERATURE} \
    LLM_MAX_RETRIES=${LLM_MAX_RETRIES} \
    TIME_DIFF_THRESHOLD=${TIME_DIFF_THRESHOLD} \
    SYSTEM_INTERVAL=${SYSTEM_INTERVAL} \
    POISON_RATE=${POISON_RATE} \
    MIRAGE_RATE=${MIRAGE_RATE}
    
# Set the default entrypoint to bash
ENTRYPOINT ["/bin/bash"]