# Explicitly use ARM64 version of ROS Humble
FROM --platform=linux/arm64 arm64v8/ros:humble

# Remove existing ROS repos and keys completely
RUN rm -rf /etc/apt/sources.list.d/ros* && \
    rm -rf /usr/share/keyrings/ros* && \
    # Clean apt cache
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install basic tools first
RUN apt-get update && \
    apt-get install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    ca-certificates \
    software-properties-common

# Set up the new ROS repository with updated key
RUN mkdir -p /usr/share/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update

# Install ROS packages and dependencies
RUN apt-get install -y \
    git \
    python3-pip \
    ssh-client \
    ros-humble-rosbridge-server \
    python3-rosdep \
    python3-colcon-common-extensions \
    usbutils \
    can-utils \
    iproute2 \
    kmod \
    python3-numpy \
    libboost-python-dev \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install moteus pyusb opencv-python pydocstyle

# Create user
RUN useradd trickfire \
    --shell /bin/bash \
    --create-home \
    --no-log-init \
    -G video

# Set up bashrc (create a basic one if the file doesn't exist)
RUN echo "source /opt/ros/humble/setup.bash" >> /home/trickfire/.bashrc

# Set working directory
WORKDIR /home/trickfire

# Switch to the user
USER trickfire