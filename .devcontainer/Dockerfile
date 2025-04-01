# Use the official ROS 2 Humble image
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Install useful tools and packages
RUN apt update && apt install -y \
    git \
    curl \
    build-essential \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-pip \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    && apt clean

# Set default shell
SHELL ["/bin/bash", "-c"]

# Create workspace and set working directory
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Automatically source ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
