FROM nvidia/cuda:12.4.0-base-ubuntu22.04

# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
SHELL [ "/bin/bash" , "-c" ]

# Install ROS2 Humble
RUN apt update \
 && apt install -y --no-install-recommends curl \
 && apt install -y --no-install-recommends gnupg2 \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y net-tools gedit \
 && apt-get install -y wget
 
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update \
 && apt install -y ros-humble-desktop

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update && apt-get install -y gazebo
RUN apt-get install -y ros-humble-gazebo-ros-pkgs

RUN apt-get install -y python3-rosdep \
 && apt-get install -y python3-colcon-common-extensions

RUN apt-get install -y git

RUN apt-get install -y ros-humble-gazebo-ros2-control \
                       ros-humble-xacro 

RUN mkdir -p /Turtlebot_lab/src

WORKDIR /Turtlebot_lab

RUN source /opt/ros/humble/setup.bash  \
    && colcon build \
    && source install/setup.bash

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /Turtlebot_lab/install/setup.bash" >> ~/.bashrc
