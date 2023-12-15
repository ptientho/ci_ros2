# syntax=docker/dockerfile:1
FROM ros:galactic-ros-base-focal

# Minimal setup
RUN apt update \
 && apt install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

SHELL ["/bin/bash","-c"]

# fix packages
RUN apt-get update && apt-get remove libfprint-2-2 fprintd libpam-fprintd && apt-get clean && apt-get install -y libfprint-2-2 fprintd libpam-fprintd

# install dev tools
RUN apt update && apt install -y \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

# install some pip packages needed for testing
RUN python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools

# install required noetic packages and gazebo
RUN apt-get update && apt-get install -y git
RUN apt update && apt install -y ros-galactic-joint-state-publisher ros-galactic-robot-state-publisher \
    ros-galactic-cartographer ros-galactic-cartographer-ros ros-galactic-gazebo-plugins \
    ros-galactic-teleop-twist-keyboard ros-galactic-teleop-twist-joy ros-galactic-xacro \
    ros-galactic-nav2* ros-galactic-urdf ros-galactic-v4l2-camera ros-galactic-rviz2

WORKDIR /
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
#RUN source /opt/ros/galactic/setup.bash

# clone required packages
RUN git clone -b ros2-galactic --recursive https://github.com/rigbetellabs/tortoisebot.git
RUN git clone https://github.com/ptientho/tortoisebot_waypoints_ros2.git
RUN git clone https://github.com/ptientho/tortoisebot_waypoints_interface_ros2.git

# copy gazebo.launch.py
COPY gazebo.launch.py tortoisebot/tortoisebot_gazebo/launch/

# ignore tortoisebot_control build process
RUN touch tortoisebot/tortoisebot_control/COLCON_IGNORE

WORKDIR /ros2_ws
RUN source /opt/ros/galactic/setup.bash \
    && colcon build

# set DISPLAY variable
ENV DISPLAY=:1

RUN echo source /ros2_ws/install/setup.bash >> ~/.bashrc

#CMD /bin/bash -c "source install/setup.bash; ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True"