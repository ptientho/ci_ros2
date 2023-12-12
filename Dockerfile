# syntax=docker/dockerfile:1
FROM ros:galactic-ros-base

SHELL ["/bin/bash","-c"]

# install required noetic packages and gazebo
RUN apt-get update && apt-get install -y git \
    ros-galactic-joint-state-publisher ros-galactic-robot-state-publisher \
    ros-galactic-cartographer ros-galactic-cartographer-ros ros-galactic-gazebo-plugins \
    ros-galactic-teleop-twist-keyboard  ros-galactic-teleop-twist-joy ros-galactic-xacro \
    ros-galactic-nav2* ros-galactic-urdf ros-galactic-v4l2-camera \
    ros-galactic-gazebo-ros-pkgs python3-colcon-common-extensions

WORKDIR /
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
RUN source /opt/ros/galactic/setup.bash

# clone required packages
RUN git clone -b ros2-galactic --recursive https://github.com/rigbetellabs/tortoisebot.git
RUN git clone https://github.com/ptientho/tortoisebot_waypoints_ros2.git
RUN git clone https://github.com/ptientho/tortoisebot_waypoints_interface_ros2.git

WORKDIR /ros2_ws
RUN source /opt/ros/galactic/setup.bash \
    && colcon build

# set DISPLAY variable
ENV DISPLAY=:1

RUN echo source /ros2_ws/install/setup.bash >> ~/.bashrc

# launch Gazebo
CMD /bin/bash -c "ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True; rostest tortoisebot_waypoints waypoints_test.test --reuse-master"