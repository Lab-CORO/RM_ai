FROM osrf/ros:noetic-desktop-full

# select bash as default shell
SHELL ["/bin/bash", "-c"]

# install catkin
RUN apt update && apt install python3-catkin-tools python3-wstool -y
RUN source /opt/ros/noetic/setup.bash
