FROM dustynv/ros:humble-pytorch-l4t-r35.3.1

RUN apt-get update && apt-get install -y git vim cmake

RUN mkdir -p /opt/robomaster/src
WORKDIR /opt/robomaster/src
COPY camera /opt/robomaster/src/camera
WORKDIR /opt/robomaster

RUN . /opt/ros/humble/install/setup.sh && colcon build --symlink-install

COPY cyclone_dds_profile_base.xml /opt/robomaster

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=cyclone_dds_profile_base.xml
