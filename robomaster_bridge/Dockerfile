FROM dustynv/ros:humble-ros-base-l4t-r35.3.1

RUN apt-get update && apt-get install -y git nano

RUN mkdir /opt/robomaster
WORKDIR /opt/robomaster

RUN git clone https://github.com/janblumenkamp/robomaster_ros2_can.git --recurse
WORKDIR /opt/robomaster/robomaster_ros2_can
RUN . /opt/ros/humble/install/setup.sh && colcon build

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
