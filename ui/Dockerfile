FROM dustynv/ros:humble-ros-base-l4t-r35.3.1

RUN apt-get update && apt-get install -y git nano

RUN mkdir /opt/robomaster
WORKDIR /opt/robomaster

RUN pip3 install luma.oled
RUN pip3 install Jetson.GPIO

COPY ui /opt/robomaster/
WORKDIR /opt/robomaster/src
RUN git clone https://github.com/proroklab/ros2_robomaster_msgs.git

WORKDIR /opt/robomaster
RUN . /opt/ros/humble/install/setup.bash && colcon build --symlink-install

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
