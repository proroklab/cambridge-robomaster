FROM dustynv/ros:humble-ros-base-l4t-r35.3.1

RUN apt-get update && apt-get install -y git nano

RUN apt-get install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
 gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
 libgstreamer-plugins-base1.0-dev

RUN mkdir -p /opt/robomaster/src
WORKDIR /opt/robomaster/src

RUN git clone https://github.com/clydemcqueen/gscam2.git
###
COPY gscam_failure_exit.patch /opt/robomaster/src/gscam2
WORKDIR /opt/robomaster/src/gscam2
RUN git apply gscam_failure_exit.patch
WORKDIR /opt/robomaster/src
###
RUN git clone https://github.com/ptrmu/ros2_shared.git -b master
WORKDIR /opt/robomaster

RUN . /opt/ros/humble/install/setup.sh && rosdep install -y --from-paths . --ignore-src
RUN . /opt/ros/humble/install/setup.sh && colcon build

COPY cyclone_dds_profile_base.xml /opt/robomaster
COPY cam_param_jpg.yaml /opt/robomaster
COPY cam_param_raw.yaml /opt/robomaster
COPY front_camera.yaml /opt/robomaster

ENV ROS_DOMAIN_ID=0
#ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#ENV CYCLONEDDS_URI=cyclone_dds_profile_base.xml
