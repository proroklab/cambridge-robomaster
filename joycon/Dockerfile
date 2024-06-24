FROM dustynv/ros:humble-pytorch-l4t-r35.3.1

RUN apt-get update && apt-get install -y git vim cmake

RUN mkdir -p /opt/robomaster/src
WORKDIR /opt/robomaster/src
# Commit 038a12722bac3983d7fee462ad074a8f18c7bd5f
RUN git clone -b ros2 https://github.com/ros-drivers/joystick_drivers.git
# Commit b169c12768fd39fedda65bb34e0db057ccb0d13f
RUN git clone -b ros2 https://github.com/ros/diagnostics.git
RUN git clone -b humble https://github.com/ros2/launch.git
WORKDIR /opt/robomaster

RUN . /opt/ros/humble/install/setup.sh && rosdep install -y --from-paths . --ignore-src --rosdistro humble
RUN . /opt/ros/humble/install/setup.sh && colcon build --symlink-install

COPY cyclone_dds_profile_base.xml /opt/robomaster
COPY gamecontrollerdb.txt /opt/robomaster
ENV SDL_GAMECONTROLLERCONFIG="$(cat /opt/robomaster/gamecontrollerdb.txt)"

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=cyclone_dds_profile_base.xml
