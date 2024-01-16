cmake_minimum_required(VERSION 3.5)
project(camera)

find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(CUDA REQUIRED)
find_package(catkin QUIET)
find_package(ament_cmake QUIET)
find_package(VPI 2.0)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)

link_directories(/usr/lib/aarch64-linux-gnu/tegra)

set(CMAKE_CXX_STANDARD 14)
include_directories(${CUDA_INCLUDE_DIRS})
add_definitions(-DROS2)

set(common_src src/image_converter.cpp src/ros_utils.cpp)

add_executable(camera_source src/camera_source.cpp ${common_src})
target_link_libraries(camera_source ${catkin_LIBRARIES} jetson-inference)

add_executable(camera_proc src/camera_proc.cpp ${common_src})
target_link_libraries(camera_proc ${catkin_LIBRARIES} jetson-inference)

ament_target_dependencies(camera_source rclcpp std_msgs sensor_msgs image_transport camera_info_manager)
install(TARGETS camera_source DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(camera_proc rclcpp std_msgs sensor_msgs image_transport cv_bridge OpenCV image_geometry)
install(TARGETS camera_proc DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
