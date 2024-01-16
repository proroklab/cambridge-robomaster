```
ros2 run camera video_source --ros-args -r __ns:=/robomaster_1 -p resource:=csi://0 -p width:=1920 -p height:=1080
ros2 run camera video_source --ros-args -r __ns:=/robomaster_1/camera_0 -p camera_info_name:=front_camera -p camera_info_url:=file:///ros2_ws/front_camera.yaml
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
colcon build --packages-select camera --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 run --prefix="gdb --args" camera video_source --ros-args -r __ns:=/robomaster_1 -p resource:=csi://0 -p width:=1920 -p height:=1080
```
