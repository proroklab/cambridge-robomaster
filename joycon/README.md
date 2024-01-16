ros2 run joy game_controller_node --ros-args -r __ns:=/robomaster_1/joy -p device_id:=0
ros2 run joy joy_enumerate_devices
. install/setup.bash && SDL_GAMECONTROLLERCONFIG=$(cat gamecontrollerdb.txt) ros2 run joy game_controller_node --ros-args -r __ns:=/robomaster_1/joy -p device_id:=0
