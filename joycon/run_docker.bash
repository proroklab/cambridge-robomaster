docker run --runtime nvidia -it --rm \
                --network host \
		--hostname $(cat /etc/hostname) \
		--privileged \
		joycon_driver:latest
#		-v /run/udev/data:/run/udev/data \
#		--device /dev/input/by-id/usb-Inno_GamePad.._Inno_GamePad.._0000001-event-joystick \
#		--device /dev/input/by-id/usb-Inno_GamePad.._Inno_GamePad.._0000001-joystick \
#--device /dev/input/js0 \
#		--device /dev/input/event0 \
		
# ros2 launch ros_deep_learning video_source.ros2.launch input:=csi://0 input_width:=1920 input_height:=1080
