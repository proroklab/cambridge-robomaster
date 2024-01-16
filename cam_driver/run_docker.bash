docker run --runtime nvidia -it --rm \
                --network host \
                -v /tmp/argus_socket:/tmp/argus_socket \
                -v /etc/enctune.conf:/etc/enctune.conf \
                -v /etc/nv_tegra_release:/etc/nv_tegra_release \
                -v camera:/opt/cam_tmp/src/camera \
		-v /dev/shm:/dev/shm \
		--hostname $(cat /etc/hostname) \
		cam_driver_dnv:latest

# ros2 launch ros_deep_learning video_source.ros2.launch input:=csi://0 input_width:=1920 input_height:=1080
