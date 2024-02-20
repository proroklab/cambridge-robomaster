docker run --runtime nvidia -it --rm \
                --network host \
		-e JETSON_MODEL_NAME=JETSON_ORIN_NX \
		--device /dev/i2c-7 \
		--device /dev/gpiochip0 \
		--hostname $(cat /etc/hostname) \
		robomaster_ui:latest
		#-v .:/opt/robomaster \
