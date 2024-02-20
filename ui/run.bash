docker run --runtime nvidia --rm \
                --network host \
		-v .:/opt/robomaster \
		-e JETSON_MODEL_NAME=JETSON_ORIN_NX \
		--device /dev/i2c-7 \
		--device /dev/gpiochip0 \
		--hostname $(cat /etc/hostname) \
		robomaster_ui:latest /bin/bash -c "python3 demo.py"
