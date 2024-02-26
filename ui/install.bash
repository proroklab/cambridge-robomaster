#!/usr/bin/env bash

set -e # Fail script if any command fails

if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo."
    exit 1
fi

if [ -z "$(docker images -q robomaster_ui:latest 2> /dev/null)" ]; then
    echo "Build docker container"
    docker build . -t robomaster_ui:latest
fi

echo "Prepare systemd service"
hostname=$(hostname)

set +e # read returns 1 by default
IFS='' read -r -d '' robomaster_ui <<EOF
[Unit]
Description=RoboMaster UI
After=docker.service
Requires=docker.service

[Service]
Type=simple
TimeoutStopSec=2
ExecStart=/usr/bin/docker run --rm --runtime nvidia --net=host --ipc=host -e JETSON_MODEL_NAME=JETSON_ORIN_NX --device /dev/i2c-7 --device /dev/gpiochip0 --hostname ${hostname} robomaster_ui:latest /bin/bash -c ". install/setup.bash && ros2 launch launch/ui.launch.py" 
Restart=always

[Install]
WantedBy=multi-user.target
EOF
set -e

echo "Write systemd service"
echo "${robomaster_ui}" > /etc/systemd/system/robomaster_ui.service

echo "Enable and start service"
systemctl enable robomaster_ui
systemctl start robomaster_ui

echo "Success!"

