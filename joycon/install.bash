#!/usr/bin/env bash

set -e # Fail script if any command fails

if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo."
    exit 1
fi

echo "Build docker container"
docker build . -t joycon_driver:latest

echo "Prepare systemd service"
hostname=$(hostname)
namespace=$(hostname | tr '-' '_')

set +e # read returns 1 by default
IFS='' read -r -d '' camera_service <<EOF
[Unit]
Description=Joycon
After=docker.service
Requires=docker.service

[Service]
Type=simple
TimeoutStopSec=5
ExecStart=/usr/bin/docker run --runtime nvidia --rm --network host --privileged joycon_driver:latest /bin/bash -c ". install/setup.bash && SDL_GAMECONTROLLERCONFIG='$(< gamecontrollerdb.txt)' ros2 run joy game_controller_node --ros-args -r __ns:=/${namespace} -p device_id:=0"
Restart=always

[Install]
WantedBy=multi-user.target
EOF
set -e

echo "${camera_service}" > /etc/systemd/system/joycon.service

echo "Enable and start service"
systemctl enable joycon
systemctl start joycon

echo "Success!"
