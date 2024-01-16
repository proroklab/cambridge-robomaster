#!/usr/bin/env bash

set -e # Fail script if any command fails

if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo."
    exit 1
fi

echo "Build docker container"
docker build . -t cam_driver:latest

echo "Prepare systemd service"
hostname=$(hostname)
namespace=$(hostname | tr '-' '_')

set +e # read returns 1 by default
IFS='' read -r -d '' camera_service <<EOF
[Unit]
Description=Camera Stream 0
After=docker.service
Requires=docker.service
After=nvargus-daemon.service
Requires=nvargus-daemon.service

[Service]
ExecStart=/usr/bin/docker run --rm --runtime nvidia --net=host --ipc=host --pid=host -v /tmp/argus_socket:/tmp/argus_socket --hostname ${hostname} cam_driver:latest /bin/bash -c ". install/setup.bash && while true; do ros2 run gscam2 gscam_main --ros-args --params-file cam_param_jpg.yaml -r __ns:=/${namespace}/camera_0; done"
Restart=always

[Install]
WantedBy=multi-user.target
EOF
set -e

echo "Write systemd service"
echo "${camera_service}" > /etc/systemd/system/camera_stream_0.service

echo "Enable and start service"
systemctl enable camera_stream_0
systemctl start camera_stream_0

echo "Success!"
