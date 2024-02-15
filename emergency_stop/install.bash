#!/usr/bin/env bash

set -e # Fail script if any command fails

if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo."
    exit 1
fi

echo "Build docker container"
docker build . -t emergency_stop:latest

echo "Prepare system service and config files"
set +e # read returns 1 by default
read -r -d '' btn_service <<EOF
[Unit]
Description=Emergency Stop
After=docker.service
Requires=docker.service
After=systemd-networkd.service
Requires=systemd-networkd

[Service]
ExecStart=/usr/bin/docker run --rm --network host --hostname $(hostname) --privileged emergency_stop:latest /bin/bash -c ". install/setup.bash && ros2 run emergency_stop emergency_stop_button"
KillSignal=SIGINT
Restart=always

[Install]
WantedBy=multi-user.target
EOF

set -e

echo "Write systemd service"
echo "${btn_service}" > /etc/systemd/system/emergency_stop.service

echo "Enable and start systemd service"
systemctl enable emergency_stop
systemctl start emergency_stop

echo "Success! Reboot required."

