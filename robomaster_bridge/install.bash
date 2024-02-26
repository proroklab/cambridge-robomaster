#!/usr/bin/env bash

set -e # Fail script if any command fails

if [[ $UID != 0 ]]; then
    echo "Please run this script with sudo."
    exit 1
fi

if [ -z "$(docker images -q robomaster_bridge:latest 2> /dev/null)" ]; then
    echo "Build docker container"
    docker build . -t robomaster_bridge:latest
fi

echo "Prepare system service and config files"
set +e # read returns 1 by default
read -r -d '' camera_service <<EOF
[Unit]
Description=Connect Robomaster
After=docker.service
Requires=docker.service
After=systemd-networkd.service
Requires=systemd-networkd

[Service]
ExecStartPre=ifconfig can0 txqueuelen 1000
ExecStart=/usr/bin/docker run --rm --runtime nvidia --network host --hostname $(hostname) robomaster_bridge:latest /bin/bash -c ". install/setup.bash && ros2 launch robomaster_can_ros_bridge/launch/bridge.launch.py"
KillSignal=SIGINT
Restart=always

[Install]
WantedBy=multi-user.target
EOF

read -r -d '' can_modules <<EOF
can
can_raw
mttcan
EOF

read -r -d '' can_network <<EOF
[Match]
Name=can0

[CAN]
BitRate=1M
RestartSec=100ms
EOF

set -e

echo "Write systemd service"
echo "${camera_service}" > /etc/systemd/system/robomaster_bridge.service

echo "${can_modules}" > /etc/modules-load.d/can.conf

echo "${can_network}" > /etc/systemd/network/80-can.network

echo "Delete mttcan blacklisting"
set +e # Allow error if it does not exist
mv /etc/modprobe.d/denylist-mttcan.conf .
set -e

echo "Enable systemd-networkd to bring up can on boot"
systemctl enable systemd-networkd
systemctl start systemd-networkd

echo "Enable and start robomaster bridge service"
systemctl enable robomaster_bridge
systemctl start robomaster_bridge

echo "Success! Reboot required."

# Not sure if needed

