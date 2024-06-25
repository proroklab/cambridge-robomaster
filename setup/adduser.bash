#!/bin/bash

# Check if the script is run as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root or with sudo"
    exit 1
fi

read -p "Enter a user name: " username

# check length
if [ ${#username} -lt 4 ]; then
    echo "Error: Username too short"
    exit 1
fi

read -p "Add user ${username}? (y/n)" confirm
if [[ $confirm =~ [yY] ]]; then
    adduser ${username}
    usermod -aG sudo,docker ${username}
    echo "${username} ALL=(ALL:ALL) NOPASSWD: ALL" | tee /etc/sudoers.d/${username}
fi
