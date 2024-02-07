#!/bin/bash
host_id=$(echo $(hostname) | sed 's/robomaster-//')
ip_last_octet=$(($host_id + 2))

nmcli con add con-name robomaster type wifi ifname wlan0 ssid "robomaster" -- wifi-sec.key-mgmt wpa-psk wifi-sec.psk "**r0b0t**" ipv4.method manual ipv4.addresses 10.3.1.$ip_last_octet/24 ipv4.gateway 10.3.1.1 ipv4.dns 10.3.1.1 ipv4.routes "192.168.0.0/24 10.3.1.1"
nmcli con add con-name wired type ethernet ifname eth0 -- ipv4.method manual ipv4.addresses 10.3.0.$ip_last_octet/24 ipv4.gateway 10.3.0.1

