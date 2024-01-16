#nmcli con add con-name robomaster type wifi ifname wlan0 ssid "robomaster" -- wifi-sec.key-mgmt wpa-psk wifi-sec.psk "**r0b0t**" ipv4.method manual ipv4.addresses 10.3.1.3/24 ipv4.gateway 10.3.1.1 ipv4.dns 10.3.1.1 ipv4.routes "192.168.0.0/24 10.3.1.1"
#nmcli con add con-name wired type ethernet ifname eth0 -- ipv4.method manual ipv4.addresses 10.3.0.3/24 ipv4.gateway 10.3.0.1

nmcli con add con-name robot-adhoc type wifi ifname wlan1 mode adhoc ssid "robomaster-adhoc" 802-11-wireless.mode IBSS -- ipv4.method manual ipv4.addresses 10.3.2.3/24
