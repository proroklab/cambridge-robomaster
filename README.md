General information:
User nvidia, password nvidia

1) Change hostname
Modify /etc/hostname to the new hostname, then reboot
```
sudo nano /etc/hostname
>robomaster-1
sudo reboot
```

2) Network setup
This allows access to the internet through wgb
```
sudo nmcli con add con-name robomaster type wifi ifname wlan0 ssid "robomaster" -- wifi-sec.key-mgmt wpa-psk wifi-sec.psk "**r0b0t**" ipv4.method manual ipv4.addresses 10.3.1.x/24 ipv4.gateway 10.3.1.1 ipv4.dns 10.3.1.1 ipv4.routes "192.168.0.0/24 10.3.1.1"
sudo nmcli con add con-name wired type ethernet ifname eth0 -- ipv4.method manual ipv4.addresses 10.3.0.x/24 ipv4.gateway 10.3.0.1
```
where `x = robomaster-id + 2` (e.g. robomaster-0 gets infrastructure ip 10.3.1.2)

3) Install nvidia jetpack and other utilities
```
sudo apt update && apt depends nvidia-jetpack | awk '{print $2}' | uniq | xargs -I {} bash -c "sudo apt -o Dpkg::Options::="--force-confold" -y install {} ; sudo apt clean"
sudo apt install vim cmake tmux
```
This may take a while. Reboot after.

4) Set up docker
```
sudo usermod -aG docker nvidia # Add nvidia to docker group, otherwise requires sudo to execute
```

5) Set up CAN

6) Set up cameras
The cameras can be tested with the following command if logged in on a terminal in non-headless mode (i.e. with a monitor connected):
```
gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=3840,height=2160,framerate=30/1' ! queue ! nvvidconv ! fakesink
```

7) Adhoc setup
```
sudo nmcli con add con-name robot-adhoc type wifi ifname wlan1 mode adhoc ssid "robomaster-adhoc" 802-11-wireless.mode IBSS -- ipv4.method manual ipv4.addresses 10.3.2.3/24
```
