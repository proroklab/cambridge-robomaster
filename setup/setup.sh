# Rename hostname
sudo nmcli con add con-name robomaster type wifi ifname wlan0 ssid "robomaster" -- wifi-sec.key-mgmt wpa-psk wifi-sec.psk "**r0b0t**" ipv4.method manual ipv4.addresses 10.3.1.2/24 ipv4.gateway 10.3.1.1
#sudo usermod -aG docker nvidia
sudo apt update && apt depends nvidia-jetpack | awk '{print $2}' | uniq | xargs -I {} bash -c "sudo apt -o Dpkg::Options::="--force-confold" -y install {} ; sudo apt clean"
# gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=3840,height=2160,framerate=30/1' ! queue ! nvvidconv ! fakesink
# cmake, docker compose, tmux
