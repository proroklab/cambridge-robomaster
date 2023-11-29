# change hostname
# adjust docker hostname in connect_robomaster service
cp connect_robomaster.service /etc/systemd/system
cp can.conf /etc/modules-load.d
cp 80-can.network /etc/systemd/network
#cp 80-can.link /etc/systemd/network
mv /etc/modprobe.d/denylist-mttcan.conf .
systemctl enable systemd-networkd
systemctl start systemd-networkd
systemctl enable connect_robomaster
systemctl start connect_robomaster
