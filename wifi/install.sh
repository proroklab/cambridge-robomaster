#cp adhoc-wifi-setup.timer /etc/systemd/system
cp adhoc-wifi-setup.service /etc/systemd/system
systemctl enable adhoc-wifi-setup
systemctl start adhoc-wifi-setup
