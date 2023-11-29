# adjust docker hostname in connect_robomaster service
cp camera_stream_0.service /etc/systemd/system
systemctl enable camera_stream_0
systemctl start camera_stream_0
