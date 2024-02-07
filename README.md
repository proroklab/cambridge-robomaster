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
sudo ./setup/nmcli.bash
```

3) Install nvidia jetpack and other utilities
```
sudo ./setup/apt.bash
```
This may take a while. Reboot after.

4) Set up docker
```
sudo ./docker/add_group.bash
sudo ./docker/setup_docker_compose.bash
sudo cp docker/daemon.json /etc/docker
sudo service docker restart
```

5) Set up CAN
If not building from scratch, you can pull from the registry:
```
docker image pull 10.3.0.3:5000/robomaster_bridge:latest
```

```
cd robomaster_bridge
sudo ./install.bash
```

6) Set up cameras
The cameras can be tested with the following command if logged in on a terminal in non-headless mode (i.e. with a monitor connected):
```
gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=3840,height=2160,framerate=30/1' ! queue ! nvvidconv ! fakesink
```

Setup:
If not building from scratch, you can pull from the registry:
```
docker image pull 10.3.0.3:5000/cam_driver_dnv:latest
```

```
cd cam_driver
sudo ./install.bash
```

7) Adhoc setup
```
sudo ./adhoc/nmcli.bash
```
