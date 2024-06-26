---
layout: default
title: Software Setup
nav_order: 6
---

# Software
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

## Flashing

Flash the BSP following the instructions of the BSP Readme.

After flashing has succeeded, connect via USB to the Jetson and power it on. The Jetson should boot up and appear as USB to Ethernet device on the host computer, assigning an IP to the host. Try pinging `ping 192.168.55.1` to verify the connection.

Copy the SSH keys of the host to the Jetson by running `ssh-copy-id nvidia@192.168.55.1`. The password will be prompted, which is `nvidia`. This skip can be skipped, in that case the passowrd needs to be entered every time the SSH connection is established.

Next, we copy the content of this folder to the Jetson. This can be accomplished by running `copy_to_jetson_initial.bash` from the root of the repository.

Now, SSH into the Jetson by running `ssh nvidia@192.168.55.1`. Change the directory to the `Robot` folder, and follow the instructions below.

General information Jetsons:
User nvidia, password nvidia

## Change hostname
Allow NVIdia user to use sudo without password, modify /etc/hostname to the new hostname, then reboot
```
sudo ./setup/sudocfg.bash
sudo vim /etc/hostname
>robomaster-1
sudo reboot
```

## Network setup
This allows access to the internet through wgb
```
sudo ./setup/nmcli.bash
> Connection 'robomaster' (56d6c150-46eb-4028-be24-2faa2eec9486) successfully added.
> Connection 'wired' (ba5c3b46-94df-44e2-bd49-3cdeba1f25d1) successfully added.
```

You can test the setup by pinging the router:
```
ping 10.3.1.1
> PING 10.3.1.1 (10.3.1.1) 56(84) bytes of data.
64 bytes from 10.3.1.1: icmp_seq=1 ttl=64 time=4.82 ms
64 bytes from 10.3.1.1: icmp_seq=2 ttl=64 time=89.2 ms
```

And by showing the network config, where the `wlan0` configuration should look like this:
```
ip a
> 4: wlan0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether f4:3b:d8:e0:33:a5 brd ff:ff:ff:ff:ff:ff
    altname wlP1p1s0
    inet 10.3.1.4/24 brd 10.3.1.255 scope global noprefixroute wlan0
       valid_lft forever preferred_lft forever
    inet6 fe80::680c:3a7f:11b1:cc8b/64 scope link noprefixroute 
       valid_lft forever preferred_lft forever
```

Optionally, if the router is configured and connected to the internet, you can test pinging a website:
```
ping google.com
> PING google.com (216.58.204.78) 56(84) bytes of data.
64 bytes from lhr48s49-in-f14.1e100.net (216.58.204.78): icmp_seq=1 ttl=54 time=19.3 ms
```

The ethernet is set up with ip address 10.3.0.x. After connecting a cable to another device, the eth0 interface should show up.
```
ip a 
> 3: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc pfifo_fast state UP group default qlen 1000
    link/ether 48:b0:2d:d8:bf:74 brd ff:ff:ff:ff:ff:ff
    altname enP8p1s0
    inet 10.3.0.4/24 brd 10.3.0.255 scope global noprefixroute eth0
       valid_lft forever preferred_lft forever
    inet6 fe80::65bb:1eb:71e1:986/64 scope link noprefixroute 
       valid_lft forever preferred_lft forever
```

## Install nvidia jetpack and other utilities
```
sudo ./setup/apt.bash
```
This may take a while. Reboot after.

## Set up docker
```
sudo ./docker/add_group.bash
sudo ./docker/setup_docker_compose.bash
sudo cp docker/daemon.json /etc/docker
sudo service docker restart
```

Optional: Pull images for CAN, cam and UI:
```
sudo ./docker/pull_images.sh
```
If this step is skipped, the installation commands will build from scratch.

## Set up CAN
```
cd robomaster_bridge
sudo ./install.bash
```

## Set up cameras
The cameras can be tested with the following command if logged in on a terminal in non-headless mode (i.e. with a monitor connected):
```
gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=3840,height=2160,framerate=30/1' ! queue ! nvvidconv ! fakesink
gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=3840,height=2160,framerate=30/1' ! queue ! nvvidconv ! xvimagesink -e
```

Setup:
```
cd cam_driver
sudo ./install.bash
```

## Set up UI
```
cd ui
sudo ./install.bash
```

## Adhoc setup
```
sudo ./adhoc/nmcli.bash
```

## Adding users
```
./setup/adduser.bash
```
