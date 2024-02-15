###
Network:
append to /etc/dhcpcd.conf
```
interface wlan0
static routers=10.1.1.1
static ip_address=10.1.1.2/24
```

### Install
vim, tmux, docker.io

sudo gpasswd -a $USER docker

