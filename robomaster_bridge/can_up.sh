modprobe can
modprobe can-raw
modprobe mttcan
ip link set can0 type can bitrate 1000000 berr-reporting on restart-ms 100
ip link set can0 up
ifconfig can0 txqueuelen 1000
