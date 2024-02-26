#!/bin/bash
host_id=$(echo $(hostname) | sed 's/robomaster-//')
ip_last_octet=$(($host_id + 2))

ahDevName='wlan1' #we assume all devices use the same device name for ad-hoc interfaces
ahIp=10.3.2.$ip_last_octet
networkSSID='roboAdHoc' #can be anything, of no particular consequence
networkFreq='5200' #2412 or 5200  primary channel frequency (note bandwidths above 20MHz do not use this as a centre frequency; HT40+ 'adds' 20MHz to maximum frequency that would be used in HT20)
networkChannel='HT20' #[NOHT, HT20, HT40+, 80MHz] set channel bandwidth AND HT/VHT mode. Recommend sticking to HT modes.
networkIBSS='00:31:41:59:26:54' #MAC address used to identify the IBSS
hardwareRetries='1'

rfkill unblock all
ip link show dev $ahDevName || (echo "Could not find device" $ahDevName; exit 1)
ip link set $ahDevName down || (echo "Could not down device"; exit 1)
iw $ahDevName set type managed || (echo "Set type failed"; exit 1)
ip addr flush dev $ahDevName || (echo "Flush failed"; exit 1)
iwconfig $ahDevName retry short $hardwareRetries || (echo "Set short retries failed"; exit 1)
iwconfig $ahDevName retry long $hardwareRetries  || (echo "Set long retries failed"; exit 1)
iw reg set GB || (echo "Set regulatory domain failed"; exit 1)
iw $ahDevName set type ibss || (echo "Set type to adhoc failed"; exit 1)
ip link set $ahDevName up || (echo "Up device failed"; exit 1)
iw $ahDevName ibss join $networkSSID $networkFreq $networkChannel fixed-freq $networkIBSS  || (echo "Joining adhoc failed failed"; exit 1)
ip address add $ahIp/24 brd + dev $ahDevName
