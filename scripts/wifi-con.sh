#!/bin/sh 
iface=wlan0

#stop any persistent wireless wpa2 authentication sessions
killall wpa_supplicant

#shut down wireless interface
ifconfig $iface down

#set working mode of wireless device
iwconfig $iface mode Managed

#enable interface
ifconfig $iface up

#apply dragonfly3 settings to device
wpa_supplicant -B -D nl80211 -i $iface -c ./wpa-zhaw.conf -dd

#obtain an IP address
dhclient $iface
