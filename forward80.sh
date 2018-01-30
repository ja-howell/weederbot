#!/bin/bash
echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward

# Don't drop initial connections on port 80
sudo iptables -A FORWARD -i wlan0 -o br0 -p tcp --syn --dport 80 -m conntrack --ctstate NEW -j ACCEPT

# Don't drop communication
sudo iptables -A FORWARD -i wlan0 -o br0 -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT
sudo iptables -A FORWARD -i br0 -o wlan0 -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT

# Set up network address translation
sudo iptables -t nat -A PREROUTING -i wlan0 -p tcp --dport 80 -j DNAT --to-destination 192.168.1.13
sudo iptables -t nat -A POSTROUTING -o br0 -p tcp --dport 80 -d 192.168.1.13 -j SNAT --to-source 192.168.1.100

