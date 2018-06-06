#!/bin/bash

socat udp4-datagram:255.255.255.255:11412,bind=:11412,range=192.168.131.0/24,broadcast,so-bindtodevice=enp1s0 pty,link=/dev/ttycan0 &
#socat udp4-datagram:224.0.0.1:11412,bind=:11412,range=192.168.131.0/24,ip-add-membership=224.0.0.1:192.168.131.1 pty,link=/dev/ttycan0 &
sleep 1
slcand -o -c -F -s5 /dev/ttycan0 vcan0 &
sleep 1
ifconfig vcan0 up
sleep 1
ip link set vcan0 txqueuelen 1000
wait
