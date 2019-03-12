#!/bin/bash
sudo /usr/bin/slcand -f -s6 -o -c /dev/ttyEvoCan
sudo ip link set slcan0 up
