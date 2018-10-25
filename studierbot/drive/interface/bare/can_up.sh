#!/bin/bash
sudo /usr/bin/slcand -f -s6 -o -c /dev/ttyEvoCan
sudo ifconfig slcan0 up
