#!/bin/bash
sudo slcan_attach -f -s6 -o -c /dev/ttyACM0
sudo slcand ttyACM0 slcan0
sudo ifconfig slcan0 up
