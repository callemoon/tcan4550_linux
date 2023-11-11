#!/bin/bash

sudo modprobe can_dev
sudo insmod tcan4550.ko

sudo dtoverlay tcan4550-can0-overlay.dtbo

sudo ip link set can0 up type can bitrate 1000000 restart-ms 1000
sudo ifconfig can0 txqueuelen 1000
