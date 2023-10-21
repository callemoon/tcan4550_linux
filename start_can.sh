#!/bin/bash

sudo modprobe can_dev
sudo insmod tcan4550.ko

sudo dtoverlay tcan4550-can0-overlay.dtbo

sudo ip link set can0 up type can bitrate 500000
sudo ifconfig can0 txqueuelen 350
