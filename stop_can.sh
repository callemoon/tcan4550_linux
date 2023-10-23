#!/bin/bash

sudo ifconfig can0 down

sudo rmmod tcan4550.ko
sudo modprobe can_dev --remove

sudo dtoverlay -r tcan4550-can0-overlay