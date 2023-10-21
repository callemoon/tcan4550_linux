# TCAN4550 linux kernel driver

Tested with CAN FD Click 6 using a TCAN4550 chip https://www.mikroe.com/can-fd-6-click connected to a Raspberry Pi 4.

## To connect to Raspberry Pi 4

Driver uses Raspberry Pi SPI 0 so use standard Raspberry Pi SPI 0 pins (MISO, MOSI, CS). GPIO25 is used for interrupt. GPIO21 is used for reset. TCAN board needs both 3V3 and VBAT supply. (6-24v). 

## To build driver on Raspberry Pi 4

Install kernel headers: 'sudo apt-get install raspberrypi-kernel-headers'
Call 'make'

A device tree overlay is used to connect driver to SPI0 bus of Raspberry Pi. To build device tree overlay binary from .dts file.

sudo dtc -@ -I dts -O dtb -o tcan4550-can0-overlay.dtbo tcan4550-can0-overlay.dts

To load and start driver

Run script: ./start_can

To unload and stop driver

Run script: ./stop_can

## performance

To generate load use cangen can0 -g0
To receive data use candump can0

TX ~70%@500kbit/s
RX >60%@500kbit/s

