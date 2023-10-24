# TCAN4550 Linux kernel driver

Tested with CAN FD Click 6 using a TCAN4550 chip https://www.mikroe.com/can-fd-6-click connected to a Raspberry Pi 4 running Pi OS bullseye 64-bit.

## To connect to Raspberry Pi 4

Driver uses Raspberry Pi SPI 0 so use standard Raspberry Pi SPI 0 pins (MOSI pin 19, MISO pin 21, CS pin pin 24). GPIO25 (pin 22) is used for interrupt. GPIO21 (pin 40) is used for reset. TCAN board also needs both 3V3 and VBAT supply. (6-24v).

## To build driver on Raspberry Pi 4

Install kernel headers: 'sudo apt-get install raspberrypi-kernel-headers'  
Call 'make'  

A device tree overlay is used to connect driver to SPI0 bus of Raspberry Pi. To build device tree overlay binary from .dts file.  

sudo dtc -@ -I dts -O dtb -o tcan4550-can0-overlay.dtbo tcan4550-can0-overlay.dts  

To load and start driver run script: ./start_can  

To stop and unload the driver run script: ./stop_can  

Adjust bitrate by editing the start_can script  

## Performance test
Install can-utils with sudo apt-get install can-utils  

To generate bus load use cangen can0 -g0 (g0 means no gap between messages)  
To measure busload use canbusload can0@1000000  

TX ~95%@1000kbit/s  
RX >60%@1000kbit/s  

## Limitations
Does not support CAN FD  
Does not support bus off or bus warning  
Does not support suspend/resume


