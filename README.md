# TCAN4550 Linux kernel driver

Tested with CAN FD Click 6 using a TCAN4550 chip https://www.mikroe.com/can-fd-6-click connected to a Raspberry Pi 4 running Pi OS Boookworm 64-bit.

## To connect to Raspberry Pi 4

Driver uses Raspberry Pi SPI 0 so use standard Raspberry Pi SPI 0 pins (MOSI pin 19, MISO pin 21, CS pin pin 24). GPIO25 (pin 22) is used for interrupt. GPIO21 (pin 40) is used for reset. TCAN board also needs both 3V3 and VBAT supply. (6-24v).

## To build driver on Raspberry Pi 4

If not already installed, install kernel headers: 'sudo apt-get install raspberrypi-kernel-headers'  
Call 'make'  

A device tree overlay is used to connect driver to SPI0 bus of Raspberry Pi. To build device tree overlay binary from .dts file.  

sudo dtc -@ -I dts -O dtb -o tcan4550-can0-overlay.dtbo tcan4550-can0-overlay.dts  

## To run driver on Raspberry Pi 4

To load and start driver run script: ./start_can.sh  

To stop and unload the driver run script: ./stop_can.sh  

Adjust bitrate by editing the start_can.sh script  

## To run driver on Raspberry Pi 5
This driver also works on Raspberry Pi 5 but due to a bug in the SPI driver for Raspberry Pi 5, MAX_SPI_BURST_TX_MESSAGES and MAX_SPI_BURST_RX_MESSAGES
must be set to 3 or lower. In contrary to Raspberry Pi 4, Raspberrby Pi 5 supports 32-bit SPI transfers.

## Performance test
Install can-utils with sudo apt-get install can-utils  

To generate bus load use cangen can0 -g0 (g0 means no gap between messages)  
To measure busload use canbusload can0@1000000  

TX >80%@1000kbit/s  
RX >90%@1000kbit/s  

## Additional supported functions

loopback - ip link set can0 type can loopback on (loop rx <=> tx pins on CAN controller internally)  
one-shot - ip link set can0 type can one-shot on (do not retransmit in case of transmit failure)  
listen-only - ip link set can0 type can listen-only on (do not send anything, not even ack for received packges)  

## Limitations
Does not support CAN FD

## References
https://www.kernel.org/doc/html/v6.6/networking/can.html



