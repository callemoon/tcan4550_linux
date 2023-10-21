# tcan4550_linux_kernel_driver

To build driver on Raspberry Pi 4

Install kernel headers: 'sudo apt-get install raspberrypi-kernel-headers'
Call 'make'

To build device tree overlay binary from .dts file

sudo dtc -@ -I dts -O dtb -o tcan4550-can0-overlay.dtbo tcan4550-can0-overlay.dts

To start driver

Run script: ./start_can
