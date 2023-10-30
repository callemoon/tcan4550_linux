# SPDX-License-Identifier: GPL-2.0-only
#
#  Makefile for the TCAN4550 controller driver.
#

obj-m += tcan4550.o

KDIR = /lib/modules/$(shell uname -r)/build

all:
	make -C $(KDIR)  M=$(shell pwd) modules

clean:
	make -C $(KDIR)  M=$(shell pwd) clean
