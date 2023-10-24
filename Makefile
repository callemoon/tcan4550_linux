# SPDX-License-Identifier: GPL-2.0-only
#
#  Makefile for the TCAN4550 controller driver.
#

obj-m += tcan4550.o

KDIR = /opt/cclinux/3.2/sysroots/cortexa35-poky-linux/lib/modules/5.15.32-lts-next+g66633302ecf8/build

all:
	make -C $(KDIR)  M=$(shell pwd) modules

clean:
	make -C $(KDIR)  M=$(shell pwd) clean
