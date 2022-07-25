# Makefile for Boltek Lightning Detector Device Driver
# (c) Boltek 2008
# Author: Patrick McManus <mcmanus@ducksong.com>
#
# GPL v2
#
# make
# insmod boltek.ko

ifneq ($(KERNELRELEASE),)
obj-m   := boltek.o
else
KDIR := /lib/modules/$(shell uname -r)/build
modules modules_install clean help:
	make -C $(KDIR) M=$(shell pwd) $@
endif


