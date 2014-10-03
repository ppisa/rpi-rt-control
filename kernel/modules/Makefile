# Setup environment parameters

# architekture
ARCH=arm

# The target tool-chain prefix
#CROSS_COMPILE=/media/Data/KERNEL/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/arm-linux-gnueabihf-

# Directory where compiled modukes are installed
#INSTALL_MOD_PATH=/target/lib/modules/$(shell uname -r)/extra

MY_KERNEL_VERSION=$(shell uname -r)
MY_KERNEL_BUILD=/lib/modules/$(MY_KERNEL_VERSION)/build

my_all:
	make -C $(MY_KERNEL_BUILD) M=`pwd` modules


obj-m = rpi_gpio_irc_module.o

#clear compilation outputs
clear:
	rm -rf Module.symvers servoPi_modul.o servoPi_modul.ko servoPi_modul.mod.c modules.order servoPi_modul.mod.o .servoPi_modul.ko.cmd .tmp_versions .servoPi_modul.o.cmd .servoPi_modul.mod.o.cmd

