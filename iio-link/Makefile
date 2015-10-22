ifneq ($(KERNELRELEASE),)
obj-m := iio-link.o
else
KDIR := /home/jayvee/linux-adi
all:
	make -C $(KDIR) M=$(PWD) modules ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
clean:
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers  modul*
endif
