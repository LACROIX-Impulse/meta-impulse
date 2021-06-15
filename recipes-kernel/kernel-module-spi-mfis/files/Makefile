obj-m := spi-mfis.o
# $src is a Kbuild variable. Using $SRC (defined above of course) doesn't work.
ccflags-y := -I$(src)/recipe-sysroot/usr/include/
#ccflags-y += -DDEBUG

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
