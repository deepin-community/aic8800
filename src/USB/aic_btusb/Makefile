MODULE_NAME = aic_btusb

CONFIG_AIC8800_BTUSB_SUPPORT = m
CONFIG_SUPPORT_VENDOR_APCF = n
# Need to set fw path in BOARD_KERNEL_CMDLINE
CONFIG_USE_FW_REQUEST = n

ifeq ($(CONFIG_SUPPORT_VENDOR_APCF), y)
obj-$(CONFIG_AIC8800_BTUSB_SUPPORT) := $(MODULE_NAME).o aic_btusb_external_featrue.o
else
obj-$(CONFIG_AIC8800_BTUSB_SUPPORT) := $(MODULE_NAME).o
endif

ccflags-$(CONFIG_SUPPORT_VENDOR_APCF) += -DCONFIG_SUPPORT_VENDOR_APCF
#$(MODULE_NAME)-y := aic_btusb_ioctl.o\
#	aic_btusb.o \

ccflags-$(CONFIG_USE_FW_REQUEST) += -DCONFIG_USE_FW_REQUEST


# Platform support list
CONFIG_PLATFORM_UBUNTU ?= y

ifeq ($(CONFIG_PLATFORM_UBUNTU), y)
ccflags-$(CONFIG_PLATFORM_UBUNTU) += -DCONFIG_PLATFORM_UBUNTU
KDIR  := /lib/modules/$(shell uname -r)/build
PWD   := $(shell pwd)
KVER := $(shell uname -r)
MODDESTDIR := /lib/modules/$(KVER)/kernel/drivers/net/wireless/aic8800
SUBARCH = $(shell uname -m | sed -e s/i.86/i386/ -e s/armv.l/arm/ -e s/aarch64/arm64/)
ARCH ?= $(SUBARCH)
CROSS_COMPILE :=
endif

all: modules
modules:
	make -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

install:
	mkdir -p $(MODDESTDIR)
	install -p -m 644 $(MODULE_NAME).ko  $(MODDESTDIR)/
	/sbin/depmod -a ${KVER}
	echo $(MODULE_NAME) >> /etc/modules-load.d/aic_bt.conf

uninstall:
	rm -rfv $(MODDESTDIR)/$(MODULE_NAME).ko
	/sbin/depmod -a ${KVER}
	rm -f /etc/modules-load.d/aic_bt.conf

clean:
	rm -rf *.o *.ko *.o.* *.mod.* modules.* Module.* .a* .o* .*.o.* *.mod .tmp* .cache.mk .Module.symvers.cmd .modules.order.cmd

