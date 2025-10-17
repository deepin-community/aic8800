CONFIG_AIC8800_BTLPM_SUPPORT = m

obj-$(CONFIG_AIC8800_BTLPM_SUPPORT) := aic8800_btlpm.o

ccflags-y += -I$(srctree)/$(src)/../aic8800_bsp

# Platform support list
CONFIG_PLATFORM_UBUNTU ?= y


CONFIG_SUPPORT_LPM ?= n
CONFIG_AUTO_PM ?= n

aic8800_btlpm-y := \
	aic_bluetooth_main.o \
	rfkill.o \

aic8800_btlpm-$(CONFIG_SUPPORT_LPM) += lpm.o

ccflags-y += -DAIC_TRACE_INCLUDE_PATH=$(src)

ccflags-$(CONFIG_AUTO_PM) += -DCONFIG_AUTO_PM

ifeq ($(CONFIG_PLATFORM_UBUNTU), y)
KDIR  ?= /lib/modules/$(shell uname -r)/build
PWD   ?= $(shell pwd)
KVER ?= $(shell uname -r)
MODDESTDIR ?= /lib/modules/$(KVER)/kernel/drivers/net/wireless/
ARCH ?= x86_64
CROSS_COMPILE ?=
endif


all: modules
modules:
	make -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

install:
	mkdir -p $(MODDESTDIR)
	install -p -m 644 $(MODULE_NAME).ko  $(MODDESTDIR)
	/sbin/depmod -a ${KVER}

uninstall:
	rm -rfv $(MODDESTDIR)/$(MODULE_NAME).ko
	/sbin/depmod -a ${KVER}

clean:
	rm -rf *.o *.ko *.o.* *.mod.* modules.* Module.* .a* .o* .*.o.* *.mod .tmp* .cache.mk


