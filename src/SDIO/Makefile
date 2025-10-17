CONFIG_AIC8800_BTLPM_SUPPORT := m
CONFIG_AIC8800_WLAN_SUPPORT := m
CONFIG_AIC_WLAN_SUPPORT := m

obj-$(CONFIG_AIC8800_BTLPM_SUPPORT) += aic8800_btlpm/
obj-$(CONFIG_AIC8800_WLAN_SUPPORT) += aic8800_fdrv/
obj-$(CONFIG_AIC_WLAN_SUPPORT) += aic8800_bsp/

MAKEFLAGS +=-j$(shell nproc)

########## config option ##########
export CONFIG_USE_FW_REQUEST = n
export CONFIG_PREALLOC_RX_SKB = y
export CONFIG_PREALLOC_TXQ = y
export CONFIG_OOB = n
export CONFIG_GPIO_WAKEUP = n
export CONFIG_RESV_MEM_SUPPORT = y
###################################

########## Platform support list ##########
export CONFIG_PLATFORM_UBUNTU = y

ifeq ($(CONFIG_PLATFORM_UBUNTU), y)
KDIR  = /lib/modules/$(shell uname -r)/build
PWD   = $(shell pwd)
KVER = $(shell uname -r)
MODDESTDIR = /lib/modules/$(KVER)/kernel/drivers/net/wireless/aic8800
SUBARCH = $(shell uname -m | sed -e s/i.86/i386/ -e s/armv.l/arm/ -e s/aarch64/arm64/)
ARCH ?= $(SUBARCH)
CROSS_COMPILE ?=
endif
###########################################

all: modules
modules:
	make -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

install:
	mkdir -p $(MODDESTDIR)
	install -p -m 644 aic8800_bsp/aic8800_bsp.ko  $(MODDESTDIR)/
	install -p -m 644 aic8800_fdrv/aic8800_fdrv.ko  $(MODDESTDIR)/
	install -p -m 644 aic8800_btlpm/aic8800_btlpm.ko  $(MODDESTDIR)/
	/sbin/depmod -a ${KVER}

uninstall:
	rm -rfv $(MODDESTDIR)/aic8800_bsp.ko
	rm -rfv $(MODDESTDIR)/aic8800_fdrv.ko
	rm -rfv $(MODDESTDIR)/aic8800_btlpm.ko
	/sbin/depmod -a ${KVER}

clean:
	cd aic8800_bsp/;make clean;cd ..
	cd aic8800_fdrv/;make clean;cd ..
	cd aic8800_btlpm/;make clean;cd ..
	rm -rf modules.order Module.symvers .modules.order.cmd .Module.symvers.cmd .tmp_versions/

