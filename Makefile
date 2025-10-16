all:
	$(MAKE) -C src/PCIE/aic8800/aic8800_fdrv
	$(MAKE) -C src/SDIO
	$(MAKE) -C src/USB/aic_btusb
	$(MAKE) -C src/USB/aic8800
