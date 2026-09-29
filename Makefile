saa716x_core-objs	:= saa716x_pci.o	\
				saa716x_i2c.o	\
				saa716x_cgu.o	\
				saa716x_msi.o	\
				saa716x_dma.o	\
				saa716x_vip.o	\
				saa716x_aip.o	\
				saa716x_phi.o	\
				saa716x_boot.o	\
				saa716x_fgpi.o	\
				saa716x_adap.o	\
				saa716x_gpio.o	\
				saa716x_greg.o	\
				saa716x_rom.o	\
				saa716x_spi.o	\

saa716x_ff-objs	:= saa716x_ff_main.o	\
				saa716x_ff_cmd.o	\
				saa716x_ff_ir.o

saa716x_capture-objs := saa716x_cap.o	\
				saa716x_v4l2.o	\
				saa716x_alsa.o	\
				saa716x_debugfs.o

obj-$(CONFIG_SAA716X_CORE)		+= saa716x_core.o
obj-$(CONFIG_DVB_SAA716X_BUDGET)	+= saa716x_budget.o
obj-$(CONFIG_DVB_SAA716X_HYBRID)	+= saa716x_hybrid.o
obj-$(CONFIG_DVB_SAA716X_FF)		+= saa716x_ff.o
obj-$(CONFIG_SAA716X_CAPTURE)		+= saa716x_capture.o
obj-$(CONFIG_TDA19978)			+= tda19978.o
obj-$(CONFIG_MST3367)                   += mst3367-drv.o
obj-$(CONFIG_ECHDCAP_RX)                += echdcap_rx.o

EXTRA_CFLAGS = -Idrivers/media/tuners/ -Idrivers/media/dvb-frontends/
