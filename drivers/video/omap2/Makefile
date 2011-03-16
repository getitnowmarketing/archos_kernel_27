obj-$(CONFIG_FB_OMAP2) += omapfb.o
omapfb-y := omapfb-main.o omapfb-sysfs.o omapfb-ioctl.o

obj-$(CONFIG_PANEL_GENERIC) += panel-generic.o
obj-$(CONFIG_PANEL_SHARP_LS037V7DW01) += panel-sharp-ls037v7dw01.o

obj-$(CONFIG_CTRL_BLIZZARD) += ctrl-blizzard.o
obj-$(CONFIG_PANEL_N800) += panel-n800.o
obj-$(CONFIG_PANEL_SAMSUNG_LMS480KC01) += panel-samsung-wvga48.o
obj-$(CONFIG_PANEL_TPO_WVGA48) += panel-tpo-wvga48.o
obj-$(CONFIG_PANEL_CPT_WVGA48) += panel-cpt-wvga48.o
obj-$(CONFIG_PANEL_ARCHOS_HDMI) += panel-archos-hdmi.o
obj-$(CONFIG_PANEL_ARCHOS_EXTDAC) += panel-archos-extdac.o

ifeq ($(CONFIG_FB_OMAP2_DEBUG),y)
	EXTRA_CFLAGS += -DDEBUG
endif
