# Makefile for OMAP3 ISP driver

ifdef CONFIG_ARCH_OMAP3410
isp-mod-objs += \
	isp.o ispccdc.o ispmmu.o
else
isp-mod-objs += \
	isp.o ispccdc.o ispmmu.o \
	isppreview.o ispresizer.o isph3a.o isphist.o isp_af.o ispcsi2.o

obj-$(CONFIG_VIDEO_OMAP34XX_ISP_PREVIEWER) += \
	omap_previewer.o

obj-$(CONFIG_VIDEO_OMAP34XX_ISP_RESIZER) += \
	omap_resizer.o

obj-$(CONFIG_VIDEO_OMAP34XX_TVP_ISP_GLUE) += \
        omap_tvp_isp.o
endif

obj-$(CONFIG_VIDEO_OMAP3) += isp-mod.o
