#
# Makefile for the linux kernel.
#

# Common support


obj-y := irq.o id.o io.o sdrc.o sdrc2xxx.o control.o mux.o prcm.o\
		devices.o serial.o gpmc.o timer-gp.o \

obj-$(CONFIG_OMAP_MCBSP) += mcbsp.o

# Functions loaded to SRAM
obj-$(CONFIG_ARCH_OMAP3)		+= sram34xx.o

# SMS/SDRC
obj-$(CONFIG_ARCH_OMAP2)		+= sdrc2xxx.o
# obj-$(CONFIG_ARCH_OMAP3)		+= sdrc3xxx.o

# Power Management
ifeq ($(CONFIG_PM),y)
obj-y					+= pm.o
obj-$(CONFIG_ARCH_OMAP2)		+= pm24xx.o
obj-$(CONFIG_ARCH_OMAP2420)		+= sleep242x.o
obj-$(CONFIG_ARCH_OMAP2430)		+= sleep243x.o
obj-$(CONFIG_PM_DEBUG)			+= pm-debug.o
endif

# SmartReflex driver
obj-$(CONFIG_OMAP_SMARTREFLEX)		+= smartreflex.o

# Clock framework
obj-$(CONFIG_ARCH_OMAP2)		+= clock24xx.o
#obj-$(CONFIG_ARCH_OMAP3)		+= clock34xx.o
ifeq ($(CONFIG_OMAP3_PM),)
       obj-y += clock.o powerdomain.o clockdomain.o
       # Functions loaded to SRAM
       obj-$(CONFIG_ARCH_OMAP2) += sram24xx.o
       # Clock framework
       obj-$(CONFIG_ARCH_OMAP3) += clock34xx.o pm34xx.o sleep34xx.o
       # Power Management
       obj-$(CONFIG_PM) += pm.o 
else
       obj-y += prcm_34xx.o pm_34xx.o clock_34xx.o sram-fn_34xx.o resource_34xx.o constraint_34xx.o pm_idle_34xx.o sleep_34xx.o pm_sysfs.o prcm-debug.o
       # Power Management
endif

obj-$(CONFIG_MPU_BRIDGE)		+= dspbridge.o

# DSP
obj-$(CONFIG_OMAP_MMU_FWK)	+= mmu_mach.o
obj-$(CONFIG_OMAP_MBOX_FWK)	+= mailbox_mach.o
mailbox_mach-objs		:= mailbox.o
mmu_mach-objs			:= mmu.o

# Specific board support
obj-$(CONFIG_MACH_OMAP_GENERIC)		+= board-generic.o
obj-$(CONFIG_MACH_OMAP_H4)		+= board-h4.o board-h4-mmc.o
obj-$(CONFIG_MACH_OMAP_2430SDP)		+= board-2430sdp.o \
					   board-2430sdp-flash.o \
					   hsmmc.o \
					   usb-musb.o \
					   usb-ehci.o
obj-$(CONFIG_MACH_OMAP2EVM)		+= board-omap2evm.o \
					   hsmmc.o
obj-$(CONFIG_MACH_OMAP_3430SDP)		+= board-3430sdp.o \
					   hsmmc.o \
					   usb-musb.o \
					   usb-ehci.o \
					   board-3430sdp-flash.o
obj-$(CONFIG_MACH_OMAP3EVM)		+= board-omap3evm.o \
					   hsmmc.o \
					   usb-musb.o usb-ehci.o \
					   board-omap3evm-flash.o
obj-$(CONFIG_MACH_OMAP3_BEAGLE)		+= board-omap3beagle.o \
					   usb-musb.o usb-ehci.o \
					   hsmmc.o
obj-$(CONFIG_MACH_OMAP_LDP)		+= board-ldp.o \
					   hsmmc.o \
					   board-ldp-flash.o \
					   usb-musb.o
obj-$(CONFIG_MACH_OMAP_ZOOM2)		+= board-zoom2.o \
					   hsmmc.o \
					   board-ldp-flash.o \
					   usb-musb.o \
					   board-zoom-debugboard.o
obj-$(CONFIG_MACH_OMAP_APOLLON)		+= board-apollon.o \
					   board-apollon-mmc.o	\
					   board-apollon-keys.o
obj-$(CONFIG_MACH_NOKIA_N800)		+= board-n800.o board-n800-flash.o \
					   board-n800-mmc.o board-n800-bt.o \
					   board-n800-audio.o board-n800-usb.o \
					   board-n800-dsp.o \
					   board-n800-camera.o
obj-$(CONFIG_MACH_NOKIA_N810)		+= board-n810.o
obj-$(CONFIG_MACH_OVERO)		+= board-overo.o \
					   hsmmc.o \
					   usb-musb.o \
					   usb-ehci.o
obj-$(CONFIG_MACH_ARCHOS_G6H)		+= board-archosg6h.o
obj-$(CONFIG_MACH_ARCHOS_G6S)		+= board-archosg6s.o
obj-$(CONFIG_MACH_ARCHOS_A5H)		+= board-archos-a5h.o
obj-$(CONFIG_MACH_ARCHOS_A5S)		+= board-archos-a5s.o
obj-$(CONFIG_MACH_ARCHOS_A5ST)		+= board-archos-a5st.o \
					   archos-lcd-cpt-wvga48.o
obj-$(CONFIG_MACH_ARCHOS)		+= board-archosg6.o \
					   board-archosg7.o \
					   archos-hsmmc.o \
					   archos-wakeup.o \
					   archos-battery.o \
					   archos-audio.o \
					   archos-wifi-bt.o \
					   tps65023.o \
				           archos-keys.o \
					   archos-usb2sata.o \
					   archos-usb-ehci.o \
					   archos-usb-musb.o \
					   archos-usb-android.o \
					   archos-usbhdd.o \
					   archos-lcd-samsung-wvga48.o \
					   archos-lcd-tpo-wvga48.o \
					   archos-accel.o \
					   archos-irblaster.o \
					   archos-irremote.o \
					   archos-touchscreen.o \
					   archos-accessory.o \
					   archos-tvout-venc.o \
					   archos-tvout-hdmi.o \
					   archos-tvout-extdac.o \
					   archos-nl5350.o \
					   archos-atmega.o \
					   archos-flash.o \
					   archos-dieID.o

obj-$(CONFIG_MACH_ARCHOS_CORE_DUMP)	+= core_dump.o
obj-$(CONFIG_ARCHOS_FIXUP)		+= archos-fixup.o

# TUSB 6010 chips
obj-$(CONFIG_MACH_OMAP2_TUSB6010)	+= usb-tusb6010.o

