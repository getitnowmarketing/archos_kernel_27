cx18-objs    := cx18-driver.o cx18-cards.o cx18-i2c.o cx18-firmware.o cx18-gpio.o \
	cx18-queue.o cx18-streams.o cx18-fileops.o cx18-ioctl.o cx18-controls.o \
	cx18-mailbox.o cx18-vbi.o cx18-audio.o cx18-video.o cx18-irq.o \
	cx18-av-core.o cx18-av-audio.o cx18-av-firmware.o cx18-av-vbi.o cx18-scb.o \
	cx18-dvb.o

obj-$(CONFIG_VIDEO_CX18) += cx18.o

EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core
EXTRA_CFLAGS += -Idrivers/media/dvb/frontends
EXTRA_CFLAGS += -Idrivers/media/common/tuners
