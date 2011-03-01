/*
 * wm8951.h  --  WM8951 Soc Audio driver
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@openedhand.com>
 *
 * Based on wm8753.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _WM8951_H
#define _WM8951_H

/* WM8951 register space */

#define WM8951_LINVOL   0x00
#define WM8951_RINVOL   0x01
#define WM8951_APANA    0x04
#define WM8951_APDIGI   0x05
#define WM8951_PWR      0x06
#define WM8951_IFACE    0x07
#define WM8951_SRATE    0x08
#define WM8951_ACTIVE   0x09
#define WM8951_RESET	0x0f

#define WM8951_CACHEREGNUM 	10

#define WM8951_SYSCLK	0
#define WM8951_DAI		0

struct wm8951_setup_data {
	unsigned short i2c_address;
};

extern struct snd_soc_dai wm8951_dai;
extern struct snd_soc_codec_device soc_codec_dev_wm8951;

#endif
