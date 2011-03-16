/*
 * linux/sound/arm/omap/omap-alsa-dma.h
 *
 * Common audio DMA handling for the OMAP processors
 *
 * Copyright (C) 2006 Mika Laitio <lamikr@cc.jyu.fi>
 *
 * Copyright (C) 2005 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
 *
 * Copyright (C) 2000, 2001 Nicolas Pitre <nico@cam.org>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 *
 * 2004/08/12  Nishanth Menon - Modified to integrate Audio requirements on
 * 				1610, 1710 platforms
 *
 * 2005/07/25  INdT Kernel Team - Renamed to omap-alsa-dma.h. Ported to Alsa.
 */

#ifndef __OMAP_AUDIO_ALSA_DMA_H
#define __OMAP_AUDIO_ALSA_DMA_H

#include <mach/omap-alsa.h>

/* Global data structures */

typedef void (*dma_callback_t) (int lch, u16 ch_status, void *data);

/* arch specific functions */

void omap_clear_alsa_sound_dma(struct audio_stream *s);

int omap_request_alsa_sound_dma(int device_id, const char *device_name,
				void *data, int **channels);
int omap_free_alsa_sound_dma(void *data, int **channels);

int omap_start_alsa_sound_dma(struct audio_stream *s, dma_addr_t dma_ptr,
				u_int dma_size);

int omap_stop_alsa_sound_dma(struct audio_stream *s);

#ifdef CONFIG_SND_OMAP3_TWL4030
int omap_transfer_posn_alsa_sound_dma(struct audio_stream *s);
int omap_init_alsa_sound_dma(int mode);
int twl4030_conf_data_interface(void);
#else
static inline int omap_transfer_posn_alsa_sound_dma(struct audio_stream *s)
						{ return 0; }
static inline int omap_init_alsa_sound_dma(int mode)
						{ return 0; }
static inline int twl4030_conf_data_interface(void)
						{ return 0; }
#endif

#endif
