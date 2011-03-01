#ifndef _DISPC_H
#define _DISPC_H

#include <linux/interrupt.h>

#define DISPC_PLANE_GFX			0
#define DISPC_PLANE_VID1		1
#define DISPC_PLANE_VID2		2

#define DISPC_RGB_1_BPP			0x00
#define DISPC_RGB_2_BPP			0x01
#define DISPC_RGB_4_BPP			0x02
#define DISPC_RGB_8_BPP			0x03
#define DISPC_RGB_12_BPP		0x04
#define DISPC_RGB_16_BPP		0x06
#define DISPC_RGB_24_BPP_UNPACK_32	0x08
#define DISPC_RGB_24_BPP		0x09
#define DISPC_YUV2_422			0x0a
#define DISPC_UYVY_422			0x0b
#define DISPC_ARGB_32_BPP		0x0c
#define DISPC_RGBA_32_BPP		0x0d
#define DISPC_RGBx_32_BPP		0x0e

#define DISPC_BURST_4x32		0
#define DISPC_BURST_8x32		1
#define DISPC_BURST_16x32		2

#define DISPC_LOAD_CLUT_AND_FRAME	0x00
#define DISPC_LOAD_CLUT_ONLY		0x01
#define DISPC_LOAD_FRAME_ONLY		0x02
#define DISPC_LOAD_CLUT_ONCE_FRAME	0x03

#define DISPC_TFT_DATA_LINES_12		0
#define DISPC_TFT_DATA_LINES_16		1
#define DISPC_TFT_DATA_LINES_18		2
#define DISPC_TFT_DATA_LINES_24		3

extern void omap_dispc_set_lcd_size(int width, int height);

extern void omap_dispc_enable_lcd_out(int enable);
extern void omap_dispc_enable_digit_out(int enable);

extern int omap_dispc_request_irq(unsigned long irq_mask,
                                  void (*callback)(void *data), void *data);
extern void omap_dispc_free_irq(unsigned long irq_mask,
                                void (*callback)(void *data), void *data);

extern const struct lcd_ctrl omap2_int_ctrl;

/* from dsipll.c */
extern int omap_disp_start_dsi_pll(u32 clkrate);
extern int omap_disp_stop_dsi_pll(void);

/* from venc.c */
enum tv_standards { TVSTD_PAL_BDGHI=0, TVSTD_PAL_N, TVSTD_PAL_NC, 
	TVSTD_PAL_M, TVSTD_PAL_60, TVSTD_NTSC_M, TVSTD_NTSC_443, TVSTD_NTSC_J
};

#endif
