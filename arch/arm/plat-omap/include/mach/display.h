/*
 * linux/include/asm-arm/arch-omap/display.h
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __ASM_ARCH_OMAP_DISPLAY_H
#define __ASM_ARCH_OMAP_DISPLAY_H

#include <asm/atomic.h>


#define DISPC_IRQ_FRAMEDONE		(1 << 0)
#define DISPC_IRQ_VSYNC			(1 << 1)
#define DISPC_IRQ_EVSYNC_EVEN		(1 << 2)
#define DISPC_IRQ_EVSYNC_ODD		(1 << 3)
#define DISPC_IRQ_ACBIAS_COUNT_STAT	(1 << 4)
#define DISPC_IRQ_PROG_LINE_NUM		(1 << 5)
#define DISPC_IRQ_GFX_FIFO_UNDERFLOW	(1 << 6)
#define DISPC_IRQ_GFX_END_WIN		(1 << 7)
#define DISPC_IRQ_PAL_GAMMA_MASK	(1 << 8)
#define DISPC_IRQ_OCP_ERR		(1 << 9)
#define DISPC_IRQ_VID1_FIFO_UNDERFLOW	(1 << 10)
#define DISPC_IRQ_VID1_END_WIN		(1 << 11)
#define DISPC_IRQ_VID2_FIFO_UNDERFLOW	(1 << 12)
#define DISPC_IRQ_VID2_END_WIN		(1 << 13)
#define DISPC_IRQ_SYNC_LOST		(1 << 14)
#define DISPC_IRQ_SYNC_LOST_DIGIT	(1 << 15)
#define DISPC_IRQ_WAKEUP		(1 << 16)

enum omap_display_type {
	OMAP_DISPLAY_TYPE_NONE		= 0,
	OMAP_DISPLAY_TYPE_DPI		= 1 << 0,
	OMAP_DISPLAY_TYPE_DBI		= 1 << 1,
	OMAP_DISPLAY_TYPE_SDI		= 1 << 2,
	OMAP_DISPLAY_TYPE_DSI		= 1 << 3,
	OMAP_DISPLAY_TYPE_VENC		= 1 << 4,
};

enum omap_plane {
	OMAP_DSS_GFX	= 0,
	OMAP_DSS_VIDEO1	= 1,
	OMAP_DSS_VIDEO2	= 2
};

enum omap_channel {
	OMAP_DSS_CHANNEL_LCD	= 0,
	OMAP_DSS_CHANNEL_DIGIT	= 1,
};

enum omap_color_mode {
	OMAP_DSS_COLOR_CLUT1	= 1 << 0,  /* BITMAP 1 */
	OMAP_DSS_COLOR_CLUT2	= 1 << 1,  /* BITMAP 2 */
	OMAP_DSS_COLOR_CLUT4	= 1 << 2,  /* BITMAP 4 */
	OMAP_DSS_COLOR_CLUT8	= 1 << 3,  /* BITMAP 8 */
	OMAP_DSS_COLOR_RGB12U	= 1 << 4,  /* RGB12, 16-bit container */
	OMAP_DSS_COLOR_ARGB16	= 1 << 5,  /* ARGB16 */
	OMAP_DSS_COLOR_RGB16	= 1 << 6,  /* RGB16 */
	OMAP_DSS_COLOR_RGB24U	= 1 << 7,  /* RGB24, 32-bit container */
	OMAP_DSS_COLOR_RGB24P	= 1 << 8,  /* RGB24, 24-bit container */
	OMAP_DSS_COLOR_YUV2	= 1 << 9,  /* YUV2 4:2:2 co-sited */
	OMAP_DSS_COLOR_UYVY	= 1 << 10, /* UYVY 4:2:2 co-sited */
	OMAP_DSS_COLOR_ARGB32	= 1 << 11, /* ARGB32 */
	OMAP_DSS_COLOR_RGBA32	= 1 << 12, /* RGBA32 */
	OMAP_DSS_COLOR_RGBX32	= 1 << 13, /* RGBx32 */

	OMAP_DSS_COLOR_GFX_OMAP3 =
		OMAP_DSS_COLOR_CLUT1 | OMAP_DSS_COLOR_CLUT2 |
		OMAP_DSS_COLOR_CLUT4 | OMAP_DSS_COLOR_CLUT8 |
		OMAP_DSS_COLOR_RGB12U | OMAP_DSS_COLOR_ARGB16 |
		OMAP_DSS_COLOR_RGB16 | OMAP_DSS_COLOR_RGB24U |
		OMAP_DSS_COLOR_RGB24P | OMAP_DSS_COLOR_ARGB32 |
		OMAP_DSS_COLOR_RGBA32 | OMAP_DSS_COLOR_RGBX32,

	OMAP_DSS_COLOR_VID_OMAP3 =
		OMAP_DSS_COLOR_RGB12U | OMAP_DSS_COLOR_ARGB16 |
		OMAP_DSS_COLOR_RGB16 | OMAP_DSS_COLOR_RGB24U |
		OMAP_DSS_COLOR_RGB24P | OMAP_DSS_COLOR_ARGB32 |
		OMAP_DSS_COLOR_RGBA32 | OMAP_DSS_COLOR_RGBX32 |
		OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY,
};

enum omap_lcd_display_type {
	OMAP_DSS_LCD_DISPLAY_STN,
	OMAP_DSS_LCD_DISPLAY_TFT,
};

enum omap_dss_load_mode {
	OMAP_DSS_LOAD_CLUT_AND_FRAME	= 0,
	OMAP_DSS_LOAD_CLUT_ONLY		= 1,
	OMAP_DSS_LOAD_FRAME_ONLY	= 2,
	OMAP_DSS_LOAD_CLUT_ONCE_FRAME	= 3,
};

enum omap_dss_trans_key_type {
	OMAP_DSS_COLOR_KEY_GFX_DST = 0,
	OMAP_DSS_COLOR_KEY_VID_SRC = 1,
};

enum omap_rfbi_te_mode {
	OMAP_DSS_RFBI_TE_MODE_1 = 1,
	OMAP_DSS_RFBI_TE_MODE_2 = 2,
};

enum omap_panel_config {
	OMAP_DSS_LCD_IVS		= 1<<0,
	OMAP_DSS_LCD_IHS		= 1<<1,
	OMAP_DSS_LCD_IPC		= 1<<2,
	OMAP_DSS_LCD_IEO		= 1<<3,
	OMAP_DSS_LCD_RF			= 1<<4,
	OMAP_DSS_LCD_ONOFF		= 1<<5,

	OMAP_DSS_LCD_TFT		= 1<<20,
};

enum omap_dss_venc_type {
	OMAP_DSS_VENC_TYPE_COMPOSITE,
	OMAP_DSS_VENC_TYPE_SVIDEO,
};

enum omap_dss_venc_norm {
	OMAP_DSS_VENC_NORM_PAL,
	OMAP_DSS_VENC_NORM_NTSC,
};

struct omap_display;
struct omap_panel;
struct omap_ctrl;

/* RFBI */

struct rfbi_timings {
	int cs_on_time;
	int cs_off_time;
	int we_on_time;
	int we_off_time;
	int re_on_time;
	int re_off_time;
	int we_cycle_time;
	int re_cycle_time;
	int cs_pulse_width;
	int access_time;

	int clk_div;

	u32 tim[5];             /* set by rfbi_convert_timings() */

	int converted;
};

void omap_rfbi_write_command(const void *buf, u32 len);
void omap_rfbi_read_data(void *buf, u32 len);
void omap_rfbi_write_data(const void *buf, u32 len);
void omap_rfbi_write_pixels(const void *buf, int scr_width, int x, int y,
			    int w, int h);
int omap_rfbi_enable_te(int enable, unsigned line);
int omap_rfbi_setup_te(enum omap_rfbi_te_mode mode,
			     unsigned hs_pulse_time, unsigned vs_pulse_time,
			     int hs_pol_inv, int vs_pol_inv, int extif_div);

/* DSI */
int dsi_vc_dcs_write(int channel, u8 *data, int len);
int dsi_vc_dcs_write_nosync(int channel, u8 *data, int len);
int dsi_vc_dcs_read(int channel, u8 dcs_cmd, u8 *buf, int buflen);
int dsi_vc_set_max_rx_packet_size(int channel, u16 len);
int dsi_vc_send_null(int channel);

/* Board specific data */
struct omap_display_data {
	enum omap_display_type type;

	union {
		struct {
			int data_lines;
		} dpi;

		struct {
			int channel;
			int data_lines;
		} rfbi;

		struct {
			int datapairs;
		} sdi;

		struct {
			int clk_lane;
			int clk_pol;
			int data1_lane;
			int data1_pol;
			int data2_lane;
			int data2_pol;
			unsigned long ddr_clk_hz;
		} dsi;

		struct {
			enum omap_dss_venc_type type;
			enum omap_dss_venc_norm norm;
		} venc;
	} u;

	int panel_reset_gpio;
	int ctrl_reset_gpio;

	const char *name;		/* for debug */
	const char *ctrl_name;
	const char *panel_name;

	void *priv;

	/* platform specific enable/disable */
	int (*panel_enable)(struct omap_display *display);
	void (*panel_disable)(struct omap_display *display);
	int (*ctrl_enable)(struct omap_display *display);
	void (*ctrl_disable)(struct omap_display *display);
	int (*set_backlight)(struct omap_display *display,
			int level);
	int (*set_vcom)(struct omap_display *display,
			int level);

};

struct device;

/* Board specific data */
struct omap_dss_platform_data {
	unsigned (*get_last_off_on_transaction_id)(struct device *dev);
	int num_displays;
	struct omap_display_data *displays[];
};

struct omap_ctrl {
	struct module *owner;

	const char *name;

	int (*init)(struct omap_display *display);
	void (*cleanup)(struct omap_display *display);
	int (*enable)(struct omap_display *display);
	void (*disable)(struct omap_display *display);
	int (*suspend)(struct omap_display *display);
	int (*resume)(struct omap_display *display);
	void (*setup_update)(struct omap_display *display,
			     int x, int y, int w, int h);

	int (*enable_te)(struct omap_display *display, int enable);

	int (*get_rotate)(struct omap_display *display);
	int (*set_rotate)(struct omap_display *display, int rotate);

	int (*get_mirror)(struct omap_display *display);
	int (*set_mirror)(struct omap_display *display, int enable);

	int (*run_test)(struct omap_display *display, int test);

	int pixel_size;

	struct rfbi_timings timings;

	void *priv;
};

struct omap_video_timings {
	/* Unit: pixels */
	u16 x_res;
	/* Unit: pixels */
	u16 y_res;
	/* Unit: KHz */
	u32 pixel_clock;
	/* Unit: pixel clocks */
	u16 hsw;	/* Horizontal synchronization pulse width */
	/* Unit: pixel clocks */
	u16 hfp;	/* Horizontal front porch */
	/* Unit: pixel clocks */
	u16 hbp;	/* Horizontal back porch */
	/* Unit: line clocks */
	u16 vsw;	/* Vertical synchronization pulse width */
	/* Unit: line clocks */
	u16 vfp;	/* Vertical front porch */
	/* Unit: line clocks */
	u16 vbp;	/* Vertical back porch */
	
	unsigned interlaced:1; /* this is an interlaced timing */
};

struct omap_color_key {
	enum omap_dss_trans_key_type type;
	u32 color;
	u32 enable;
};

struct omap_panel {
	struct module *owner;

	const char *name;

	int (*init)(struct omap_display *display);
	void (*cleanup)(struct omap_display *display);
	int (*remove)(struct omap_display *display);
	int (*enable)(struct omap_display *display);
	void (*disable)(struct omap_display *display);
	int (*suspend)(struct omap_display *display);
	int (*resume)(struct omap_display *display);
	int (*run_test)(struct omap_display *display, int test);

	struct omap_video_timings timings;

	int acbi;	/* ac-bias pin transitions per interrupt */
	/* Unit: line clocks */
	int acb;	/* ac-bias pin frequency */

	enum omap_panel_config config;

	int bpp;

	void *priv;
};

/* XXX perhaps this should be removed */
enum omap_dss_overlay_managers {
	OMAP_DSS_OVL_MGR_LCD,
	OMAP_DSS_OVL_MGR_TV,
};

struct omap_overlay_manager;

struct omap_overlay_info {
	int enabled;
	u32 paddr;
	void *vaddr;
	int tv_field1_offset;
	int screen_width;
	int pos_x;
	int pos_y;
	int width;
	int height;
	int out_width;	/* if 0, out_width == width */
	int out_height;	/* if 0, out_height == height */
	int rotation;
	int mirror;
	enum omap_color_mode color_mode;
	int global_alpha;
};

enum omap_overlay_caps {
	OMAP_DSS_OVL_CAP_SCALE = 1 << 0,
};

struct omap_overlay {

	const char *name;
	int id;
	struct omap_overlay_manager *manager;
	enum omap_color_mode supported_modes;
	struct omap_overlay_info info;
	enum omap_overlay_caps caps;

	int (*set_manager)(struct omap_overlay *ovl,
		struct omap_overlay_manager *mgr);
	int (*unset_manager)(struct omap_overlay *ovl);

	int (*setup_input)(struct omap_overlay *ovl,
			u32 paddr, void *vaddr,
			int tv_field1_offset,
			int screen_width,
			int width, int height,
			enum omap_color_mode color_mode, int rotation,
			int mirror, int global_alpha);
	int (*setup_output)(struct omap_overlay *ovl,
			int pos_x, int pos_y,
			int out_width, int out_height);
	int (*enable)(struct omap_overlay *ovl, int enable);
};

enum omap_overlay_manager_caps {
	OMAP_DSS_OVL_MGR_CAP_DISPC = 1 << 0,
};

struct omap_overlay_manager_info {
	u32 default_color;

	enum omap_dss_trans_key_type trans_key_type;
	u32 trans_key;
	int trans_enabled;

	int alpha_enabled;
};

struct omap_overlay_manager {

	const char *name;
	int id;
	enum omap_overlay_manager_caps caps;
	struct omap_display *display;
	int num_overlays;
	struct omap_overlay *overlays;
	enum omap_display_type supported_displays;
	struct omap_overlay_manager_info info;

	/* if true, info has been changed, but not applied() yet */
	int info_dirty;

	int (*set_display)(struct omap_overlay_manager *mgr,
		struct omap_display *display);
	int (*unset_display)(struct omap_overlay_manager *mgr);

	int (*apply)(struct omap_overlay_manager *mgr);

	int (*set_manager_info)(struct omap_overlay_manager *mgr,
			struct omap_overlay_manager_info *info);
	void (*get_manager_info)(struct omap_overlay_manager *mgr,
			struct omap_overlay_manager_info *info);

};

enum omap_display_caps {
	OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE = 1 << 0,
};

enum omap_dss_update_mode {
	OMAP_DSS_UPDATE_DISABLED = 0,
	OMAP_DSS_UPDATE_AUTO,
	OMAP_DSS_UPDATE_MANUAL,
};

enum omap_dss_display_state {
	OMAP_DSS_DISPLAY_DISABLED = 0,
	OMAP_DSS_DISPLAY_ACTIVE,
	OMAP_DSS_DISPLAY_SUSPENDED,
};

struct omap_display {
	struct device *dev;

	/*atomic_t ref_count;*/
	int ref_count;

	struct completion frame_done;

	enum omap_display_type type;
	const char *name;

	enum omap_display_caps caps;

	struct omap_overlay_manager *manager;

	enum omap_dss_display_state state;

	struct omap_display_data hw_config;	/* board specific data */
	struct omap_ctrl *ctrl;			/* static common data */
	struct omap_panel *panel;		/* static common data */

	int (*enable)(struct omap_display *display);
	void (*disable)(struct omap_display *display);

	int (*suspend)(struct omap_display *display);
	int (*resume)(struct omap_display *display);

	void (*get_resolution)(struct omap_display *display,
			int *xres, int *yres);

	int (*check_timings)(struct omap_display *display,
			struct omap_video_timings *timings);
	void (*set_bg_color)(struct omap_display *display,
			unsigned int color);
	int (*get_bg_color)(struct omap_display *display);
	void (*set_timings)(struct omap_display *display,
			struct omap_video_timings *timings);
	void (*get_timings)(struct omap_display *display,
			struct omap_video_timings *timings);
	void (*set_color_keying)(struct omap_display *display,
			struct omap_color_key *key);
	void (*enable_alpha_blending)(struct omap_display *display,
			unsigned int enable);
	int (*get_alpha_blending)(struct omap_display *display);
	void (*get_color_keying)(struct omap_display *display,
			struct omap_color_key *key);
	int (*update)(struct omap_display *display,
			       int x, int y, int w, int h);
	int (*sync)(struct omap_display *display);

	int (*set_update_mode)(struct omap_display *display,
			enum omap_dss_update_mode);
	enum omap_dss_update_mode (*get_update_mode)
		(struct omap_display *display);

	int (*enable_te)(struct omap_display *display, int enable);
	int (*get_te)(struct omap_display *display);

	int (*get_rotate)(struct omap_display *display);
	int (*set_rotate)(struct omap_display *display, int rotate);

	int (*get_mirror)(struct omap_display *display);
	int (*set_mirror)(struct omap_display *display, int enable);
	
	enum omap_dss_venc_norm (*get_tv_norm)(struct omap_display *display);
	int (*set_tv_norm)(struct omap_display *display, enum omap_dss_venc_norm norm);

	enum omap_dss_venc_type (*get_tv_type)(struct omap_display *display);
	int (*set_tv_type)(struct omap_display *display, enum omap_dss_venc_type type);

	int (*run_test)(struct omap_display *display, int test);
};

int omap_dss_get_num_displays(void);
struct omap_display *omap_dss_get_display(int no);
void omap_dss_put_display(struct omap_display *display);

void omap_dss_register_ctrl(struct omap_ctrl *ctrl);
void omap_dss_unregister_ctrl(struct omap_ctrl *ctrl);

void omap_dss_register_panel(struct omap_panel *panel);
void omap_dss_unregister_panel(struct omap_panel *panel);

int omap_dss_get_num_overlay_managers(void);
struct omap_overlay_manager *omap_dss_get_overlay_manager(int num);

int omap_dss_get_num_overlays(void);
struct omap_overlay *omap_dss_get_overlay(int num);

void omap_dispc_set_linenbr_irq(int linenum);
void omap_dispc_set_plane_ba0(enum omap_plane plane, u32 paddr);
void omap_dispc_set_plane_ba1(enum omap_plane plane, u32 paddr);
void omap_dispc_set_lcd_timings(int hsw, int hfp, int hbp,
				   int vsw, int vfp, int vbp);
void omap_dispc_go(enum omap_channel channel);

typedef void (*omap_dispc_isr_t) (void *arg, u32 mask);
void *omap_dispc_register_isr(omap_dispc_isr_t isr, void *arg, u32 mask);
int omap_dispc_unregister_isr(void *handle);


typedef void (*omap2_disp_isr_t)(void* arg, struct pt_regs *regs);
int omap2_disp_register_isr(omap2_disp_isr_t isr, void *arg, unsigned int mask);
int omap2_disp_unregister_isr(omap2_disp_isr_t isr);
void omap2_disp_get_dss(void);
void omap2_disp_put_dss(void);
#endif
