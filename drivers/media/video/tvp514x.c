/*
 * drivers/media/video/tvp514x.c
 *
 * TI TVP5146/47 decoder driver
 *
 * Copyright (C) 2008 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * Contributors:
 *     Sivaraj R <sivaraj@ti.com>
 *     Brijesh R Jadav <brijesh.j@ti.com>
 *     Hardik Shah <hardik.shah@ti.com>
 *     Manjunath Hadli <mrh@ti.com>
 *     Karicheri Muralidharan <m-karicheri2@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <media/v4l2-int-device.h>
#include <media/tvp514x.h>

#include "tvp514x_regs.h"

#define MODULE_NAME	TVP514X_MODULE_NAME

/* Private macros for TVP */
#define I2C_RETRY_COUNT                 (5)
#define LOCK_RETRY_COUNT                (5)
#define LOCK_RETRY_DELAY                (200)

/* FIXME: these need to be propagated to the application,
 * make a nice header file for them */
#define V4L2_CID_RED_GAIN		(V4L2_CID_PRIVATE_BASE+0)
#define V4L2_CID_BLUE_GAIN		(V4L2_CID_PRIVATE_BASE+1)
#define V4L2_CID_GREEN_GAIN		(V4L2_CID_PRIVATE_BASE+2)
#define V4L2_CID_PB_SATURATION		(V4L2_CID_PRIVATE_BASE+3)
#define V4L2_CID_PR_SATURATION		(V4L2_CID_PRIVATE_BASE+4)

/* Define this if you want to allow an input to be set only if the chip has locked a signal on it. */
//#define LOCKED_INPUT_NEEDED

/* Debug functions */
static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

#define dump_reg(client, reg, val)				\
	do {							\
		val = tvp514x_read_reg(client, reg);		\
		v4l_info(client, "Reg(0x%.2X): 0x%.2X\n", reg, val); \
	} while (0)

/**
 * struct tvp514x_decoded - TVP5146/47 decoder object
 * @v4l2_int_device: Slave handle
 * @pdata: Board specific
 * @client: I2C client data
 * @id: Entry from I2C table
 * @ver: Chip version
 * @state: TVP5146/47 decoder state - detected or not-detected
 * @pix: Current pixel format
 * @num_fmts: Number of formats
 * @fmt_list: Format list
 * @current_std: Current standard
 * @num_stds: Number of standards
 * @std_list: Standards list
 * @num_ctrls: Number of controls
 * @ctrl_list: Control list
 * @route: input and output routing at chip level
 */
struct tvp514x_decoder {
	struct v4l2_int_device *v4l2_int_device;
	const struct tvp514x_platform_data *pdata;
	struct i2c_client *client;

	struct i2c_device_id *id;

	int ver;
	enum tvp514x_state state;

	struct v4l2_pix_format pix;
	int num_fmts;
	const struct v4l2_fmtdesc *fmt_list;

	enum tvp514x_std current_std;
	int num_stds;
	struct tvp514x_std_info *std_list;

	int num_ctrls;
	const struct tvp514x_ctrl_info *ctrl_list;

	struct v4l2_routing route;
};

/* TVP514x default register values */
static struct tvp514x_reg tvp514x_reg_list[] = {
	{TOK_WRITE, REG_INPUT_SEL, 0x0C},	/* Composite selected */
	{TOK_WRITE, REG_AFE_GAIN_CTRL, 0x0F},
	{TOK_WRITE, REG_VIDEO_STD, 0x00},	/* Auto mode */
	{TOK_WRITE, REG_OPERATION_MODE, 0x00},	/* Enable */
	{TOK_WRITE, REG_AUTOSWITCH_MASK, 0x3F}, /* Every std possible */
	{TOK_WRITE, REG_COLOR_KILLER, 0x00},
	{TOK_WRITE, REG_LUMA_CONTROL1, 0x00},
	{TOK_WRITE, REG_LUMA_CONTROL2, 0x0C},
	{TOK_WRITE, REG_LUMA_CONTROL3, 0x03},
	{TOK_WRITE, REG_BRIGHTNESS, 0x80},
	{TOK_WRITE, REG_CONTRAST, 0x80},
	{TOK_WRITE, REG_SATURATION, 0x80},
	{TOK_WRITE, REG_HUE, 0x00},
	{TOK_WRITE, REG_CHROMA_CONTROL1, 0x00},
	{TOK_WRITE, REG_CHROMA_CONTROL2, 0x02},
	{TOK_SKIP, 0x0F, 0x00},	/* Reserved */
	{TOK_WRITE, REG_COMP_PR_SATURATION, 0x80},
	{TOK_WRITE, REG_COMP_Y_CONTRAST, 0x80},
	{TOK_WRITE, REG_COMP_PB_SATURATION, 0x80},
	{TOK_SKIP, 0x13, 0x00},	/* Reserved */
	{TOK_WRITE, REG_COMP_Y_BRIGHTNESS, 0x80},
	{TOK_SKIP, 0x15, 0x00},	/* Reserved */
	{TOK_SKIP, REG_AVID_START_PIXEL_LSB, 0x55},	/* NTSC timing */
	{TOK_SKIP, REG_AVID_START_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_AVID_STOP_PIXEL_LSB, 0x25},
	{TOK_SKIP, REG_AVID_STOP_PIXEL_MSB, 0x03},
	{TOK_SKIP, REG_HSYNC_START_PIXEL_LSB, 0x00},	/* NTSC timing */
	{TOK_SKIP, REG_HSYNC_START_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_HSYNC_STOP_PIXEL_LSB, 0x40},
	{TOK_SKIP, REG_HSYNC_STOP_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_VSYNC_START_LINE_LSB, 0x04},	/* NTSC timing */
	{TOK_SKIP, REG_VSYNC_START_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VSYNC_STOP_LINE_LSB, 0x07},
	{TOK_SKIP, REG_VSYNC_STOP_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VBLK_START_LINE_LSB, 0x01},	/* NTSC timing */
	{TOK_SKIP, REG_VBLK_START_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VBLK_STOP_LINE_LSB, 0x15},
	{TOK_SKIP, REG_VBLK_STOP_LINE_MSB, 0x00},
#if 0
	{TOK_SKIP, REG_AVID_START_PIXEL_LSB, 0x5F},	/* PAL timing */
	{TOK_SKIP, REG_AVID_START_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_AVID_STOP_PIXEL_LSB, 0x2F},
	{TOK_SKIP, REG_AVID_STOP_PIXEL_MSB, 0x03},
	{TOK_SKIP, REG_HSYNC_START_PIXEL_LSB, 0x07},	/* PAL timing */
	{TOK_SKIP, REG_HSYNC_START_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_HSYNC_STOP_PIXEL_LSB, 0x47},
	{TOK_SKIP, REG_HSYNC_STOP_PIXEL_MSB, 0x00},
	{TOK_SKIP, REG_VSYNC_START_LINE_LSB, 0x01},	/* PAL timing */
	{TOK_SKIP, REG_VSYNC_START_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VSYNC_STOP_LINE_LSB, 0x04},
	{TOK_SKIP, REG_VSYNC_STOP_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VBLK_START_LINE_LSB, 0x6F},	/* PAL timing */
	{TOK_SKIP, REG_VBLK_START_LINE_MSB, 0x00},
	{TOK_SKIP, REG_VBLK_STOP_LINE_LSB, 0x18},
	{TOK_SKIP, REG_VBLK_STOP_LINE_MSB, 0x00},
#endif
	{TOK_SKIP, 0x26, 0x00},	/* Reserved */
	{TOK_SKIP, 0x27, 0x00},	/* Reserved */
	{TOK_SKIP, REG_FAST_SWTICH_CONTROL, 0xCC},
	{TOK_SKIP, 0x29, 0x00},	/* Reserved */
	{TOK_SKIP, REG_FAST_SWTICH_SCART_DELAY, 0x00},
	{TOK_SKIP, 0x2B, 0x00},	/* Reserved */
	{TOK_SKIP, REG_SCART_DELAY, 0x00},
	{TOK_SKIP, REG_CTI_DELAY, 0x00},
	{TOK_SKIP, REG_CTI_CONTROL, 0x00},
	{TOK_SKIP, 0x2F, 0x00},	/* Reserved */
	{TOK_SKIP, 0x30, 0x00},	/* Reserved */
	{TOK_SKIP, 0x31, 0x00},	/* Reserved */
	{TOK_WRITE, REG_SYNC_CONTROL, 0x00},	/* HS, VS active high */
	{TOK_WRITE, REG_OUTPUT_FORMATTER1, 0x40},	/* 10-bit BT.656 */
	{TOK_WRITE, REG_OUTPUT_FORMATTER2, 0x11},	/* Enable clk & data */
	{TOK_WRITE, REG_OUTPUT_FORMATTER3, 0xFF},	/* Enable AVID & FLD */
	{TOK_WRITE, REG_OUTPUT_FORMATTER4, 0xFF},	/* Enable VS & HS */
	{TOK_WRITE, REG_OUTPUT_FORMATTER5, 0xFF},
	{TOK_WRITE, REG_OUTPUT_FORMATTER6, 0xFF},
	{TOK_WRITE, REG_CLEAR_LOST_LOCK, 0x01},	/* Clear status */
	{TOK_SKIP, 0x3A, 0x00},	/* Read only */
	{TOK_SKIP, 0x3B, 0x00},	/* Read only */
	{TOK_SKIP, 0x3C, 0x00},	/* Read only */
	{TOK_SKIP, 0x3D, 0x00},	/* Read only */
	{TOK_SKIP, 0x3E, 0x00},	/* Reserved */
	{TOK_SKIP, 0x3F, 0x00},	/* Read only */
	{TOK_SKIP, 0x40, 0x00},	/* Read only */
	{TOK_SKIP, 0x41, 0x00},	/* Read only */
	{TOK_SKIP, 0x42, 0x00},	/* Read only */
	{TOK_SKIP, 0x43, 0x00},	/* Read only */
	{TOK_SKIP, 0x44, 0x00},	/* Reserved */
	{TOK_SKIP, 0x45, 0x00},	/* Reserved */
	{TOK_SKIP, REG_AFE_COARSE_GAIN_CH1, 0x20},
	{TOK_SKIP, REG_AFE_COARSE_GAIN_CH2, 0x20},
	{TOK_SKIP, REG_AFE_COARSE_GAIN_CH3, 0x20},
	{TOK_SKIP, REG_AFE_COARSE_GAIN_CH4, 0x20},
	{TOK_WRITE, REG_AFE_FINE_GAIN_PB_B_LSB, 0x00},
	{TOK_WRITE, REG_AFE_FINE_GAIN_PB_B_MSB, 0x04},
	{TOK_WRITE, REG_AFE_FINE_GAIN_Y_G_CHROMA_LSB, 0x00},
	{TOK_WRITE, REG_AFE_FINE_GAIN_Y_G_CHROMA_MSB, 0x04},
	{TOK_WRITE, REG_AFE_FINE_GAIN_PR_R_LSB, 0x00},
	{TOK_WRITE, REG_AFE_FINE_GAIN_PR_R_MSB, 0x04},
	{TOK_TERM, 0, 0},
};

/* List of image formats supported by TVP5146/47 decoder
 * Currently we are using 8 bit mode only, but can be
 * extended to 10/20 bit mode.
 */
static const struct v4l2_fmtdesc tvp514x_fmt_list[] = {
	{
	 .index = 0,
	 .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	 .flags = 0,
	 .description = "8-bit UYVY 4:2:2 Format",
	 .pixelformat = V4L2_PIX_FMT_UYVY,
	},
};

#define TVP514X_NUM_FORMATS		ARRAY_SIZE(tvp514x_fmt_list)

/*
 * Supported standards -
 *
 * Currently supports two standards only, need to add support for rest of the
 * modes, like SECAM, etc...
 */
static struct tvp514x_std_info tvp514x_std_list[] = {
	/* Standard: STD_NTSC_MJ */
	[STD_NTSC_MJ] = {
	 .width = NTSC_NUM_ACTIVE_PIXELS,
	 .height = NTSC_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_NTSC_MJ_BIT,
	 .standard = {
		      .index = 0,
		      .id = V4L2_STD_NTSC_M,
		      .name = "NTSC",
		      .frameperiod = {1001, 30000},
		      .framelines = 525
		     },
	},
	/* Standard: STD_PAL_BDGHIN */
	[STD_PAL_BDGHIN] = {
	 .width = PAL_NUM_ACTIVE_PIXELS,
	 .height = PAL_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_PAL_BDGHIN_BIT,
	 .standard = {
		      .index = 1,
		      .id = V4L2_STD_PAL_B | V4L2_STD_PAL_G | V4L2_STD_PAL_H |
				V4L2_STD_PAL_I | V4L2_STD_PAL_N,
		      .name = "PAL-BDGHIN",
		      .frameperiod = {1, 25},
		      .framelines = 625
		     },
	},
	/* Standard: STD_PAL_M */
	[STD_PAL_M] = {
	 .width = PAL_NUM_ACTIVE_PIXELS,
	 .height = PAL_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_PAL_M_BIT,
	 .standard = {
		      .index = 2,
		      .id = V4L2_STD_PAL_M,
		      .name = "PAL-M",
		      .frameperiod = {1001, 30000},
		      .framelines = 525
		     },
	},
	/* Standard: STD_PAL_Nc */
	[STD_PAL_Nc] = {
	 .width = PAL_NUM_ACTIVE_PIXELS,
	 .height = PAL_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_PAL_COMBINATION_N_BIT,
	 .standard = {
		      .index = 3,
		      .id = V4L2_STD_PAL_Nc,
		      .name = "PAL-Nc",
		      .frameperiod = {1, 25},
		      .framelines = 625
		     },
	},
	/* Standard: STD_NTSC_443 */
	[STD_NTSC_443] = {
	 .width = PAL_NUM_ACTIVE_PIXELS,
	 .height = PAL_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_NTSC_4_43_BIT,
	 .standard = {
		      .index = 4,
		      .id = V4L2_STD_NTSC_443,
		      .name = "NTSC-443",
		      .frameperiod = {1001, 30000},
		      .framelines = 525
		     },
	},
	/* Standard: STD_SECAM */
	[STD_SECAM] = {
	 .width = PAL_NUM_ACTIVE_PIXELS,
	 .height = PAL_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_SECAM_BIT,
	 .standard = {
		      .index = 5,
		      .id = V4L2_STD_SECAM,
		      .name = "SECAM",
		      .frameperiod = {1, 25},
		      .framelines = 625
		     },
	},
	/* Standard: STD_PAL_60 */
	[STD_PAL_60] = {
	 .width = PAL_NUM_ACTIVE_PIXELS,
	 .height = PAL_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_PAL_60_BIT,
	 .standard = {
		      .index = 6,
		      .id = V4L2_STD_PAL_60,
		      .name = "PAL-60",
		      .frameperiod = {1001, 30000},
		      .framelines = 525
		     },
	},
	/* Standard: need to add for additional standard */
};

#define TVP514X_NUM_STANDARDS		ARRAY_SIZE(tvp514x_std_list)

/* Supported controls */
static const struct tvp514x_ctrl_info tvp514x_ctrl_list[] = {
	{
		.regs = {
			.cvbs_reg_address = REG_BRIGHTNESS,
			.component_reg_address = REG_COMP_Y_BRIGHTNESS,
		},
		.query_ctrl = {
			.id = V4L2_CID_BRIGHTNESS,
			.name = "BRIGHTNESS",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = 128
		},
	}, {
		.regs = {
			.cvbs_reg_address = REG_CONTRAST,
			.component_reg_address = REG_COMP_Y_CONTRAST,
		},
		.query_ctrl = {
			.id = V4L2_CID_CONTRAST,
			.name = "CONTRAST",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = 128
		},
	}, {
		.regs = {
			.cvbs_reg_address = REG_SATURATION,
		},
		.query_ctrl = {
			.id = V4L2_CID_SATURATION,
			.name = "SATURATION",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = 128
		},
	}, {
		.regs = {
			.cvbs_reg_address = REG_HUE,
		},
		.query_ctrl = {
			.id = V4L2_CID_HUE,
			.name = "HUE",
			.type = V4L2_CTRL_TYPE_INTEGER,
			.minimum = -127,
			.maximum = 127,
			.step = 1,
			.default_value = 0
		},
	}, {
		.regs = {
			.rgb_reg_address = REG_AFE_GAIN_CTRL,
		},
		.query_ctrl = {
			.id = V4L2_CID_AUTOGAIN,
			.name = "Automatic Gain Control",
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 1
		},
	}, {
		.regs = {
			.rgb_reg_address = REG_AFE_FINE_GAIN_Y_G_CHROMA_LSB,
			.rgb_reg_msb_address = REG_AFE_FINE_GAIN_Y_G_CHROMA_MSB,
		},
		.query_ctrl = {
			.id = V4L2_CID_GREEN_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "ggain",
			.minimum = 0,
			.maximum = 99,
			.step = 0x1,
			.default_value = 0x31,
		},
	}, {
		.regs = {
			.rgb_reg_address = REG_AFE_FINE_GAIN_PB_B_LSB,
			.rgb_reg_msb_address = REG_AFE_FINE_GAIN_PB_B_MSB,
		},
		.query_ctrl = {
			.id = V4L2_CID_BLUE_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "bgain",
			.minimum = 0,
			.maximum = 99,
			.step = 0x1,
			.default_value = 0x31,
		},
	}, {
		.regs = {
			.rgb_reg_address = REG_AFE_FINE_GAIN_PR_R_LSB,
			.rgb_reg_msb_address = REG_AFE_FINE_GAIN_PR_R_MSB,
		},
		.query_ctrl = {
			.id = V4L2_CID_RED_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "rgain",
			.minimum = 0,
			.maximum = 99,
			.step = 0x1,
			.default_value = 0x31,
		},
	}, {
		.regs = {
			.component_reg_address = REG_COMP_PB_SATURATION,
		},
		.query_ctrl = {
			.id = V4L2_CID_PB_SATURATION,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "PbSat",
			.minimum = 0,
			.maximum = 255,
			.step = 0x1,
			.default_value = 0x31,
		},
	}, {
		.regs = {
			.component_reg_address = REG_COMP_PR_SATURATION,
		},
		.query_ctrl = {
			.id = V4L2_CID_PR_SATURATION,
			.type =
			V4L2_CTRL_TYPE_INTEGER,
			.name = "PrSat",
			.minimum = 0,
			.maximum = 255,
			.step = 0x1,
			.default_value = 0x31,
		},
	}
};

#define TVP514X_NUM_CONTROLS		ARRAY_SIZE(tvp514x_ctrl_list)

/*
 * Read a value from a register in an TVP5146/47 decoder device.
 * Returns value read if successful, or non-zero (-1) otherwise.
 */
static int tvp514x_read_reg(struct i2c_client *client, u8 reg)
{
	int err;
	int retry = 0;
read_again:

	err = i2c_smbus_read_byte_data(client, reg);
	if (err == -1) {
		if (retry <= I2C_RETRY_COUNT) {
			v4l_warn(client, "Read: retry ... %d\n", retry);
			retry++;
			msleep_interruptible(10);
			goto read_again;
		}
	}

	return err;
}

/*
 * Write a value to a register in an TVP5146/47 decoder device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int tvp514x_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int err;
	int retry = 0;
write_again:

	err = i2c_smbus_write_byte_data(client, reg, val);
	if (err) {
		if (retry <= I2C_RETRY_COUNT) {
			v4l_warn(client, "Write: retry ... %d\n", retry);
			retry++;
			msleep_interruptible(10);
			goto write_again;
		}
	}

	return err;
}

static inline unsigned int tvp514x_vbus_read(struct i2c_client *c, unsigned int addr)
{
	unsigned int value;
	tvp514x_write_reg(c, REG_VBUS_ADDRESS_ACCESS1, addr & 0xff);
	tvp514x_write_reg(c, REG_VBUS_ADDRESS_ACCESS2, addr >>8 & 0xff );
	tvp514x_write_reg(c, REG_VBUS_ADDRESS_ACCESS3, addr >>16 & 0xff);
	value = tvp514x_read_reg(c, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR);
	v4l_dbg(3, debug, c, "tvp514x: read vbus reg 0x%x (%x:%x:%x)= %x\n", addr,addr & 0xff, addr >>8 & 0xff, addr >>16 & 0xff, value);
	return value;
}

static inline void tvp514x_vbus_write(struct i2c_client *c, unsigned int addr, unsigned int value)
{
	tvp514x_write_reg(c, REG_VBUS_ADDRESS_ACCESS1, addr & 0xff);
	tvp514x_write_reg(c, REG_VBUS_ADDRESS_ACCESS2, addr >>8 & 0xff );
	tvp514x_write_reg(c, REG_VBUS_ADDRESS_ACCESS3, addr >>16 & 0xff);
	tvp514x_write_reg(c, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, value);
	v4l_dbg(3, debug, c, "tvp514x: write vbus reg 0x%x (%x:%x:%x)= %x\n", addr,addr & 0xff, addr >>8 & 0xff, addr >>16 & 0xff, value);
}

/*
 * tvp514x_write_regs : Initializes a list of TVP5146/47 registers
 *		if token is TOK_TERM, then entire write operation terminates
 *		if token is TOK_DELAY, then a delay of 'val' msec is introduced
 *		if token is TOK_SKIP, then the register write is skipped
 *		if token is TOK_WRITE, then the register write is performed
 *
 * reglist - list of registers to be written
 * Returns zero if successful, or non-zero otherwise.
 */
static int tvp514x_write_regs(struct i2c_client *client,
			      const struct tvp514x_reg reglist[])
{
	int err;
	const struct tvp514x_reg *next = reglist;

	for (; next->token != TOK_TERM; next++) {
		if (next->token == TOK_DELAY) {
			msleep(next->val);
			continue;
		}

		if (next->token == TOK_SKIP)
			continue;

		err = tvp514x_write_reg(client, next->reg, (u8) next->val);
		if (err) {
			v4l_err(client, "Write failed. Err[%d]\n", err);
			return err;
		}
	}
	return 0;
}

/*
 * tvp514x_get_current_std:
 * Returns the current standard detected by TVP5146/47
 */
static enum tvp514x_std tvp514x_get_current_std(struct tvp514x_decoder
						*decoder)
{
	u8 std, std_status;

	std = tvp514x_read_reg(decoder->client, REG_VIDEO_STD);
	if ((std & VIDEO_STD_MASK) == VIDEO_STD_AUTO_SWITCH_BIT) {
		/* use the standard status register */
		std_status = tvp514x_read_reg(decoder->client,
				REG_VIDEO_STD_STATUS);
	} else
		std_status = std;	/* use the standard register itself */

	switch (std_status & VIDEO_STD_MASK) {
	case VIDEO_STD_NTSC_MJ_BIT:
		return STD_NTSC_MJ;

	case VIDEO_STD_PAL_BDGHIN_BIT:
		return STD_PAL_BDGHIN;

	case VIDEO_STD_PAL_M_BIT:
		return STD_PAL_M;

	case VIDEO_STD_PAL_COMBINATION_N_BIT:
		return STD_PAL_Nc;

	case VIDEO_STD_NTSC_4_43_BIT:
		return STD_NTSC_443;

	case VIDEO_STD_SECAM_BIT:
		return STD_SECAM;

	case VIDEO_STD_PAL_60_BIT:
		return STD_PAL_60;

	default:
		return STD_INVALID;
	}

	return STD_INVALID;
}

/*
 * TVP5146/47 register dump function
 */
static void tvp514x_reg_dump(struct tvp514x_decoder *decoder)
{
	u8 value;

	dump_reg(decoder->client, REG_INPUT_SEL, value);
	dump_reg(decoder->client, REG_AFE_GAIN_CTRL, value);
	dump_reg(decoder->client, REG_VIDEO_STD, value);
	dump_reg(decoder->client, REG_OPERATION_MODE, value);
	dump_reg(decoder->client, REG_AUTOSWITCH_MASK, value);
	dump_reg(decoder->client, REG_COLOR_KILLER, value);
	dump_reg(decoder->client, REG_LUMA_CONTROL1, value);
	dump_reg(decoder->client, REG_LUMA_CONTROL2, value);
	dump_reg(decoder->client, REG_LUMA_CONTROL3, value);
	dump_reg(decoder->client, REG_BRIGHTNESS, value);
	dump_reg(decoder->client, REG_CONTRAST, value);
	dump_reg(decoder->client, REG_SATURATION, value);
	dump_reg(decoder->client, REG_HUE, value);
	dump_reg(decoder->client, REG_CHROMA_CONTROL1, value);
	dump_reg(decoder->client, REG_CHROMA_CONTROL2, value);
	dump_reg(decoder->client, REG_COMP_PR_SATURATION, value);
	dump_reg(decoder->client, REG_COMP_Y_CONTRAST, value);
	dump_reg(decoder->client, REG_COMP_PB_SATURATION, value);
	dump_reg(decoder->client, REG_COMP_Y_BRIGHTNESS, value);
	dump_reg(decoder->client, REG_AVID_START_PIXEL_LSB, value);
	dump_reg(decoder->client, REG_AVID_START_PIXEL_MSB, value);
	dump_reg(decoder->client, REG_AVID_STOP_PIXEL_LSB, value);
	dump_reg(decoder->client, REG_AVID_STOP_PIXEL_MSB, value);
	dump_reg(decoder->client, REG_HSYNC_START_PIXEL_LSB, value);
	dump_reg(decoder->client, REG_HSYNC_START_PIXEL_MSB, value);
	dump_reg(decoder->client, REG_HSYNC_STOP_PIXEL_LSB, value);
	dump_reg(decoder->client, REG_HSYNC_STOP_PIXEL_MSB, value);
	dump_reg(decoder->client, REG_VSYNC_START_LINE_LSB, value);
	dump_reg(decoder->client, REG_VSYNC_START_LINE_MSB, value);
	dump_reg(decoder->client, REG_VSYNC_STOP_LINE_LSB, value);
	dump_reg(decoder->client, REG_VSYNC_STOP_LINE_MSB, value);
	dump_reg(decoder->client, REG_VBLK_START_LINE_LSB, value);
	dump_reg(decoder->client, REG_VBLK_START_LINE_MSB, value);
	dump_reg(decoder->client, REG_VBLK_STOP_LINE_LSB, value);
	dump_reg(decoder->client, REG_VBLK_STOP_LINE_MSB, value);
	dump_reg(decoder->client, REG_SYNC_CONTROL, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER1, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER2, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER3, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER4, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER5, value);
	dump_reg(decoder->client, REG_OUTPUT_FORMATTER6, value);
	dump_reg(decoder->client, REG_CLEAR_LOST_LOCK, value);
	dump_reg(decoder->client, 0x3A, value);
	dump_reg(decoder->client, 0x3B, value);
	dump_reg(decoder->client, 0x3C, value);
	dump_reg(decoder->client, 0x3D, value);
	dump_reg(decoder->client, 0x3E, value);
	dump_reg(decoder->client, 0x3F, value);
	dump_reg(decoder->client, 0x40, value);
	dump_reg(decoder->client, 0x41, value);
	dump_reg(decoder->client, 0x42, value);
	dump_reg(decoder->client, 0x43, value);
	dump_reg(decoder->client, 0x44, value);
	dump_reg(decoder->client, 0x45, value);
}

/*
 * Configure the TVP5146/47 with the current register settings
 * Returns zero if successful, or non-zero otherwise.
 */
static int tvp514x_configure(struct tvp514x_decoder *decoder)
{
	int err;

	/* common register initialization */
	err =
	    tvp514x_write_regs(decoder->client, tvp514x_reg_list);
	if (err)
		return err;

	if (debug)
		tvp514x_reg_dump(decoder);

	return 0;
}

/*
 * Detect if an tvp514x is present, and if so which revision.
 * A device is considered to be detected if the chip ID (LSB and MSB)
 * registers match the expected values.
 * Any value of the rom version register is accepted.
 * Returns ENODEV error number if no device is detected, or zero
 * if a device is detected.
 */
static int tvp514x_detect(struct tvp514x_decoder *decoder)
{
	u8 chip_id_msb, chip_id_lsb, rom_ver;

	chip_id_msb = tvp514x_read_reg(decoder->client, REG_CHIP_ID_MSB);
	chip_id_lsb = tvp514x_read_reg(decoder->client, REG_CHIP_ID_LSB);
	rom_ver = tvp514x_read_reg(decoder->client, REG_ROM_VERSION);

	v4l_dbg(1, debug, decoder->client,
		 "chip id detected msb:0x%x lsb:0x%x rom version:0x%x\n",
		 chip_id_msb, chip_id_lsb, rom_ver);
	if ((chip_id_msb != TVP514X_CHIP_ID_MSB)
		|| ((chip_id_lsb != TVP5146_CHIP_ID_LSB)
		&& (chip_id_lsb != TVP5147_CHIP_ID_LSB))) {
		/* We didn't read the values we expected, so this must not be
		 * an TVP5146/47.
		 */
		v4l_err(decoder->client,
			"chip id mismatch msb:0x%x lsb:0x%x\n",
			chip_id_msb, chip_id_lsb);
		return -ENODEV;
	}

	decoder->ver = rom_ver;
	decoder->state = STATE_DETECTED;

	v4l_info(decoder->client,
			"\n%s found at 0x%x (%s)\n", decoder->client->name,
			decoder->client->addr << 1,
			decoder->client->adapter->name);
	return 0;
}

/*********************
 * VBI related stuff *
 *********************/

struct tvp514x_vbi_type {
	unsigned int vbi_type;
	unsigned int ini_line;
	unsigned int end_line;
	unsigned int by_field:1;
};

static struct tvp514x_vbi_type vbi_supported_services[] = {
	/* Teletext, SECAM, WST System A */
	{V4L2_SLICED_TELETEXT_SECAM, 6, 23, 1},
	/* Teletext, PAL, WST System B */
	{V4L2_SLICED_TELETEXT_PAL_B, 6, 22, 1},
	/* Teletext, PAL, WST System C */
	{V4L2_SLICED_TELETEXT_PAL_C, 6, 22, 1},
	/* Teletext, NTSC, WST System B */
	{V4L2_SLICED_TELETEXT_NTSC_B, 10, 21, 1},
	/* Tetetext, NTSC NABTS System C */
	{V4L2_SLICED_TELETEXT_NTSC_C, 10, 21, 1},
	/* Teletext, NTSC-J, NABTS System D */
	{V4L2_SLICED_TELETEXT_NTSC_D, 10, 21, 1},
	/* Closed Caption, PAL/SECAM */
	{V4L2_SLICED_CAPTION_625, 22, 22, 1},
	/* Closed Caption, NTSC */
	{V4L2_SLICED_CAPTION_525, 21, 21, 1},
	/* Wide Screen Signal, PAL/SECAM */
	{V4L2_SLICED_WSS_625, 23, 23, 1},
	/* Wide Screen Signal, NTSC C */
	{V4L2_SLICED_WSS_525, 20, 20, 1},
	/* Vertical Interval Timecode (VITC), PAL/SECAM */
	{V4L2_SLICED_VITC_625, 6, 22, 0},
	/* Vertical Interval Timecode (VITC), NTSC */
	{V4L2_SLICED_VITC_525, 10, 20, 0},
	/* Video Program System (VPS), PAL */
	{V4L2_SLICED_VPS, 16, 16, 0},
	/* End of struct */
	{(u16) - 1, 0, 0}
};

/* Fills VBI capabilities based on i2c_vbi_ram_value struct */
static void tvp514x_vbi_get_cap(const struct tvp514x_vbi_type
				*supported_services,
				struct v4l2_sliced_vbi_cap *cap)
{
	int line;
	const struct tvp514x_vbi_type *service = supported_services;

	memset(cap, 0, sizeof *cap);

	while (service->vbi_type != (u16) - 1) {
		for (line = service->ini_line;
		     line <= service->end_line; line++) {
			cap->service_lines[0][line] |= service->vbi_type;
			cap->service_lines[1][line] |= service->vbi_type;
		}
		cap->service_set |= service->vbi_type;

		service++;
	}
}

static int tvp514x_get_vbi(struct tvp514x_decoder *decoder, const struct tvp514x_vbi_type
			   *supported_services, int line, const int field)
{
	int std;
	u8 line_mode;
	int type = 0, i = 0;

	std = tvp514x_get_current_std(decoder);
	
	if (std == STD_SECAM || std == STD_PAL_Nc || std == STD_PAL_BDGHIN) {
		/* Don't follow NTSC Line number convension */
		line += 3;
	}

	if (line < 6 || line > 27)
		return 0;

	for ( i=0; i<GENERAL_LINEREGS_NUM; i++ ) {
		if ( tvp514x_vbus_read(decoder->client, 0x800600+i*2) == line ) {
			line_mode = tvp514x_vbus_read(decoder->client, 0x800601+i*2);
			switch (line_mode & 0x07) {
			case 0x0:
				type = V4L2_SLICED_TELETEXT;
				break;
			case 0x01:
				type = V4L2_SLICED_CAPTION;
				break;
			case 0x02:
				type = V4L2_SLICED_WSS;
				break;
			case 0x03:
				type = V4L2_SLICED_VITC;
				break;
			case 0x04:
				type = V4L2_SLICED_VPS;
				break;
			default:
				break;
			}
		}
	}

	return type;
}

/* Set vbi processing
 * type - one of tvp5146_vbi_types
 * line - line to gather data
 * fields: bit 0 field1, bit 1, field2
 * flags (default=0xf0) is a bitmask, were set means:
 *	bit 7: enable filtering null bytes on CC
 *	bit 6: send data also to FIFO
 *	bit 5: don't allow data with errors on FIFO
 *	bit 4: enable ECC when possible
 * pix_align = pix alignment:
 *	LSB = field1
 *	MSB = field2
 * VDP line used vertical date processor VDP
 * Only on Teletextemode and VITC service, the VDP has to process multiple lines, so we use general line mode for the other services
 */

static int tvp514x_set_vbi(struct tvp514x_decoder *decoder, const struct tvp514x_vbi_type
			   *supported_services, unsigned int type,
			   u8 flags, int line, const int fields)
{
	int std;
	u8 line_mode=0;
	int pos = 0, i = 0;
	const struct tvp514x_vbi_type *service = supported_services;
	static int register_occupied = 0;  // count the number of the occupied general line VDP registers, max:9

	std = tvp514x_get_current_std(decoder);

	v4l_dbg(1, debug, decoder->client, "set_vbi type=%x, line=%d\r\n", type, line);

	while (service->vbi_type != (u16) - 1) {
		if ((type & service->vbi_type) &&
		    (line >= service->ini_line) &&
		    (line <= service->end_line)) {
			/*found */
			type = service->vbi_type;
			break;
		}

		service++;
		pos++;
	}
	if (service->vbi_type == (u16) - 1) {
		v4l_err(decoder->client, "Unknown vbi\n");
		return 0;
	}

// 	if (std & V4L2_STD_625_50) {
// 		/* Don't follow NTSC Line number convention */
// 		/* TVP5150 seems to have different line counting */
// 		line += 3;
// 	}

	if (line < 6 || line > 27)
		return 0;

	switch ( type ) {
	case V4L2_SLICED_WSS_625:
	case V4L2_SLICED_WSS_525:
		line_mode = 0x02;
		break;
	case V4L2_SLICED_TELETEXT_SECAM:
	case V4L2_SLICED_TELETEXT_PAL_B:
	case V4L2_SLICED_TELETEXT_PAL_C:
	case V4L2_SLICED_TELETEXT_NTSC_B:
	case V4L2_SLICED_TELETEXT_NTSC_C:
	case V4L2_SLICED_TELETEXT_NTSC_D:
		line_mode = 0x0;
		break;
	case V4L2_SLICED_VITC_625:
	case V4L2_SLICED_VITC_525:
		line_mode = 0x03;
		break;
	case V4L2_SLICED_CAPTION_625:
	case V4L2_SLICED_CAPTION_525:
		line_mode = 0x01;
		break;
	case V4L2_SLICED_VPS:
		line_mode = 0x04;
		break;
	default:
		break;
	}

	if ( fields )
		line_mode |= 0x08;
	else
		line_mode &= ~0x08;

	printk("line mode type:%d.\n",line_mode);

	line_mode |= flags & 0xf0;

	printk("line mode type:%d.\n",line_mode);



	if ( service->ini_line == service->end_line ) {
		/* use general line mode */
		if ( register_occupied == 9 ) {
			v4l_err(decoder->client, "General line mode VDP registers overload\n");
			return 0;
		}
		
		for ( i=0; i<register_occupied; i++ ) {
			if ( tvp514x_vbus_read(decoder->client, 0x800600+i*2) == line ) {
				printk("The service of line %d is already registered\n",line);
				if ( (tvp514x_vbus_read(decoder->client, 0x800601+i*2) | 0xf) == (line_mode | 0xf) ) {
					printk("The same service of line %d is already registered\n",line);
					tvp514x_vbus_write(decoder->client, 0x800601+i*2, line_mode);
					return type;
				}
				printk("Go to regitser another service for line %d\n",line);	
			}
		}

		tvp514x_vbus_write(decoder->client, 0x800600+register_occupied*2, line);
		tvp514x_vbus_write(decoder->client, 0x800601+register_occupied*2, line_mode);
		register_occupied ++;
	} else {
		/* use global line mode */
		tvp514x_write_reg(decoder->client, REG_VDP_LINE_START, service->ini_line);
		tvp514x_write_reg(decoder->client, REG_VDP_LINE_STOP, service->end_line);
		tvp514x_write_reg(decoder->client, REG_VDP_GLOBAL_LINE_MODE, line_mode);
	}
	return type;
}

/*
 * Following are decoder interface functions implemented by
 * TVP5146/47 decoder driver.
 */

/**
 * ioctl_querystd - V4L2 decoder interface handler for VIDIOC_QUERYSTD ioctl
 * @s: pointer to standard V4L2 device structure
 * @std_id: standard V4L2 std_id ioctl enum
 *
 * Returns the current standard detected by TVP5146/47. If no active input is
 * detected, returns -EINVAL
 */
static int ioctl_querystd(struct v4l2_int_device *s, v4l2_std_id *std_id)
{
	struct tvp514x_decoder *decoder = s->priv;
	enum tvp514x_std current_std;
	enum tvp514x_input input_sel;
	u8 sync_lock_status, lock_mask;

	if (std_id == NULL)
		return -EINVAL;

	/* get the current standard */
	current_std = tvp514x_get_current_std(decoder);
	if (current_std == STD_INVALID) {
		decoder->current_std = current_std;
		*std_id = 0;
		return 0;
	}

#ifdef LOCKED_INPUT_NEEDED
	input_sel = decoder->route.input;

	switch (input_sel) {
	case INPUT_CVBS_VI1A:
	case INPUT_CVBS_VI1B:
	case INPUT_CVBS_VI1C:
	case INPUT_CVBS_VI2A:
	case INPUT_CVBS_VI2B:
	case INPUT_CVBS_VI2C:
	case INPUT_CVBS_VI3A:
	case INPUT_CVBS_VI3B:
	case INPUT_CVBS_VI3C:
	case INPUT_CVBS_VI4A:
		lock_mask = STATUS_CLR_SUBCAR_LOCK_BIT |
			STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;

	case INPUT_SVIDEO_VI2A_VI1A:
	case INPUT_SVIDEO_VI2B_VI1B:
	case INPUT_SVIDEO_VI2C_VI1C:
	case INPUT_SVIDEO_VI2A_VI3A:
	case INPUT_SVIDEO_VI2B_VI3B:
	case INPUT_SVIDEO_VI2C_VI3C:
	case INPUT_SVIDEO_VI4A_VI1A:
	case INPUT_SVIDEO_VI4A_VI1B:
	case INPUT_SVIDEO_VI4A_VI1C:
	case INPUT_SVIDEO_VI4A_VI3A:
	case INPUT_SVIDEO_VI4A_VI3B:
	case INPUT_SVIDEO_VI4A_VI3C:
		lock_mask = STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;
	case INPUT_RGB_VI1A_VI2A_VI3A:
	case INPUT_YPbPr_VI1A_VI2A_VI3A:
	case INPUT_SCART_VI1A_VI2A_VI3A_VI4A:
		lock_mask = STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT; /* Other locks to check? */
		break;
		/*Need to add other interfaces*/
	default:
		return -EINVAL;
	}
	/* check whether signal is locked */
	sync_lock_status = tvp514x_read_reg(decoder->client, REG_STATUS1);
	if (lock_mask != (sync_lock_status & lock_mask))
		return -EINVAL;	/* No input detected */
#endif

	decoder->current_std = current_std;
	*std_id = decoder->std_list[current_std].standard.id;

	v4l_dbg(1, debug, decoder->client, "Current STD: %s",
			decoder->std_list[current_std].standard.name);
	return 0;
}

/**
 * ioctl_s_std - V4L2 decoder interface handler for VIDIOC_S_STD ioctl
 * @s: pointer to standard V4L2 device structure
 * @std_id: standard V4L2 v4l2_std_id ioctl enum
 *
 * If std_id is supported, sets the requested standard. Otherwise, returns
 * -EINVAL
 */
static int ioctl_s_std(struct v4l2_int_device *s, v4l2_std_id *std_id)
{
	struct tvp514x_decoder *decoder = s->priv;
	int err, i;

	if (std_id == NULL)
		return -EINVAL;

	/* Autoswitch mode requested */
	if(*std_id == V4L2_STD_ALL) {
		err = tvp514x_write_reg(decoder->client, REG_VIDEO_STD,
				VIDEO_STD_AUTO_SWITCH_BIT);
		if (err) return err;
		tvp514x_reg_list[REG_VIDEO_STD].val = VIDEO_STD_AUTO_SWITCH_BIT;
		decoder->current_std = tvp514x_get_current_std(decoder);
		v4l_dbg(1, debug, decoder->client, "Standard set to: Auto");
		return 0;
	}
		
	for (i = 0; i < decoder->num_stds; i++)
		if (*std_id & decoder->std_list[i].standard.id)
			break;

	if ((i == decoder->num_stds) || (i == STD_INVALID))
		return -EINVAL;

	err = tvp514x_write_reg(decoder->client, REG_VIDEO_STD,
				decoder->std_list[i].video_std);
	if (err)
		return err;

	decoder->current_std = i;
	tvp514x_reg_list[REG_VIDEO_STD].val = decoder->std_list[i].video_std;

	v4l_dbg(1, debug, decoder->client, "Standard set to: %s",
			decoder->std_list[i].standard.name);
	return 0;
}

/**
 * ioctl_s_routing - V4L2 decoder interface handler for VIDIOC_S_INPUT ioctl
 * @s: pointer to standard V4L2 device structure
 * @index: number of the input
 *
 * If index is valid, selects the requested input. Otherwise, returns -EINVAL if
 * the input is not supported or there is no active signal present in the
 * selected input.
 */
static int ioctl_s_routing(struct v4l2_int_device *s,
				struct v4l2_routing *route)
{
	struct tvp514x_decoder *decoder = s->priv;
	int err;
	enum tvp514x_input input_sel;
	enum tvp514x_output output_sel;
	enum tvp514x_std current_std = STD_INVALID;
	u8 sync_lock_status, lock_mask;
	int try_count = LOCK_RETRY_COUNT;

	if ((!route) || (route->input >= INPUT_INVALID) ||
			(route->output >= OUTPUT_INVALID))
		return -EINVAL;	/* Index out of bound */

	input_sel = route->input;
	output_sel = route->output;

	err = tvp514x_write_reg(decoder->client, REG_INPUT_SEL, input_sel);
	if (err)
		return err;

	output_sel |= tvp514x_read_reg(decoder->client,
			REG_OUTPUT_FORMATTER1) & 0x7;
	err = tvp514x_write_reg(decoder->client, REG_OUTPUT_FORMATTER1,
			output_sel);
	if (err)
		return err;

	tvp514x_reg_list[REG_INPUT_SEL].val = input_sel;
	tvp514x_reg_list[REG_OUTPUT_FORMATTER1].val = output_sel;

	/* Clear status */
	msleep(LOCK_RETRY_DELAY);
	err =
	    tvp514x_write_reg(decoder->client, REG_CLEAR_LOST_LOCK, 0x01);
	if (err)
		return err;

	switch (input_sel) {
	case INPUT_CVBS_VI1A:
	case INPUT_CVBS_VI1B:
	case INPUT_CVBS_VI1C:
	case INPUT_CVBS_VI2A:
	case INPUT_CVBS_VI2B:
	case INPUT_CVBS_VI2C:
	case INPUT_CVBS_VI3A:
	case INPUT_CVBS_VI3B:
	case INPUT_CVBS_VI3C:
	case INPUT_CVBS_VI4A:
		lock_mask = STATUS_CLR_SUBCAR_LOCK_BIT |
			STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		/* From the Gen6 driver */
		tvp514x_write_reg(decoder->client, REG_LUMA_CONTROL2, 0x04);
		break;

	case INPUT_SVIDEO_VI2A_VI1A:
	case INPUT_SVIDEO_VI2B_VI1B:
	case INPUT_SVIDEO_VI2C_VI1C:
	case INPUT_SVIDEO_VI2A_VI3A:
	case INPUT_SVIDEO_VI2B_VI3B:
	case INPUT_SVIDEO_VI2C_VI3C:
	case INPUT_SVIDEO_VI4A_VI1A:
	case INPUT_SVIDEO_VI4A_VI1B:
	case INPUT_SVIDEO_VI4A_VI1C:
	case INPUT_SVIDEO_VI4A_VI3A:
	case INPUT_SVIDEO_VI4A_VI3B:
	case INPUT_SVIDEO_VI4A_VI3C:
		lock_mask = STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		/* From the Gen6 driver */
		tvp514x_write_reg(decoder->client, REG_LUMA_CONTROL2, 0x4C);
		break;
	case INPUT_RGB_VI1A_VI2A_VI3A:
	case INPUT_YPbPr_VI1A_VI2A_VI3A:
	case INPUT_SCART_VI1A_VI2A_VI3A_VI4A:
		lock_mask = STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT; /* Other locks to check? */
		break;
		/*Need to add other interfaces*/
	default:
		return -EINVAL;
	}

#ifdef LOCKED_INPUT_NEEDED
	while (try_count-- > 0) {
		/* Allow decoder to sync up with new input */
		msleep(LOCK_RETRY_DELAY);

		/* get the current standard for future reference */
		current_std = tvp514x_get_current_std(decoder);
		if (current_std == STD_INVALID)
			continue;

		sync_lock_status = tvp514x_read_reg(decoder->client,
				REG_STATUS1);
		if (lock_mask == (sync_lock_status & lock_mask))
			break;	/* Input detected */
	}

	if ((current_std == STD_INVALID) || (try_count < 0)) {
		v4l_dbg(1, debug, decoder->client,
			"TVP514x not locked, input requested : %d",
			input_sel);
		return -EINVAL;
	}
#else
	current_std = tvp514x_get_current_std(decoder);
#endif

	decoder->current_std = current_std;
	decoder->route.input = route->input;
	decoder->route.output = route->output;

	v4l_dbg(1, debug, decoder->client,
			"Input set to: %d, std : %d",
			input_sel, current_std);

	return 0;
}

/**
 * ioctl_queryctrl - V4L2 decoder interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the ctrl_list[] array. Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int
ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qctrl)
{
	struct tvp514x_decoder *decoder = s->priv;
	int id, index;
	const struct tvp514x_ctrl_info *control = NULL;

	if (qctrl == NULL)
		return -EINVAL;

	id = qctrl->id;
	memset(qctrl, 0, sizeof(struct v4l2_queryctrl));
	qctrl->id = id;

	for (index = 0; index < decoder->num_ctrls; index++) {
		control = &decoder->ctrl_list[index];
		if (control->query_ctrl.id == qctrl->id)
			break;	/* Match found */
	}
	if (index == decoder->num_ctrls)
		return -EINVAL;	/* Index out of bound */

	memcpy(qctrl, &control->query_ctrl, sizeof(struct v4l2_queryctrl));

	v4l_dbg(1, debug, decoder->client,
			"Query Control: %s : Min - %d, Max - %d, Def - %d",
			control->query_ctrl.name,
			control->query_ctrl.minimum,
			control->query_ctrl.maximum,
			control->query_ctrl.default_value);
	return 0;
}

/**
 * ioctl_g_ctrl - V4L2 decoder interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the decoder. Otherwise, returns -EINVAL if the control is not
 * supported.
 */
static int
ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *ctrl)
{
	struct tvp514x_decoder *decoder = s->priv;
	int index, value;
	const struct tvp514x_ctrl_info *control = NULL;

	if (ctrl == NULL)
		return -EINVAL;

	for (index = 0; index < decoder->num_ctrls; index++) {
		control = &decoder->ctrl_list[index];
		if (control->query_ctrl.id == ctrl->id)
			break;	/* Match found */
	}
	if (index == decoder->num_ctrls)
		return -EINVAL;	/* Index out of bound */

	switch(decoder->route.input) {
	case INPUT_CVBS_VI1A:
	case INPUT_CVBS_VI1B:
	case INPUT_CVBS_VI1C:
	case INPUT_CVBS_VI2A:
	case INPUT_CVBS_VI2B:
	case INPUT_CVBS_VI2C:
	case INPUT_CVBS_VI3A:
	case INPUT_CVBS_VI3B:
	case INPUT_CVBS_VI3C:
	case INPUT_CVBS_VI4A:
	case INPUT_SVIDEO_VI2A_VI1A:
	case INPUT_SVIDEO_VI2B_VI1B:
	case INPUT_SVIDEO_VI2C_VI1C:
	case INPUT_SVIDEO_VI2A_VI3A:
	case INPUT_SVIDEO_VI2B_VI3B:
	case INPUT_SVIDEO_VI2C_VI3C:
	case INPUT_SVIDEO_VI4A_VI1A:
	case INPUT_SVIDEO_VI4A_VI1B:
	case INPUT_SVIDEO_VI4A_VI1C:
	case INPUT_SVIDEO_VI4A_VI3A:
	case INPUT_SVIDEO_VI4A_VI3B:
	case INPUT_SVIDEO_VI4A_VI3C:
		if(control->regs.cvbs_reg_address) {
			value = tvp514x_read_reg(decoder->client,
					control->regs.cvbs_reg_address);
			/* cross check */
			if (value != tvp514x_reg_list[control->regs.cvbs_reg_address].val)
				return -EINVAL;	/* Driver & TVP5146/47 setting mismatch */
		}
		else return -EINVAL;
		if(control->regs.cvbs_reg_msb_address) {
			value |= (tvp514x_read_reg(decoder->client,
					control->regs.cvbs_reg_msb_address)
					& 0x0F) << 8;
			/* cross check */
			if (value != tvp514x_reg_list[control->regs.cvbs_reg_msb_address].val)
				return -EINVAL;	/* Driver & TVP5146/47 setting mismatch */
		}
		break;
	case INPUT_YPbPr_VI1A_VI2A_VI3A:
		if(control->regs.component_reg_address) {
			value = tvp514x_read_reg(decoder->client,
					control->regs.component_reg_address);
			/* cross check */
			if (value != tvp514x_reg_list[control->regs.component_reg_address].val)
				return -EINVAL;	/* Driver & TVP5146/47 setting mismatch */
		}
		else return -EINVAL;
		if(control->regs.component_reg_msb_address) {
			value |= (tvp514x_read_reg(decoder->client,
					control->regs.component_reg_msb_address)
					& 0x0F) << 8;
			/* cross check */
			if (value != tvp514x_reg_list[control->regs.component_reg_msb_address].val)
				return -EINVAL;	/* Driver & TVP5146/47 setting mismatch */
		}
		break;
	case INPUT_RGB_VI1A_VI2A_VI3A:
	case INPUT_SCART_VI1A_VI2A_VI3A_VI4A:
		if(control->regs.rgb_reg_address) {
			value = tvp514x_read_reg(decoder->client,
					control->regs.rgb_reg_address);
			/* cross check */
			if (value != tvp514x_reg_list[control->regs.rgb_reg_address].val)
				return -EINVAL;	/* Driver & TVP5146/47 setting mismatch */
		}
		else return -EINVAL;
		if(control->regs.rgb_reg_msb_address) {
			value |= (tvp514x_read_reg(decoder->client,
					control->regs.rgb_reg_msb_address)
					& 0x0F) << 8;
			/* cross check */
			if (value != tvp514x_reg_list[control->regs.rgb_reg_msb_address].val)
				return -EINVAL;	/* Driver & TVP5146/47 setting mismatch */
		}
		break;
	default:
		return -EINVAL;
	}

	if (V4L2_CID_AUTOGAIN == ctrl->id) {
		if ((value & 0x3) == 3)
			value = 1;
		else
			value = 0;
	}

	if (V4L2_CID_HUE == ctrl->id) {
		if (value == 0x7F)
			value = 180;
		else if (value == 0x80)
			value = -180;
		else
			value = 0;
	}

	if (V4L2_CID_GREEN_GAIN == ctrl->id ||
		V4L2_CID_BLUE_GAIN == ctrl->id ||
		V4L2_CID_RED_GAIN == ctrl->id) {
		value = 0x400 + (30 * value);
	}

	ctrl->value = value;

	v4l_dbg(1, debug, decoder->client,
			"Get Cotrol: %s - %d",
			control->query_ctrl.name, value);
	return 0;
}

/**
 * ioctl_s_ctrl - V4L2 decoder interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW. Otherwise, returns -EINVAL if the control is not supported.
 */
static int
ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *ctrl)
{
	struct tvp514x_decoder *decoder = s->priv;
	int err, value, index;
	const struct tvp514x_ctrl_info *control = NULL;

	if (ctrl == NULL)
		return -EINVAL;

	value = (__s32) ctrl->value;
	for (index = 0; index < decoder->num_ctrls; index++) {
		control = &decoder->ctrl_list[index];
		if (control->query_ctrl.id == ctrl->id)
			break;	/* Match found */
	}
	if (index == decoder->num_ctrls)
		return -EINVAL;	/* Index out of bound */

	if (V4L2_CID_AUTOGAIN == ctrl->id) {
		if (value == 1)
			value = 0x0F;
		else if (value == 0)
			value = 0x0C;
		else
			return -ERANGE;
	} else if (V4L2_CID_GREEN_GAIN == ctrl->id ||
			V4L2_CID_BLUE_GAIN == ctrl->id ||
			V4L2_CID_RED_GAIN == ctrl->id) {
		value = 0x400 + (30 * value);
	} else {
		if ((value < control->query_ctrl.minimum)
			|| (value > control->query_ctrl.maximum))
			return -ERANGE;
	}

	switch(decoder->route.input) {
	case INPUT_CVBS_VI1A:
	case INPUT_CVBS_VI1B:
	case INPUT_CVBS_VI1C:
	case INPUT_CVBS_VI2A:
	case INPUT_CVBS_VI2B:
	case INPUT_CVBS_VI2C:
	case INPUT_CVBS_VI3A:
	case INPUT_CVBS_VI3B:
	case INPUT_CVBS_VI3C:
	case INPUT_CVBS_VI4A:
	case INPUT_SVIDEO_VI2A_VI1A:
	case INPUT_SVIDEO_VI2B_VI1B:
	case INPUT_SVIDEO_VI2C_VI1C:
	case INPUT_SVIDEO_VI2A_VI3A:
	case INPUT_SVIDEO_VI2B_VI3B:
	case INPUT_SVIDEO_VI2C_VI3C:
	case INPUT_SVIDEO_VI4A_VI1A:
	case INPUT_SVIDEO_VI4A_VI1B:
	case INPUT_SVIDEO_VI4A_VI1C:
	case INPUT_SVIDEO_VI4A_VI3A:
	case INPUT_SVIDEO_VI4A_VI3B:
	case INPUT_SVIDEO_VI4A_VI3C:
		if(control->regs.cvbs_reg_address) {
			err = tvp514x_write_reg(decoder->client,
				control->regs.cvbs_reg_address, value & 0xFF);
			if (err) return err;
			tvp514x_reg_list[control->regs.cvbs_reg_address].val = value;
		}
		else return -EINVAL;
		if(control->regs.cvbs_reg_msb_address) {
			err = tvp514x_write_reg(decoder->client,
					control->regs.cvbs_reg_msb_address,
					(value >> 8) & 0x0F);
			if (err) return err;
			tvp514x_reg_list[control->regs.cvbs_reg_msb_address].val = value;
		}
		break;
	case INPUT_YPbPr_VI1A_VI2A_VI3A:
		if(control->regs.component_reg_address) {
			err = tvp514x_write_reg(decoder->client,
				control->regs.component_reg_address, value & 0xFF);
			if (err) return err;
			tvp514x_reg_list[control->regs.component_reg_address].val = value;
		}
		else return -EINVAL;
		if(control->regs.component_reg_msb_address) {
			err = tvp514x_write_reg(decoder->client,
					control->regs.component_reg_msb_address,
					(value >> 8) & 0x0F);
			if (err) return err;
			tvp514x_reg_list[control->regs.component_reg_msb_address].val = value;
		}
		break;
	case INPUT_RGB_VI1A_VI2A_VI3A:
	case INPUT_SCART_VI1A_VI2A_VI3A_VI4A:
		if(control->regs.rgb_reg_address) {
			err = tvp514x_write_reg(decoder->client,
				control->regs.rgb_reg_address, value & 0xFF);
			if (err) return err;
			tvp514x_reg_list[control->regs.rgb_reg_address].val = value;
		}
		else return -EINVAL;
		if(control->regs.rgb_reg_msb_address) {
			err = tvp514x_write_reg(decoder->client,
					control->regs.rgb_reg_msb_address,
					(value >> 8) & 0x0F);
			if (err) return err;
			tvp514x_reg_list[control->regs.rgb_reg_msb_address].val = value;
		}
		break;
	default:
		return -EINVAL;
	}

	v4l_dbg(1, debug, decoder->client,
			"Set Cotrol: %s - %d",
			control->query_ctrl.name, value);
	return err;
}

/**
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl to enumerate supported formats
 */
static int
ioctl_enum_fmt_cap(struct v4l2_int_device *s, struct v4l2_fmtdesc *fmt)
{
	struct tvp514x_decoder *decoder = s->priv;
	int index;

	if (fmt == NULL)
		return -EINVAL;

	index = fmt->index;
	if ((index >= decoder->num_fmts) || (index < 0))
		return -EINVAL;	/* Index out of bound */

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	memcpy(fmt, &decoder->fmt_list[index],
		sizeof(struct v4l2_fmtdesc));

	v4l_dbg(1, debug, decoder->client,
			"Current FMT: index - %d (%s)",
			decoder->fmt_list[index].index,
			decoder->fmt_list[index].description);
	return 0;
}

/**
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type. This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int
ioctl_try_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct tvp514x_decoder *decoder = s->priv;
	int ifmt = 0;
	struct v4l2_pix_format *pix;
//	enum tvp514x_std current_std;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	pix = &f->fmt.pix;
#if 0
	/* Calculate height and width based on current standard */
	current_std = tvp514x_get_current_std(decoder);
	if (current_std == STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;
	pix->width = decoder->std_list[current_std].width;
	pix->height = decoder->std_list[current_std].height;

	for (ifmt = 0; ifmt < decoder->num_fmts; ifmt++) {
		if (pix->pixelformat ==
			decoder->fmt_list[ifmt].pixelformat)
			break;
	}
	if (ifmt == decoder->num_fmts)
		ifmt = 0;	/* None of the format matched, select default */
	pix->pixelformat = decoder->fmt_list[ifmt].pixelformat;
#else
	pix->pixelformat = V4L2_PIX_FMT_UYVY;
#endif
	pix->field = V4L2_FIELD_INTERLACED;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	pix->priv = 0;

	v4l_dbg(1, debug, decoder->client,
			"Try FMT: pixelformat - %s, bytesperline - %d"
			"Width - %d, Height - %d",
			decoder->fmt_list[ifmt].description, pix->bytesperline,
			pix->width, pix->height);
	return 0;
}

/**
 * ioctl_s_fmt_cap - V4L2 decoder interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int
ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct tvp514x_decoder *decoder = s->priv;
	struct v4l2_pix_format *pix;
	struct v4l2_sliced_vbi_format *svbi;
	int rval;

	if (f == NULL)
		return -EINVAL;

	switch(f->type) {
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
			pix = &f->fmt.pix;
			rval = ioctl_try_fmt_cap(s, f);
			if (rval)
				return rval;
			else
				decoder->pix = *pix;
		
			return rval;

		case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
			svbi = &f->fmt.sliced;
	
			/* Fix me: user could set service_set = 0
					and fill service_lines instead */
			if (svbi->service_set & V4L2_SLICED_WSS_625) {
				/* why if we set the field to 1,
					the wss info is always zero??*/
				if (!tvp514x_set_vbi(decoder, vbi_supported_services,
				V4L2_SLICED_WSS_625, 0, 23, 0))
					return -EINVAL;
	
			} else if (svbi->service_set & V4L2_SLICED_WSS_525) {
				if (!tvp514x_set_vbi(decoder, vbi_supported_services,
				V4L2_SLICED_WSS_525, 0, 20, 0))
					return -EINVAL;
			}
			return 0;

		default :
			break;
	}

	return -EINVAL;
}

/**
 * ioctl_g_fmt_cap - V4L2 decoder interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the decoder's current pixel format in the v4l2_format
 * parameter.
 */
static int
ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct tvp514x_decoder *decoder = s->priv;

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	f->fmt.pix = decoder->pix;

	v4l_dbg(1, debug, decoder->client,
			"Current FMT: bytesperline - %d"
			"Width - %d, Height - %d",
			decoder->pix.bytesperline,
			decoder->pix.width, decoder->pix.height);
	return 0;
}

/**
 * ioctl_g_parm - V4L2 decoder interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the decoder's video CAPTURE parameters.
 */
static int
ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct tvp514x_decoder *decoder = s->priv;
	struct v4l2_captureparm *cparm;
	enum tvp514x_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	/* get the current standard */
	current_std = tvp514x_get_current_std(decoder);
	if (current_std == STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;

	cparm = &a->parm.capture;
	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe =
		decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/**
 * ioctl_s_parm - V4L2 decoder interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the decoder to use the input parameters, if possible. If
 * not possible, returns the appropriate error code.
 */
static int
ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct tvp514x_decoder *decoder = s->priv;
	struct v4l2_fract *timeperframe;
	enum tvp514x_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;	/* only capture is supported */

	timeperframe = &a->parm.capture.timeperframe;

	/* get the current standard */
	current_std = tvp514x_get_current_std(decoder);
	if (current_std == STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;

	*timeperframe =
	    decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/**
 * ioctl_g_ifparm - V4L2 decoder interface handler for vidioc_int_g_ifparm_num
 * @s: pointer to standard V4L2 device structure
 * @p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p. This value is returned in the p
 * parameter.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct tvp514x_decoder *decoder = s->priv;
	int rval;

	if (p == NULL)
		return -EINVAL;

	if (NULL == decoder->pdata->ifparm)
		return -EINVAL;

	rval = decoder->pdata->ifparm(p);
	if (rval) {
		v4l_err(decoder->client, "g_ifparm.Err[%d]\n", rval);
		return rval;
	}

	p->u.bt656.clock_curr = TVP514X_XCLK_BT656;

	return 0;
}

/**
 * ioctl_g_priv - V4L2 decoder interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold decoder's private data address
 *
 * Returns device's (decoder's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct tvp514x_decoder *decoder = s->priv;

	if (NULL == decoder->pdata->priv_data_set)
		return -EINVAL;

	return decoder->pdata->priv_data_set(p);
}

/**
 * ioctl_s_power - V4L2 decoder interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct tvp514x_decoder *decoder = s->priv;
	int err = 0;

	switch (on) {
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		err =
		    tvp514x_write_reg(decoder->client, REG_OPERATION_MODE,
					0x01);
		/* Disable mux for TVP5146/47 decoder data path */
		if (decoder->pdata->power_set)
			err |= decoder->pdata->power_set(on);
		decoder->state = STATE_NOT_DETECTED;
		break;

	case V4L2_POWER_STANDBY:
		if (decoder->pdata->power_set)
			err = decoder->pdata->power_set(on);
		break;

	case V4L2_POWER_ON:
		/* Enable mux for TVP5146/47 decoder data path */
		if ((decoder->pdata->power_set) &&
				(decoder->state == STATE_NOT_DETECTED)) {
			int i;
			struct tvp514x_init_seq *int_seq =
				(struct tvp514x_init_seq *)
				decoder->id->driver_data;

			err = decoder->pdata->power_set(on);

			/* Power Up Sequence */
			for (i = 0; i < int_seq->no_regs; i++) {
				err |= tvp514x_write_reg(decoder->client,
						int_seq->init_reg_seq[i].reg,
						int_seq->init_reg_seq[i].val);
			}
			/* Detect the sensor is not already detected */
			err |= tvp514x_detect(decoder);
			if (err) {
				v4l_err(decoder->client,
						"Unable to detect decoder\n");
				return err;
			}
		}
		err |= tvp514x_configure(decoder);
		break;

	default:
		err = -ENODEV;
		break;
	}

	return err;
}

/**
 * ioctl_init - V4L2 decoder interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the decoder device (calls tvp514x_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	struct tvp514x_decoder *decoder = s->priv;

	/* Set default standard to auto */
	tvp514x_reg_list[REG_VIDEO_STD].val =
	    VIDEO_STD_AUTO_SWITCH_BIT;

	return tvp514x_configure(decoder);
}

/**
 * ioctl_dev_exit - V4L2 decoder interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach. The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_init - V4L2 decoder interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master. Returns 0 if
 * TVP5146/47 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct tvp514x_decoder *decoder = s->priv;
	int err;

	err = tvp514x_detect(decoder);
	if (err < 0) {
		v4l_err(decoder->client,
			"Unable to detect decoder\n");
		return err;
	}

	v4l_info(decoder->client,
		 "chip version 0x%.2x detected\n", decoder->ver);

	return 0;
}

/**
 * ioctl_priv_querystatus - V4L2 decoder interface handler for
 * vidioc_int_priv_querystatus_num
 * @s: pointer to standard V4L2 device structure
 * @status: where to store the status
 *
 * Reads the status of the TVP514x (especially the macrovision status).
 */
static int ioctl_priv_querystatus(struct v4l2_int_device *s, u32 *status)
{
	struct tvp514x_decoder *decoder = s->priv;
	u8 reg1, reg2;
	static int stat1, stat2;
	reg1 = tvp514x_read_reg(decoder->client, REG_STATUS1);
	reg2 = tvp514x_read_reg(decoder->client, REG_STATUS2);
	v4l_dbg(1, debug, decoder->client, "tvp status register 1 & 2: 0x%02x 0x%02x\n",
		    reg1, reg2);

	if ((stat1 != reg1) || ((stat2 & 0x0f) != (reg2 & 0x0f))) {
		printk("tvp status register 1 & 2: 0x%02x 0x%02x\n",
		     reg1, reg2);
		stat1 = reg1;
		stat2 = reg2;
	}
	*status = 0;
	if (!(reg1 & (1 << 1))) {
		// Horizontal Sync is lost
		*status |= V4L2_IN_ST_NO_H_LOCK;
	}
	if (!(reg1 & (1 << 2))) {
		// Vertical Sync is lost
		*status |= V4L2_IN_ST_NO_SIGNAL;
	}
	if (!(reg1 & (1 << 3))) {
		// Color Sync is lost
		*status |= V4L2_IN_ST_NO_COLOR;
	}

	switch(reg2 & 0x7) {
		case STATUS_MACROVISION_DETECT_TYPE1:
			// AGC (Macrovision Type 1) detection
			*status |= V4L2_IN_ST_MACROVISION;
			*status |= V4L2_IN_ST_MACROVISION_TYPE1;
			break;
		case STATUS_MACROVISION_DETECT_TYPE2:
			// Colorstripe process Type 2 present (ignored)
			*status |= V4L2_IN_ST_MACROVISION;
			*status |= V4L2_IN_ST_MACROVISION_TYPE2;
			break;
		case STATUS_MACROVISION_DETECT_TYPE3:
			// Colorstripe process Type 3 present (ignored)
			*status |= V4L2_IN_ST_MACROVISION;
			*status |= V4L2_IN_ST_MACROVISION_TYPE3;
			break;
	}

	return 0;
}

/**
 * ioctl_priv_g_sliced_vbi_cap - V4L2 decoder interface handler for
 * vidioc_int_priv_g_sliced_vbi_cap_num
 * @s: pointer to standard V4L2 device structure
 * @cap : pointer to the v4l2_sliced_vbi_cap structure
 *
 * Fills VBI capabilities based on i2c_vbi_ram_value struct.
 */
static int ioctl_priv_g_sliced_vbi_cap(struct v4l2_int_device *s, struct v4l2_sliced_vbi_cap *cap)
{
	tvp514x_vbi_get_cap(vbi_supported_services, cap);
	return 0;
}

/**
 * ioctl_priv_g_vbi_data - V4L2 decoder interface handler for
 * vidioc_int_priv_g_vbi_data_num
 * @s: pointer to standard V4L2 device structure
 *
 * Reads the VBI data. This will not work for USB devices.
 */
static int ioctl_priv_g_vbi_data(struct v4l2_int_device *s, struct v4l2_sliced_vbi_data *data)
{
		// fixme
	struct tvp514x_decoder *decoder = s->priv;

	if (data->id & V4L2_SLICED_CAPTION) {
		/*if (!field && (vdp_status&0x10)) {
		   data->data[0]=tvp5146_read(c, TVP5150_CC_DATA_INI);
		   data->data[1]=tvp5146_read(c, TVP5150_CC_DATA_INI+1);
		   } if (field && (vdp_status&0x8)) {
		   data->data[0]=tvp5146_read(c, TVP5150_CC_DATA_INI+2);
		   data->data[1]=tvp5146_read(c, TVP5150_CC_DATA_INI+3);
		   } else data->id=0;
		   return 0; */
	} else if (data->id & V4L2_SLICED_WSS) {
		if ( tvp514x_read_reg(decoder->client,
		 REG_INTERRUPT_RAW_STATUS0) & 0x20 ) {
			data->field = 0;
			data->data[0] =
			 tvp514x_vbus_read(decoder->client, 0x800520);
			data->data[1] =
			 tvp514x_vbus_read(decoder->client, 0x800521);
			data->data[2] =
			 tvp514x_vbus_read(decoder->client, 0x800522);
			tvp514x_write_reg(decoder->client,
			 REG_INTERRUPT_CLEAR0,  0x20);
		} else {
			data->id = 0;
//			printk(KERN_ERR "wss is not available\n");
		}
	} else if (data->id & V4L2_SLICED_VPS) {
		printk(KERN_ERR "bad vbi1\n");
		data->id = 0;
	} else {
		printk(KERN_ERR "bad vbi2\n");
		data->id = 0;
	}

	return 0;
}

static struct v4l2_int_ioctl_desc tvp514x_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*) ioctl_dev_init},
	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func*) ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
	{vidioc_int_g_priv_num, (v4l2_int_ioctl_func*) ioctl_g_priv},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
	{vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
	{vidioc_int_try_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_try_fmt_cap},
	{vidioc_int_g_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
	{vidioc_int_queryctrl_num,
	 (v4l2_int_ioctl_func *) ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_querystd_num, (v4l2_int_ioctl_func *) ioctl_querystd},
	{vidioc_int_s_std_num, (v4l2_int_ioctl_func *) ioctl_s_std},
	{vidioc_int_s_video_routing_num,
		(v4l2_int_ioctl_func *) ioctl_s_routing},
	{vidioc_int_priv_querystatus_num,
	 (v4l2_int_ioctl_func *) ioctl_priv_querystatus},
	{vidioc_int_priv_g_sliced_vbi_cap_num,
	 (v4l2_int_ioctl_func *) ioctl_priv_g_sliced_vbi_cap},
	{vidioc_int_priv_g_vbi_data_num,
	 (v4l2_int_ioctl_func *) ioctl_priv_g_vbi_data},
};

static struct v4l2_int_slave tvp514x_slave = {
	.ioctls = tvp514x_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tvp514x_ioctl_desc),
};

static struct tvp514x_decoder tvp514x_dev = {
	.state = STATE_NOT_DETECTED,

	.num_fmts = TVP514X_NUM_FORMATS,
	.fmt_list = tvp514x_fmt_list,

	.pix = {		/* Default to PAL 8-bit YUV 422 */
		.width = 640, //PAL_NUM_ACTIVE_PIXELS,
		.height = 480, //PAL_NUM_ACTIVE_LINES,
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.field = V4L2_FIELD_INTERLACED,
		.bytesperline = 640*2, //PAL_NUM_ACTIVE_PIXELS * 2,
		.sizeimage =
		640*2*480, //PAL_NUM_ACTIVE_PIXELS * 2 * PAL_NUM_ACTIVE_LINES,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		},

	.current_std = STD_PAL_BDGHIN,
	.num_stds = TVP514X_NUM_STANDARDS,
	.std_list = tvp514x_std_list,

	.num_ctrls = TVP514X_NUM_CONTROLS,
	.ctrl_list = tvp514x_ctrl_list,

};

static struct v4l2_int_device tvp514x_int_device = {
	.module = THIS_MODULE,
	.name = MODULE_NAME,
	.priv = &tvp514x_dev,
	.type = v4l2_int_type_slave,
	.u = {
	      .slave = &tvp514x_slave,
	      },
};

static ssize_t show_gpio_c6(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *c = to_i2c_client(dev);
	int value;
	
	value = tvp514x_read_reg(c, REG_OUTPUT_FORMATTER6);
	if (value < 0)
		return -EIO;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", !!(value & TVP514X_GPIO_C6_OUTPUT));
}

static ssize_t store_gpio_c6(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct i2c_client *c = to_i2c_client(dev);
	int value;
	
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	value = tvp514x_read_reg(c, REG_OUTPUT_FORMATTER6);
	if (value < 0)
		return -EIO;

	if (on_off)
		value |= TVP514X_GPIO_C6_OUTPUT;
	else
		value &= ~TVP514X_GPIO_C6_OUTPUT;

	tvp514x_write_reg(c, REG_OUTPUT_FORMATTER6, value);
	return count;
}

static DEVICE_ATTR(gpio_c6, S_IRUGO|S_IWUSR, show_gpio_c6, store_gpio_c6);

static ssize_t show_gpio_c7(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *c = to_i2c_client(dev);
	int value;
	
	value = tvp514x_read_reg(c, REG_OUTPUT_FORMATTER6);
	if (value < 0)
		return -EIO;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", !!(value & TVP514X_GPIO_C7_OUTPUT));
}

static ssize_t store_gpio_c7(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct i2c_client *c = to_i2c_client(dev);
	int value;
	
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	value = tvp514x_read_reg(c, REG_OUTPUT_FORMATTER6);
	if (value < 0)
		return -EIO;

	if (on_off)
		value |= TVP514X_GPIO_C7_OUTPUT;
	else
		value &= ~TVP514X_GPIO_C7_OUTPUT;

	tvp514x_write_reg(c, REG_OUTPUT_FORMATTER6, value);
	return count;
}

static DEVICE_ATTR(gpio_c7, S_IRUGO|S_IWUSR, show_gpio_c7, store_gpio_c7);

/**
 * tvp514x_probe - decoder driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register decoder as an i2c client device and V4L2
 * device.
 */
static int
tvp514x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tvp514x_decoder *decoder = &tvp514x_dev;
	int err;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	decoder->pdata = client->dev.platform_data;
	if (!decoder->pdata) {
		v4l_err(client, "No platform data\n!!");
		return -ENODEV;
	}
	/*
	 * Fetch platform specific data, and configure the
	 * tvp514x_reg_list[] accordingly. Since this is one
	 * time configuration, no need to preserve.
	 */
	tvp514x_reg_list[REG_OUTPUT_FORMATTER2].val |=
			(decoder->pdata->clk_polarity << 1);
	tvp514x_reg_list[REG_SYNC_CONTROL].val |=
			((decoder->pdata->hs_polarity << 2) |
			(decoder->pdata->vs_polarity << 3));
	/*
	 * Save the id data, required for power up sequence
	 */
	decoder->id = (struct i2c_device_id *)id;
	/* Attach to Master */
	strcpy(tvp514x_int_device.u.slave->attach_to, decoder->pdata->master);
	decoder->v4l2_int_device = &tvp514x_int_device;
	decoder->client = client;
	i2c_set_clientdata(client, decoder);

	/* add device attributes to access GPIOs */
	err = device_create_file(&client->dev, &dev_attr_gpio_c6);
	if (err < 0)
		dev_dbg(&client->dev, "cannot create gpio_c6 attr\n");
	err = device_create_file(&client->dev, &dev_attr_gpio_c7);
	if (err < 0)
		dev_dbg(&client->dev, "cannot create gpio_c7 attr\n");

	/* Register with V4L2 layer as slave device */
	err = v4l2_int_device_register(decoder->v4l2_int_device);
	if (err) {
		i2c_set_clientdata(client, NULL);
		v4l_err(client,
			"Unable to register to v4l2. Err[%d]\n", err);

	} else
		v4l_info(client, "Registered to v4l2 master %s!!\n",
				decoder->pdata->master);

	return 0;
}

/**
 * tvp514x_remove - decoder driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister decoder as an i2c client device and V4L2
 * device. Complement of tvp514x_probe().
 */
static int __exit tvp514x_remove(struct i2c_client *client)
{
	struct tvp514x_decoder *decoder = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(decoder->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	device_remove_file(&client->dev, &dev_attr_gpio_c6);
	device_remove_file(&client->dev, &dev_attr_gpio_c7);

	return 0;
}
/*
 * TVP5146 Init/Power on Sequence
 */
static const struct tvp514x_reg tvp5146_init_reg_seq[] = {
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x02},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0x80},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x01},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x60},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0xB0},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x01},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x00},
	{TOK_WRITE, REG_OPERATION_MODE, 0x01},
	{TOK_WRITE, REG_OPERATION_MODE, 0x00},
};
static const struct tvp514x_init_seq tvp5146_init = {
	.no_regs = ARRAY_SIZE(tvp5146_init_reg_seq),
	.init_reg_seq = tvp5146_init_reg_seq,
};
/*
 * TVP5147 Init/Power on Sequence
 */
static const struct tvp514x_reg tvp5147_init_reg_seq[] =	{
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x02},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0x80},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x01},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x60},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0xB0},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x01},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x16},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0xA0},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x16},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS1, 0x60},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS2, 0x00},
	{TOK_WRITE, REG_VBUS_ADDRESS_ACCESS3, 0xB0},
	{TOK_WRITE, REG_VBUS_DATA_ACCESS_NO_VBUS_ADDR_INCR, 0x00},
	{TOK_WRITE, REG_OPERATION_MODE, 0x01},
	{TOK_WRITE, REG_OPERATION_MODE, 0x00},
};
static const struct tvp514x_init_seq tvp5147_init = {
	.no_regs = ARRAY_SIZE(tvp5147_init_reg_seq),
	.init_reg_seq = tvp5147_init_reg_seq,
};
/*
 * TVP5146M2/TVP5147M1 Init/Power on Sequence
 */
static const struct tvp514x_reg tvp514xm_init_reg_seq[] = {
	{TOK_WRITE, REG_OPERATION_MODE, 0x01},
	{TOK_WRITE, REG_OPERATION_MODE, 0x00},
};
static const struct tvp514x_init_seq tvp514xm_init = {
	.no_regs = ARRAY_SIZE(tvp514xm_init_reg_seq),
	.init_reg_seq = tvp514xm_init_reg_seq,
};
/*
 * I2C Device Table -
 *
 * name - Name of the actual device/chip.
 * driver_data - Driver data
 */
static const struct i2c_device_id tvp514x_id[] = {
	{"tvp5146", (unsigned long)&tvp5146_init},
	{"tvp5146m2", (unsigned long)&tvp514xm_init},
	{"tvp5147", (unsigned long)&tvp5147_init},
	{"tvp5147m1", (unsigned long)&tvp514xm_init},
	{},
};

MODULE_DEVICE_TABLE(i2c, tvp514x_id);

static struct i2c_driver tvp514x_i2c_driver = {
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = tvp514x_probe,
	.remove = __exit_p(tvp514x_remove),
	.id_table = tvp514x_id,
};

/**
 * tvp514x_init
 *
 * Module init function
 */
static int __init tvp514x_init(void)
{
	return i2c_add_driver(&tvp514x_i2c_driver);
}

/**
 * tvp514x_cleanup
 *
 * Module exit function
 */
static void __exit tvp514x_cleanup(void)
{
	i2c_del_driver(&tvp514x_i2c_driver);
}

module_init(tvp514x_init);
module_exit(tvp514x_cleanup);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("TVP514X linux decoder driver");
MODULE_LICENSE("GPL");
