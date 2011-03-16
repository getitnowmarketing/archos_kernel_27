/*
 * drivers/media/video/tvp514x.h
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

#ifndef _TVP514X_H
#define _TVP514X_H

/*
 * Other macros
 */
#define TVP514X_MODULE_NAME		"tvp514x"

#define TVP514X_XCLK_BT656		(27000000)

/* Number of pixels and number of lines per frame for different standards */
#define NTSC_NUM_ACTIVE_PIXELS		(720)
#define NTSC_NUM_ACTIVE_LINES		(480)
#define PAL_NUM_ACTIVE_PIXELS		(720)
#define PAL_NUM_ACTIVE_LINES		(576)

/**
 * enum tvp514x_std - enum for supported standards
 */
enum tvp514x_std {
	STD_NTSC_MJ = 0,
	STD_PAL_BDGHIN,
	STD_PAL_M,
	STD_PAL_Nc,
	STD_NTSC_443,
	STD_SECAM,
	STD_PAL_60,
	STD_INVALID
};

/**
 * enum tvp514x_state - enum for different decoder states
 */
enum tvp514x_state {
	STATE_NOT_DETECTED,
	STATE_DETECTED
};

/**
 * enum tvp514x_input - enum for different decoder input pin
 *		configuration.
 */
enum tvp514x_input {
	/*
	 * CVBS input selection
	 */
	INPUT_CVBS_VI1A = 0x0,
	INPUT_CVBS_VI1B,
	INPUT_CVBS_VI1C,
	INPUT_CVBS_VI2A = 0x04,
	INPUT_CVBS_VI2B,
	INPUT_CVBS_VI2C,
	INPUT_CVBS_VI3A = 0x08,
	INPUT_CVBS_VI3B,
	INPUT_CVBS_VI3C,
	INPUT_CVBS_VI4A = 0x0C,
	/*
	 * S-Video input selection
	 */
	INPUT_SVIDEO_VI2A_VI1A = 0x44,
	INPUT_SVIDEO_VI2B_VI1B,
	INPUT_SVIDEO_VI2C_VI1C,
	INPUT_SVIDEO_VI2A_VI3A = 0x54,
	INPUT_SVIDEO_VI2B_VI3B,
	INPUT_SVIDEO_VI2C_VI3C,
	INPUT_SVIDEO_VI4A_VI1A = 0x4C,
	INPUT_SVIDEO_VI4A_VI1B,
	INPUT_SVIDEO_VI4A_VI1C,
	INPUT_SVIDEO_VI4A_VI3A = 0x5C,
	INPUT_SVIDEO_VI4A_VI3B,
	INPUT_SVIDEO_VI4A_VI3C,

	/* Need to add entries for
	 * RGB, YPbPr and SCART.
	 */
	INPUT_RGB_VI1A_VI2A_VI3A = 0x84,
	INPUT_YPbPr_VI1A_VI2A_VI3A = 0x94,
	INPUT_SCART_VI1A_VI2A_VI3A_VI4A = 0xCC,

	INPUT_INVALID
};

/**
 * enum tvp514x_dvr_input - enum for different dvr inputs
 */
enum tvp514x_dvr_input {
	INPUT_ANALOG_CH0 = 0,
	INPUT_SVIDEO = 1,
	INPUT_SCART = 2,
	INPUT_YPbPr = 3,
	INPUT_RGB = 4,
	INPUT_ANALOG_CH1 = 5,
};

/**
 * enum tvp514x_output - enum for output format
 *			supported.
 *
 */
enum tvp514x_output {
	OUTPUT_10BIT_422_EMBEDDED_SYNC = 0,
	OUTPUT_20BIT_422_SEPERATE_SYNC,
	OUTPUT_10BIT_422_SEPERATE_SYNC = 3,
	OUTPUT_INVALID
};

/* IOCTL private command numbers. */
enum v4l2_int_priv_ioctl_num {
	/* vidioc_int_priv_start_num = 2000, */
	vidioc_int_priv_querystatus_num = 2001,	/* Macrovision support */
	vidioc_int_priv_g_sliced_vbi_cap_num,	/* VIDIOC_G_SLICED_VBI_CAP */
	vidioc_int_priv_g_vbi_data_num,		/* VIDIOC_INT_G_VBI_DATA */
};

/* Wrapper for private ioctls */
V4L2_INT_WRAPPER_1(priv_querystatus, u32, *);
V4L2_INT_WRAPPER_1(priv_g_sliced_vbi_cap, struct v4l2_sliced_vbi_cap, *);
V4L2_INT_WRAPPER_1(priv_g_vbi_data, struct v4l2_sliced_vbi_data, *);

/**
 * struct tvp514x_std_info - Structure to store standard informations
 * @width: Line width in pixels
 * @height:Number of active lines
 * @video_std: Value to write in REG_VIDEO_STD register
 * @standard: v4l2 standard structure information
 */
struct tvp514x_std_info {
	unsigned long width;
	unsigned long height;
	u8 video_std;
	struct v4l2_standard standard;
};

/**
 * struct tvp514x_ctrl_regs - Registers related to a controls
 * @cvbs_reg_address: Register offset for the CVBS/S-Video input
 * @cvbs_reg_msb_address: Register offset for the CVBS/S-Video input (MSB)
 * @component_reg_address: Register offset for the Component input
 * @component_reg_msb_address: Register offset for the Component input (MSB)
 * @rgb_reg_address: Register offset for the RGB input
 * @rgb_reg_msb_address: Register offset for the RGB input (MSB)
 */
struct tvp514x_ctrl_regs {
	u8 cvbs_reg_address;
	u8 cvbs_reg_msb_address;
	u8 component_reg_address;
	u8 component_reg_msb_address;
	u8 rgb_reg_address;
	u8 rgb_reg_msb_address;
};

/**
 * struct tvp514x_ctrl_info - Information regarding supported controls
 * @regs: Registers related to the control
 * @query_ctrl: v4l2 query control information
 */
struct tvp514x_ctrl_info {
	struct tvp514x_ctrl_regs regs;
	struct v4l2_queryctrl query_ctrl;
};

/**
 * struct tvp514x_input_info - Information regarding supported inputs
 * @input_sel: Input select register
 * @lock_mask: lock mask - depends on Svideo/CVBS
 * @input: v4l2 input information
 */
struct tvp514x_input_info {
	enum tvp514x_input input_sel;
	struct v4l2_input input;
};

/**
 * struct tvp514x_platform_data - Platform data values and access functions.
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @ifparm: Interface parameters access function.
 * @priv_data_set: Device private data (pointer) access function.
 * @clk_polarity: Clock polarity of the current interface.
 * @ hs_polarity: HSYNC Polarity configuration for current interface.
 * @ vs_polarity: VSYNC Polarity configuration for current interface.
 */
struct tvp514x_platform_data {
	char *master;
	int (*power_set) (enum v4l2_power on);
	int (*ifparm) (struct v4l2_ifparm *p);
	int (*priv_data_set) (void *);
	/* Interface control params */
	bool clk_polarity;
	bool hs_polarity;
	bool vs_polarity;
};


#endif				/* ifndef _TVP514X_H */
