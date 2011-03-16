/*
 * drivers/media/video/mt9p012.h
 *
 * Register definitions for the MT9P012 camera sensor.
 *
 * Author:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Martinez Leonides
 *
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef MT9P012_H
#define MT9P012_H


#define MT9P012_I2C_ADDR		0x36

/* The ID values we are looking for */
#define MT9P012_MOD_ID			0x2801
#define MT9P012_MFR_ID			0x0006

/* MT9P012 has 8/16/32 registers */
#define MT9P012_8BIT			1
#define MT9P012_16BIT			2
#define MT9P012_32BIT			4

/* terminating token for reg list */
#define MT9P012_TOK_TERM 		0xFF

/* delay token for reg list */
#define MT9P012_TOK_DELAY		100

/* Sensor specific GPIO signals */
#define MT9P012_RESET_GPIO  	98
#define MT9P012_STANDBY_GPIO	58

#define VAUX_2_8_V		0x09
#define VAUX_DEV_GRP_P1		0x20
#define VAUX_DEV_GRP_NONE	0x00

#define DEBUG_BASE		0x08000000
#define REG_SDP3430_FPGA_GPIO_2 (0x50)
#define FPGA_SPR_GPIO1_3v3	(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL	(0x1 << 6)

/* terminating list entry for reg */
#define MT9P012_REG_TERM		0xFF
/* terminating list entry for val */
#define MT9P012_VAL_TERM		0xFF

#define MT9P012_CLKRC			0x11

/* Used registers */
#define REG_MODEL_ID			0x0000
#define REG_REVISION_NUMBER		0x0002
#define REG_MANUFACTURER_ID		0x0003

#define REG_MODE_SELECT			0x0100
#define REG_IMAGE_ORIENTATION		0x0101
#define REG_SOFTWARE_RESET		0x0103
#define REG_GROUPED_PAR_HOLD		0x0104
#define REG_DATA_FORMAT                 0x0112

#define REG_FINE_INT_TIME		0x0200
#define REG_COARSE_INT_TIME		0x0202

#define REG_ANALOG_GAIN_GLOBAL		0x0204
#define REG_ANALOG_GAIN_GREENR		0x0206
#define REG_ANALOG_GAIN_RED		0x0208
#define REG_ANALOG_GAIN_BLUE		0x020A
#define REG_ANALOG_GAIN_GREENB		0x020C
#define REG_DIGITAL_GAIN_GREENR		0x020E
#define REG_DIGITAL_GAIN_RED		0x0210
#define REG_DIGITAL_GAIN_BLUE		0x0212
#define REG_DIGITAL_GAIN_GREENB		0x0214

#define REG_VT_PIX_CLK_DIV		0x0300
#define REG_VT_SYS_CLK_DIV		0x0302
#define REG_PRE_PLL_CLK_DIV		0x0304
#define REG_PLL_MULTIPLIER		0x0306
#define REG_OP_PIX_CLK_DIV		0x0308
#define REG_OP_SYS_CLK_DIV		0x030A

#define REG_FRAME_LEN_LINES		0x0340
#define REG_LINE_LEN_PCK		0x0342

#define REG_X_ADDR_START		0x0344
#define REG_Y_ADDR_START		0x0346
#define REG_X_ADDR_END			0x0348
#define REG_Y_ADDR_END			0x034A
#define REG_X_OUTPUT_SIZE		0x034C
#define REG_Y_OUTPUT_SIZE		0x034E
#define REG_X_ODD_INC			0x0382
#define REG_Y_ODD_INC			0x0386

#define REG_SCALING_MODE		0x0400
#define REG_SCALE_M			0x0404
#define REG_SCALE_N			0x0406

#define REG_ROW_SPEED			0x3016
#define REG_RESET_REGISTER		0x301A
#define REG_PIXEL_ORDER			0x3024
#define REG_READ_MODE			0x3040

#define REG_DATAPATH_STATUS		0x306A
#define REG_DATAPATH_SELECT		0x306E

#define REG_RESERVED_MFR_3064		0x3064
#define REG_TEST_PATTERN		0x3070


#define MT9P012_GAIN			0x00

/*
 * The nominal xclk input frequency of the MT9P012 is 12MHz, maximum
 * frequency is 64MHz, and minimum frequency is 2MHz.
 */
#define MT9P012_XCLK_MIN   2000000
#define MT9P012_XCLK_MAX   64000000
#define MT9P012_XCLK_NOM_1 12000000
#define MT9P012_XCLK_NOM_2 24000000

#define MT9P012_USE_XCLKA  	0
#define MT9P012_USE_XCLKB  	1


/* FPS Capabilities */
#define MT9P012_MIN_FPS		11
#define MT9P012_DEF_FPS		15
#define MT9P012_MAX_FPS		30

#define MT9P012_I2C_DELAY   	3
#define I2C_RETRY_COUNT		5

/* Still capture 5 MP */
#define IMAGE_WIDTH_MAX		2592
#define IMAGE_HEIGHT_MAX	1944
/* Still capture 3 MP and down to VGA, using ISP resizer */
#define IMAGE_WIDTH_MIN		2048
#define IMAGE_HEIGHT_MIN	1536


/* Video mode, for D1 NTSC, D1 PAL */
#define VIDEO_WIDTH_2X_BINN	1296
#define VIDEO_HEIGHT_2X_BINN	972

/* Sensor Video mode size for VGA, CIF, QVGA in 4x binning mode */
#define VIDEO_WIDTH_4X_BINN	648
#define VIDEO_HEIGHT_4X_BINN	486
/* To improve image quality in VGA */
#define CIF_PIXELS		(352 * 288)
#define QQVGA_PIXELS		(160 * 120)

/* Video mode, for QCIF, SQCIF */
#define VIDEO_WIDTH_4X_BINN_SCALED      216
#define VIDEO_HEIGHT_4X_BINN_SCALED     162

/* Default coarse integration times to get a good exposure */
#define COARSE_INT_TIME_216	         550
#define COARSE_INT_TIME_648	         550
#define COARSE_INT_TIME_216_30FPS	1350
#define COARSE_INT_TIME_648_30FPS	1350
#define COARSE_INT_TIME_1296		1000
#define COARSE_INT_TIME_3MP		1700
#define COARSE_INT_TIME_5MP		1700
#define COARSE_INT_TIME_INDEX	    	1
#define TST_PAT 			0x0

/* Analog gain values */
#define MIN_GAIN	0x08
#define MAX_GAIN	0x7F
#define DEF_GAIN	0x43
#define GAIN_STEP   	0x1

#define GAIN_INDEX	1

/* Exposure time values */
#define DEF_MIN_EXPOSURE	0x08
#define DEF_MAX_EXPOSURE	0x7F
#define DEF_EXPOSURE	    	0x43
#define EXPOSURE_STEP       	1

#define SENSOR_DETECTED		1
#define SENSOR_NOT_DETECTED	0

/**
 * struct mt9p012_reg - mt9p012 register format
 * @length: length of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for MT9P012 register initialization values
 */
struct mt9p012_reg {
	u16	length;
	u16 	reg;
	u32 	val;
};

enum image_size {
	BIN4XSCALE,
	BIN4X,
	BIN2X,
	THREE_MP,
	FIVE_MP
};

enum pixel_format {
	RAWBAYER10
};

#define NUM_IMAGE_SIZES		5
#define NUM_PIXEL_FORMATS	1
#define NUM_FPS			2	/* 2 ranges */
#define FPS_LOW_RANGE		0
#define FPS_HIGH_RANGE		1

/**
 * struct capture_size - image capture size information
 * @width: image width in pixels
 * @height: image height in pixels
 */
struct capture_size {
	unsigned long width;
	unsigned long height;
};

/**
 * struct mt9p012_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @default_regs: Default registers written after power-on or reset.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct mt9p012_platform_data {
	int (*power_set)(enum v4l2_power power);
	const struct mt9p012_reg *default_regs;
	int (*ifparm)(struct v4l2_ifparm *p);
	int (*priv_data_set)(void *);
};

/**
 * struct mt9p012_pll_settings - struct for storage of sensor pll values
 * @vt_pix_clk_div: vertical pixel clock divider
 * @vt_sys_clk_div: veritcal system clock divider
 * @pre_pll_div: pre pll divider
 * @fine_int_tm: fine resolution interval time
 * @frame_lines: number of lines in frame
 * @line_len: number of pixels in line
 * @min_pll: minimum pll multiplier
 * @max_pll: maximum pll multiplier
 */
struct mt9p012_pll_settings {
	u16	vt_pix_clk_div;
	u16	vt_sys_clk_div;
	u16	pre_pll_div;

	u16	fine_int_tm;
	u16	frame_lines;
	u16	line_len;

	u16	min_pll;
	u16	max_pll;
};

/*
 * Array of image sizes supported by MT9P012.  These must be ordered from
 * smallest image size to largest.
 */
const static struct capture_size mt9p012_sizes[] = {
	{  216, 162 },	/* 4X BINNING+SCALING */
	{ 640, 480 },	/* 2X BINNING */
	{ 1300, 496 },	/* 2X4 BINNING */
	{ 2048, 1536},	/* 3 MP */
	{ 2592, 1944},	/* 5 MP */
};

/* PLL settings for MT9P012 */
enum mt9p012_pll_type {
  PLL_5MP = 0,
  PLL_3MP,
  PLL_1296_15FPS,
  PLL_1296_30FPS,
  PLL_648_15FPS,
  PLL_648_30FPS,
  PLL_216_15FPS,
  PLL_216_30FPS,
  PLL_648_120FPS
};

/* Used registers */
#define I2C_REG_MODEL_ID		0x00, 0x00
#define I2C_REG_REVISION_NUMBER		0x00, 0x02
#define I2C_REG_MANUFACTURER_ID		0x00, 0x03

#define I2C_REG_MODE_SELECT		0x01, 0x00
#define I2C_REG_IMAGE_ORIENTATION	0x01, 0x01
#define I2C_REG_SOFTWARE_RESET		0x01, 0x03
#define I2C_REG_GROUPED_PAR_HOLD	0x01, 0x04
#define I2C_REG_DATA_FORMAT             0x01, 0x12

#define I2C_REG_FINE_INT_TIME		0x02, 0x00
#define I2C_REG_COARSE_INT_TIME		0x02, 0x02

#define I2C_REG_ANALOG_GAIN_GLOBAL	0x02, 0x04
#define I2C_REG_ANALOG_GAIN_GREENR	0x02, 0x06
#define I2C_REG_ANALOG_GAIN_RED		0x02, 0x08
#define I2C_REG_ANALOG_GAIN_BLUE	0x02, 0x0A
#define I2C_REG_ANALOG_GAIN_GREENB	0x02, 0x0C
#define I2C_REG_DIGITAL_GAIN_GREENR	0x02, 0x0E
#define I2C_REG_DIGITAL_GAIN_RED	0x02, 0x10
#define I2C_REG_DIGITAL_GAIN_BLUE	0x02, 0x12
#define I2C_REG_DIGITAL_GAIN_GREENB	0x02, 0x14

#define I2C_REG_VT_PIX_CLK_DIV		0x03, 0x00
#define I2C_REG_VT_SYS_CLK_DIV		0x03, 0x02
#define I2C_REG_PRE_PLL_CLK_DIV		0x03, 0x04
#define I2C_REG_PLL_MULTIPLIER		0x03, 0x06
#define I2C_REG_OP_PIX_CLK_DIV		0x03, 0x08
#define I2C_REG_OP_SYS_CLK_DIV		0x03, 0x0A

#define I2C_REG_FRAME_LEN_LINES		0x03, 0x40
#define I2C_REG_LINE_LEN_PCK		0x03, 0x42

#define I2C_REG_X_ADDR_START		0x03, 0x44
#define I2C_REG_Y_ADDR_START		0x03, 0x46
#define I2C_REG_X_ADDR_END		0x03, 0x48
#define I2C_REG_Y_ADDR_END		0x03, 0x4A
#define I2C_REG_X_OUTPUT_SIZE		0x03, 0x4C
#define I2C_REG_Y_OUTPUT_SIZE		0x03, 0x4E
#define I2C_REG_X_ODD_INC		0x03, 0x82
#define I2C_REG_Y_ODD_INC		0x03, 0x86

#define I2C_REG_SCALING_MODE		0x04, 0x00
#define I2C_REG_SCALE_M			0x04, 0x04
#define I2C_REG_SCALE_N			0x04, 0x06

#define I2C_REG_ROW_SPEED		0x30, 0x16
#define I2C_REG_RESET_REGISTER		0x30, 0x1A
#define I2C_REG_PIXEL_ORDER		0x30, 0x24
#define I2C_REG_READ_MODE		0x30, 0x40

#define I2C_REG_DATAPATH_STATUS		0x30, 0x6A
#define I2C_REG_DATAPATH_SELECT		0x30, 0x6E

#define I2C_REG_RESERVED_MFR_3064	0x30, 0x64
#define I2C_REG_TEST_PATTERN		0x30, 0x70
#define I2C_VIDEO_WIDTH_4X_BINN_SCALED  0x00, 0xd8
#define I2C_VIDEO_HEIGHT_4X_BINN_SCALED 0x00, 0xa2

#define I2C_COARSE_INT_TIME_216	        0x02, 0x26
#define I2C_VIDEO_WIDTH_2X_BINN		0x05, 0x10
#define I2C_VIDEO_HEIGHT_2X_BINN	0x03, 0xcc

#define I2C_COARSE_INT_TIME_5MP		0x06, 0xa4
#define I2C_COARSE_INT_TIME_648	        0x02, 0x26
#define I2C_COARSE_INT_TIME_216_30FPS	0x05, 0x46
#define I2C_COARSE_INT_TIME_648_30FPS	0x05, 0x46
#define I2C_COARSE_INT_TIME_1296	0x03, 0xff
#define I2C_COARSE_INT_TIME_3MP		0x06, 0xa4
#define I2C_MIN_GAIN			0x00, 0x08
#define I2C_MT9P012_8BIT	3
#define I2C_MT9P012_16BIT	4
#define I2C_MT9P012_32BIT	6

#endif /* ifndef MT9P012_H */
