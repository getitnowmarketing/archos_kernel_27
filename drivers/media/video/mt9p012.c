/*
 * drivers/media/video/mt9p012.c
 *
 * mt9p012 sensor driver
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Contributors:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Martinez Leonides
 *
 * Leverage OV9640.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>

#include "mt9p012.h"
#include "isp/isp.h"

#define DRIVER_NAME  "mt9p012"
#define MOD_NAME "MT9P012: "

#define I2C_IMAGE_WIDTH_MAX		0x0a, 0x20
#define I2C_IMAGE_HEIGHT_MAX		0x07, 0x98
#define I2C_VIDEO_WIDTH_4X_BINN		0x02, 0x88
#define I2C_VIDEO_HEIGHT_4X_BINN 	0x01, 0xe6

#define I2C_ANALOG_GAIN_LIST_SIZE	3
#define I2C_SET_EXPOSURE_TIME_LIST_SIZE	3
#define I2C_INITIAL_LIST_SIZE	32
 
#define I2C_ENTER_VIDEO_648_15FPS_LIST_SIZE	24
#define I2C_STREAM_ON_LIST_SIZE 1
#define I2C_STREAM_OFF_LIST_SIZE 1
#define I2C_SET_FPS_LIST_SIZE 1


#define I2C_ENTER_VIDEO_1296_15FPS_LIST_SIZE 22
#define I2C_ENTER_IMAGE_MODE_5MP_10FPS_LIST_SIZE 22
 
#define I2C_ENTER_VIDEO_1296_30FPS_LIST_SIZE 22

unsigned char initial_list_buf[][6] = {

   {I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x01, 0x00, 0x00}, /* hold */
	//analog gains 
	{0x02, 0x04, 0x00, 0x40, 0x00, 0x00}, // global
	{0x02, 0x06, 0x00, 0x40, 0x00, 0x00},
	{0x02, 0x08, 0x00, 0x40, 0x00, 0x00},	
	{0x02, 0x0a, 0x00, 0x40, 0x00, 0x00},		
	{0x02, 0x0c, 0x00, 0x40, 0x00, 0x00},		
	//digital gains	
	{0x02, 0x0e, 0x01, 0x00, 0x00, 0x00},		
	{0x02, 0x10, 0x01, 0x00, 0x00, 0x00},			
	{0x02, 0x12, 0x01, 0x00, 0x00, 0x00},		
	{0x02, 0x14, 0x01, 0x00, 0x00, 0x00},
    
    //MI-5130 rev7 update 5-8_A.pdf - 16.5.08
    // specific for rev7
   {0x03, 0x00, 0x00, 0x05, 0x00, 0x00},
   {0x03, 0x02, 0x00, 0x01, 0x00, 0x00}, /* hold */
   {0x03, 0x04, 0x00, 0x03, 0x00, 0x00},
   {0x03, 0x06, 0x00, 0x8C, 0x00, 0x00},
   {0x03, 0x08, 0x00, 0x08, 0x00, 0x00},
   {0x03, 0x0A, 0x01, 0x20, 0x00, 0x00},
   {0x30, 0x1A, 0x10, 0xC8, 0x00, 0x00},
   {0x30, 0x64, 0x08, 0x05, 0x00, 0x00},
   {0x30, 0x88, 0x6E, 0x61, 0x00, 0x00},
   {0x30, 0x8E, 0xE0, 0x60, 0x00, 0x00},
   {0x30, 0x84, 0x24, 0x24, 0x00, 0x00},
   {0x30, 0x92, 0x0A, 0x52, 0x00, 0x00},
   {0x30, 0x94, 0x46, 0x56, 0x00, 0x00},
   {0x30, 0x96, 0x56, 0x52, 0x00, 0x00},
   {0x30, 0xCA, 0x80, 0x06, 0x00, 0x00},
   {0x31, 0x2A, 0xDD, 0x02, 0x00, 0x00},
   {0x31, 0x2C, 0x00, 0xE4, 0x00, 0x00},
   {0x31, 0x70, 0x29, 0x9A, 0x00, 0x00},
   {0x30, 0xB0, 0x00, 0x01, 0x00, 0x00}, 
   {I2C_REG_ROW_SPEED,   0x01, 0x01, 0x00, 0x00}, 
   // we're setting  DATAPATH_SELECT_SLEW_RATE_CTRL_PARALLEL bit
   // and DATAPATH_SELECT_SLEW_RATE_CTRL_PIXCLK bit to 1 
   // This can reduce ringing and electromagnetic emissions.
   // And also makes the sensor more stable to more light
   // if the defualt value is used for this register the rev7 won't work
   {0x30, 0x6e, 0x24, 0x80, 0x00, 0x00}, 
	
   {I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x00, 0x00, 0x00},
};

struct i2c_msg initial_list[] = {
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[0][0]},
	 /* hold */
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_8BIT, &initial_list_buf[1][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[2][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[3][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[4][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[5][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[6][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[7][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[8][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[9][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[10][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[11][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[12][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[13][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[14][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[15][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[16][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[17][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[18][0]},	
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[19][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[20][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[21][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[22][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[23][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[24][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[25][0]},	
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[26][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[27][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[28][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[29][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_16BIT, &initial_list_buf[29][0]},
	{MT9P012_I2C_ADDR, 0x0, I2C_MT9P012_8BIT, &initial_list_buf[30][0]},
};
 

unsigned char enter_video_1296_15fps_buf[][6] = {
	/* hold */

	{I2C_REG_VT_PIX_CLK_DIV, 0x00, 0x05, 0x00, 0x00},
	{I2C_REG_VT_SYS_CLK_DIV, 0x00, 0x01, 0x00, 0x00},
	{I2C_REG_PRE_PLL_CLK_DIV, 0x00, 0x03, 0x00, 0x00},
	{I2C_REG_PLL_MULTIPLIER, 0x00, 0x82, 0x00, 0x00},
	{I2C_REG_OP_PIX_CLK_DIV, 0x00, 0x0a, 0x00, 0x00},
	{I2C_REG_OP_SYS_CLK_DIV, 0x00, 0x01, 0x00, 0x00},

	{I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x01, 0x00, 0x00},

	{I2C_REG_RESERVED_MFR_3064, 0x08, 0x05, 0x00, 0x00},
	{I2C_REG_RESERVED_MFR_3064, 0x08, 0x05, 0x00, 0x00},

	{I2C_REG_X_OUTPUT_SIZE, 0x05, 0x14, 0x00, 0x00},// 1300
	{I2C_REG_Y_OUTPUT_SIZE, 0x01, 0xF0, 0x00, 0x00},// 496

	{I2C_REG_X_ADDR_START, 0x00, 0x04, 0x00, 0x00},
	{I2C_REG_Y_ADDR_START, 0x00, 0x00, 0x00, 0x00},

	{I2C_REG_X_ADDR_END, 0x0a, 0x29, 0x00, 0x00}, // 2601
	{I2C_REG_Y_ADDR_END, 0x07, 0xa1, 0x00, 0x00}, // 1953
		
	{I2C_REG_FINE_INT_TIME, 0x07, 0x02, 0x00, 0x00},
	{I2C_REG_FRAME_LEN_LINES, 0x06, 0x04, 0x00, 0x00}, //lpfr
	{I2C_REG_LINE_LEN_PCK, 0x11, 0x96, 0x00, 0x00}, //ppln
	{I2C_REG_COARSE_INT_TIME, 0x03, 0x00, 0x00, 0x00},

	{I2C_REG_READ_MODE, 0x14, 0xc7, 0x00, 0x00}, // 2x4 binning

	/* update */
	{I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x00, 0x00, 0x00},
};
struct i2c_msg enter_video_1296_15fps[] = {
	/* hold */
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
					&enter_video_1296_15fps_buf[0][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[1][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[2][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[3][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[4][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[5][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[6][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[7][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[8][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[9][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[10][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[11][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[12][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[13][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[14][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[15][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[16][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[17][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[18][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[19][0]},
        {MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_15fps_buf[20][0]},

	/* update */
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
					&enter_video_1296_15fps_buf[21][0]},
};
unsigned char enter_image_mode_5MP_10fps_buf[][6] = {
	/* hold */
	{I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x01, 0x00, 0x00},
	{I2C_REG_VT_PIX_CLK_DIV, 0x00, 0x06, 0x00, 0x00},
	{I2C_REG_VT_SYS_CLK_DIV, 0x00, 0x01, 0x00, 0x00},
	{I2C_REG_PRE_PLL_CLK_DIV, 0x00, 0x05, 0x00, 0x00},
	/* 0x000a fps */
	{I2C_REG_PLL_MULTIPLIER, 0x00, 0xb8, 0x00, 0x00},
	{I2C_REG_OP_PIX_CLK_DIV, 0x00, 0x0A, 0x00, 0x00},
	{I2C_REG_OP_SYS_CLK_DIV, 0x00, 0x01, 0x00, 0x00},
	{I2C_REG_RESERVED_MFR_3064, 0x08, 0x05, 0x00, 0x00},
	{I2C_REG_X_OUTPUT_SIZE, I2C_IMAGE_WIDTH_MAX},
	{I2C_REG_Y_OUTPUT_SIZE, I2C_IMAGE_HEIGHT_MAX},
	{I2C_REG_X_ADDR_START, 0x00, 0x08, 0x00, 0x00},
	{I2C_REG_Y_ADDR_START, 0x00, 0x08, 0x00, 0x00},
	{I2C_REG_X_ADDR_END, 0x0a, 0x27, 0x00, 0x00},
	{I2C_REG_Y_ADDR_END, 0x07, 0x9f, 0x00, 0x00},
	//read mode without binning (x_odd = 1 and y_odd = 1)
	{I2C_REG_READ_MODE, 0x00, 0x41, 0x00, 0x00},
	{I2C_REG_FINE_INT_TIME, 0x03, 0x72, 0x00, 0x00},
	{I2C_REG_FRAME_LEN_LINES, 0x08, 0x08, 0x00, 0x00},
	{I2C_REG_LINE_LEN_PCK, 0x14, 0xfc, 0x00, 0x00},
	{I2C_REG_SCALE_M, 0x00, 0x00, 0x00, 0x00},
	/*    {MT9P012_16BIT, 0x0600, 2}, */
	/* disable scaler */
	{I2C_REG_SCALING_MODE, 0x00, 0x00, 0x00, 0x00},
	{I2C_REG_COARSE_INT_TIME, I2C_COARSE_INT_TIME_5MP},
        {I2C_REG_DATA_FORMAT, 0x0A, 0x0A, 0x00, 0x00},
        {I2C_REG_X_ODD_INC, 0x00, 0x01, 0x00, 0x00},
        {I2C_REG_Y_ODD_INC, 0x00, 0x01, 0x00, 0x00},
	/* update */
	{I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x00, 0x00, 0x00},
};


struct i2c_msg enter_image_mode_5MP_10fps[] = {
	/* hold */
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
					&enter_image_mode_5MP_10fps_buf[0][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[1][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[2][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[3][0]},
	/* 10 fps */
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[4][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[5][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[6][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[7][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[8][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[9][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[10][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[11][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[12][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[13][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[14][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[15][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[16][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[17][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[18][0]},
	/*    {MT9P012_16BIT, 0x0600, 2}, */
	/* disable scaler */
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[19][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[20][0]},
        {MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[21][0]},
        {MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[22][0]},
        {MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_image_mode_5MP_10fps_buf[23][0]},
	/* update */
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
					&enter_image_mode_5MP_10fps_buf[24][0]},
};
  
unsigned char enter_video_1296_30fps_buf[][6] = {
	/* hold */

	{I2C_REG_VT_PIX_CLK_DIV, 0x00, 0x05, 0x00, 0x00},
	{I2C_REG_VT_SYS_CLK_DIV, 0x00, 0x01, 0x00, 0x00},
	{I2C_REG_PRE_PLL_CLK_DIV, 0x00, 0x03, 0x00, 0x00},
	{I2C_REG_PLL_MULTIPLIER, 0x00, 0x82, 0x00, 0x00},
	{I2C_REG_OP_PIX_CLK_DIV, 0x00, 0x0a, 0x00, 0x00},
	{I2C_REG_OP_SYS_CLK_DIV, 0x00, 0x01, 0x00, 0x00},

	{I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x01, 0x00, 0x00},

	{I2C_REG_RESERVED_MFR_3064, 0x08, 0x05, 0x00, 0x00},
	{I2C_REG_RESERVED_MFR_3064, 0x08, 0x05, 0x00, 0x00},
	
	{I2C_REG_X_OUTPUT_SIZE, 0x05, 0x14, 0x00, 0x00},// 1300
	{I2C_REG_Y_OUTPUT_SIZE, 0x01, 0xF0, 0x00, 0x00},// 496

	{I2C_REG_X_ADDR_START, 0x00, 0x04, 0x00, 0x00},
	{I2C_REG_Y_ADDR_START, 0x00, 0x00, 0x00, 0x00},

	{I2C_REG_X_ADDR_END, 0x0a, 0x29, 0x00, 0x00}, // 2601
	{I2C_REG_Y_ADDR_END, 0x07, 0xa1, 0x00, 0x00}, // 1953
		
	{I2C_REG_FINE_INT_TIME, 0x07, 0x02, 0x00, 0x00},
	{I2C_REG_FRAME_LEN_LINES, 0x03, 0x02, 0x00, 0x00}, //lpfr
	{I2C_REG_LINE_LEN_PCK, 0x11, 0x96, 0x00, 0x00}, //ppln
	{I2C_REG_COARSE_INT_TIME, 0x03, 0x00, 0x00, 0x00},

	{I2C_REG_READ_MODE, 0x14, 0xc7, 0x00, 0x00}, // 2x4 binning

	/* update */
	{I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x00, 0x00, 0x00},
};


struct i2c_msg enter_video_1296_30fps[] = {
	/* hold */
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
					&enter_video_1296_30fps_buf[0][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[1][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[2][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[3][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[4][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[5][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[6][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[7][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[8][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[9][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[10][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[11][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[12][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[13][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[14][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[15][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[16][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[17][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[18][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[19][0]},
        {MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&enter_video_1296_30fps_buf[20][0]},

	/* update */
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
					&enter_video_1296_30fps_buf[21][0]},
};

unsigned char stream_off_list_buf[][6] = {
	{I2C_REG_MODE_SELECT, 0x00, 0x00, 0x00, 0x00},
};

struct i2c_msg stream_off_list[] = {
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT, &stream_off_list_buf[0][0]},
};

/* Structure to set analog gain */
unsigned char set_analog_gain_buf[][6] = {
	{I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x01, 0x00, 0x00},
	{I2C_REG_ANALOG_GAIN_GLOBAL, I2C_MIN_GAIN},
	/* updating */
	{I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x00, 0x00, 0x00},
};

/* Structure to set analog gain */
struct i2c_msg set_analog_gain[] = {
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
					&set_analog_gain_buf[0][0]},
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
					&set_analog_gain_buf[1][0]},
	/* updating */
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
					&set_analog_gain_buf[2][0]},
};

unsigned char set_fps_buf[][6] = {
	{I2C_REG_PLL_MULTIPLIER, 0x00, 0x00, 0x00, 0x00},
};

/* Exits soft standby */
struct i2c_msg set_fps[] = {
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT, &set_fps_buf[0][0]},
};

unsigned char initial_list_delay_buf[][6] = {
	{I2C_REG_SOFTWARE_RESET, 0x00, 0x01, 0x00, 0x00},
};

struct i2c_msg initial_list_delay[] = {
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
				&initial_list_delay_buf[0][0]},
};

/* Exits soft standby */
unsigned char stream_on_list_buf[][6] = {
	{I2C_REG_MODE_SELECT, 0x01, 0x00, 0x00, 0x00},
};

/* Exits soft standby */
struct i2c_msg stream_on_list[] = {
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
						&stream_on_list_buf[0][0]},
};

/* Structure which will set the exposure time */
unsigned char set_exposure_time_buf[][6] = {
	{I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x01, 0x00, 0x00},
	/* less than frame_lines-0x0001 */
	{I2C_REG_COARSE_INT_TIME, 0x01, 0xf4, 0x00, 0x00},
	/* updating */
	{I2C_REG_GROUPED_PAR_HOLD, 0x00, 0x00, 0x00, 0x00},
};

/* Structure which will set the exposure time */
struct i2c_msg set_exposure_time[] = {
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
					&set_exposure_time_buf[0][0]},
	/* less than frame_lines-1 */
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_16BIT,
						&set_exposure_time_buf[1][0]},
	/* updating */
	{MT9P012_I2C_ADDR, 0x00, I2C_MT9P012_8BIT,
						&set_exposure_time_buf[2][0]},
};

static int
mt9p012_write_regs(struct i2c_client *client, struct i2c_msg *msg, int num)
{
	int err;
	int retry = 0;
again:
	err = i2c_transfer(client->adapter, msg, num);
	if (err >= 0)
		return 0;

	if (retry <= I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retry ... %d\n", retry);
		retry++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto again;
	}
	return err;
}

/**
 * struct mt9p012_sensor - main structure for storage of sensor information
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @i2c_client: iic client device structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @scaler:
 * @ver: mt9p012 chip version
 * @fps: frames per second value
 */
struct mt9p012_sensor {
	const struct mt9p012_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	int scaler;
	int ver;
	int fps;
	int state;
};

static struct mt9p012_sensor mt9p012;
static struct i2c_driver mt9p012sensor_i2c_driver;
static unsigned long xclk_current = MT9P012_XCLK_NOM_1;

/* list of image formats supported by mt9p012 sensor */
const static struct v4l2_fmtdesc mt9p012_formats[] = {
	{
		.description    = "Bayer10 (GrR/BGb)",
		.pixelformat    = V4L2_PIX_FMT_SGRBG10,
	},
	{
		.description	= "Bayer10 pattern (GrR/BGb)",
		.pixelformat	= V4L2_PIX_FMT_PATT,
	}
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(mt9p012_formats)

static u32 min_exposure_time;
static u32 max_exposure_time;
static u32 pix_clk_freq;
static int update_exp_time, update_gain, size_updated, fps_updated;
enum v4l2_power current_power_state;

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
static struct mt9p012_pll_settings all_pll_settings[] = {
	/* PLL_5MP */
	{.vt_pix_clk_div = 4, .vt_sys_clk_div = 1, .pre_pll_div = 5,
	.fine_int_tm = 882, .frame_lines = 2056, .line_len = 5372,
	.min_pll = 160, .max_pll = 200},
	/* PLL_3MP */
	{.vt_pix_clk_div = 4, .vt_sys_clk_div = 1, .pre_pll_div = 5,
	.fine_int_tm = 882, .frame_lines = 2056, .line_len = 5372,
	.min_pll = 160, .max_pll = 200},
	/* PLL_1296_15FPS */
	{.vt_pix_clk_div = 5, .vt_sys_clk_div = 2, .pre_pll_div = 3,
	.fine_int_tm = 1794, .frame_lines = 1061, .line_len = 3360,
	.min_pll = 96, .max_pll = 190},
	/* PLL_1296_30FPS */
	{.vt_pix_clk_div = 5, .vt_sys_clk_div = 1, .pre_pll_div = 3,
	.fine_int_tm = 1794, .frame_lines = 1061, .line_len = 3360,
	.min_pll = 96, .max_pll = 150},
	/* PLL_648_15FPS */
	{.vt_pix_clk_div = 8, .vt_sys_clk_div = 2, .pre_pll_div = 2,
	.fine_int_tm = 1794, .frame_lines = 574, .line_len = 2712,
	.min_pll = 92, .max_pll = 128},
	/* PLL_648_30FPS */
	{.vt_pix_clk_div = 5, .vt_sys_clk_div = 1, .pre_pll_div = 6,
	.fine_int_tm = 1794, .frame_lines = 1374, .line_len = 3712,
	.min_pll = 96, .max_pll = 192},
	/* PLL_216_15FPS */
	{.vt_pix_clk_div = 8, .vt_sys_clk_div = 2, .pre_pll_div = 2,
	.fine_int_tm = 1794,  .frame_lines = 574, .line_len = 2712,
	.min_pll = 92, .max_pll = 126},
	/* PLL_216_30FPS */
	{.vt_pix_clk_div = 5, .vt_sys_clk_div = 2, .pre_pll_div = 3,
	.fine_int_tm = 1794,  .frame_lines = 1374, .line_len = 3712,
	.min_pll = 96, .max_pll = 192},
	/* PLL_648_120FPS */
	{.vt_pix_clk_div = 6, .vt_sys_clk_div = 1, .pre_pll_div = 2,
	.fine_int_tm = 1016, .frame_lines = 571, .line_len = 1828,
	.min_pll = 96, .max_pll = 192}
};

static enum mt9p012_pll_type current_pll_video;

struct i2c_list {
	struct i2c_msg *reg_list;
	unsigned int list_size;
};

struct i2c_list mt9p012_reg_init[NUM_FPS][NUM_IMAGE_SIZES] = {
	{
		{enter_video_1296_15fps, I2C_ENTER_VIDEO_1296_15FPS_LIST_SIZE},
		{enter_video_1296_15fps, I2C_ENTER_VIDEO_1296_15FPS_LIST_SIZE},
		{enter_video_1296_15fps, I2C_ENTER_VIDEO_1296_15FPS_LIST_SIZE},
		{enter_image_mode_5MP_10fps,
				I2C_ENTER_IMAGE_MODE_5MP_10FPS_LIST_SIZE},
		{enter_image_mode_5MP_10fps,
				I2C_ENTER_IMAGE_MODE_5MP_10FPS_LIST_SIZE},
	},
	{
		{enter_video_1296_30fps, I2C_ENTER_VIDEO_1296_30FPS_LIST_SIZE},
		{enter_video_1296_30fps, I2C_ENTER_VIDEO_1296_30FPS_LIST_SIZE},
		{enter_video_1296_30fps, I2C_ENTER_VIDEO_1296_30FPS_LIST_SIZE},
		{enter_image_mode_5MP_10fps,
				I2C_ENTER_IMAGE_MODE_5MP_10FPS_LIST_SIZE},
		{enter_image_mode_5MP_10fps,
				I2C_ENTER_IMAGE_MODE_5MP_10FPS_LIST_SIZE},
	},
};

/**
 * struct vcontrol - Video controls
 * @v4l2_queryctrl: V4L2 VIDIOC_QUERYCTRL ioctl structure
 * @current_value: current value of this control
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = DEF_MIN_EXPOSURE,
			.maximum = DEF_MAX_EXPOSURE,
			.step = EXPOSURE_STEP,
			.default_value = DEF_EXPOSURE,
		},
		.current_value = DEF_EXPOSURE,
	},
	{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Analog Gain",
			.minimum = MIN_GAIN,
			.maximum = MAX_GAIN,
			.step = GAIN_STEP,
			.default_value = DEF_GAIN,
		},
		.current_value = DEF_GAIN,
	}
};

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int
find_vctrl(int id)
{
	int i;
	

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	
	return i;
}

/**
 * mt9p012_read_reg - Read a value from a register in an mt9p012 sensor device
 * @client: i2c driver client structure
 * @data_length: length of data to be read
 * @reg: register address / offset
 * @val: stores the value that gets read
 *
 * Read a value from a register in an mt9p012 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
int
mt9p012_read_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 *val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;
	if (data_length != MT9P012_8BIT && data_length != MT9P012_16BIT
					&& data_length != MT9P012_32BIT)
		return -EINVAL;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);;
	data[1] = (u8) (reg & 0xff);
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {
		msg->len = data_length;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = 0;
		/* high byte comes first */
		if (data_length == MT9P012_8BIT)
			*val = data[0];
		else if (data_length == MT9P012_16BIT)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
				(data[1] << 16) + (data[0] << 24);
		return 0;
	}
	dev_err(&client->dev, "read from offset 0x%x error %d\n", reg, err);
	
	return err;
}
/**
 * mt9p012sensor_set_exposure_time - sets exposure time per input value
 * @exp_time: exposure time to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 *
 * If the requested exposure time is within the allowed limits, the HW
 * is configured to use the new exposure time, and the
 * video_control[] array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int mt9p012sensor_set_exposure_time(u32 exp_time, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
	int err = 0, scaling = 1;
    int i;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	u32 coarse_int_time = 0;
	

	if (current_power_state == V4L2_POWER_ON) {
		if ((exp_time < min_exposure_time) ||
				(exp_time > max_exposure_time)) {
			dev_err(&client->dev, "Exposure time not within the "
							"legal range.\n");
			dev_err(&client->dev, "Exposure time must be"
								" between\n");
			dev_err(&client->dev, "%d us and %d us\n",
					min_exposure_time, max_exposure_time);
			return -EINVAL;
		}

		/* Scaling adjustment */
		if (sensor->scaler && (sensor->fps < 16))
			scaling = 2;

		coarse_int_time = ((((exp_time / 10)
			* ((pix_clk_freq / scaling) / 1000)) / 1000)
			- (all_pll_settings[current_pll_video].fine_int_tm
			/ 10)) / (all_pll_settings[current_pll_video].line_len
			/ 10);

		dev_dbg(&client->dev, "coarse_int_time calculated = %d\n",
							coarse_int_time);

		set_exposure_time_buf[COARSE_INT_TIME_INDEX][2] =
							coarse_int_time >> 8;
		set_exposure_time_buf[COARSE_INT_TIME_INDEX][3] =
							coarse_int_time & 0xff;
		err = mt9p012_write_regs(client, set_exposure_time,
					I2C_SET_EXPOSURE_TIME_LIST_SIZE);
	} else
		update_exp_time = 1;

	if (err)
		dev_err(&client->dev, "Error setting exposure time...%d\n",
									err);
	else {
		i = find_vctrl(V4L2_CID_EXPOSURE);
		if (i >= 0) {
			lvc = &video_control[i];
		lvc->current_value = exp_time;
		}
	}
	

	return err;
}

/**
 * mt9p012sensor_set_gain - sets sensor analog gain per input value
 * @gain: analog gain value to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 analog gain entry in video_control array
 *
 * If the requested analog gain is within the allowed limits, the HW
 * is configured to use the new gain value, and the video_control
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int mt9p012sensor_set_gain(u16 gain, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
	int err = 0;
    int i;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	

	if (current_power_state == V4L2_POWER_ON) {
		if ((gain < MIN_GAIN) || (gain > MAX_GAIN)) {
			dev_err(&client->dev, "Gain not within the legal"
								" range\n");
			return -EINVAL;
		}

		set_analog_gain_buf[GAIN_INDEX][2] = gain >> 8;
		set_analog_gain_buf[GAIN_INDEX][3] = gain & 0xff;
		err = mt9p012_write_regs(client, set_analog_gain,
						I2C_ANALOG_GAIN_LIST_SIZE);
	} else
		update_gain = 1;
	if (err) {
		dev_err(&client->dev, "Error while setting analog gain: %d\n",
									err);
		return err;
	} else {
		i = find_vctrl(V4L2_CID_GAIN);
		if (i >= 0) {
			lvc = &video_control[i];
		lvc->current_value = gain;
		}
	}
	

	return err;
}

/**
 * mt9p012_calc_size - Find the best match for a requested image capture size
 * @width: requested image width in pixels
 * @height: requested image height in pixels
 *
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum image_size mt9p012_calc_size(unsigned int width,
							unsigned int height)
{
	enum image_size isize;
	unsigned long pixels = width * height;
	
	for (isize = BIN4XSCALE; isize <= FIVE_MP; isize++) {
		if (mt9p012_sizes[isize].height *
					mt9p012_sizes[isize].width >= pixels) {
			/* To improve image quality in VGA */
			if ((pixels > CIF_PIXELS) && (isize == BIN4X)) {
				isize = BIN2X;
			} else if ((pixels > QQVGA_PIXELS) &&
							(isize == BIN4XSCALE)) {
				isize = BIN4X;
			}
			return isize;
		}
	}
	

	return FIVE_MP;
}

/**
 * mt9p012_find_isize - Find the best match for a requested image capture size
 * @width: requested image width in pixels
 * @height: requested image height in pixels
 *
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum image_size mt9p012_find_isize(unsigned int width)
{
	enum image_size isize;
	

	for (isize = BIN4XSCALE; isize <= FIVE_MP; isize++) {
		if (mt9p012_sizes[isize].width >= width)
			break;
	}
	

	return isize;
}
/**
 * mt9p012_find_fps_index - Find the best fps range match for a
 *  requested frame rate
 * @fps: desired frame rate
 * @isize: enum value corresponding to image size
 *
 * Find the best match for a requested frame rate.  The best match
 * is chosen between two fps ranges (11 - 15 and 16 - 30 fps) depending on
 * the image size. For image sizes larger than BIN2X, frame rate is fixed
 * at 10 fps.
 */
static unsigned int mt9p012_find_fps_index(unsigned int fps,
							enum image_size isize)
{
	unsigned int index = FPS_LOW_RANGE;
	

	if (isize > BIN4X) {
		if (fps > 21)
			index = FPS_HIGH_RANGE;
	} else {
		if (fps > 15)
			index = FPS_HIGH_RANGE;
	}
	

	return index;
}

/**
 * mt9p012sensor_calc_xclk - Calculate the required xclk frequency
 * @c: i2c client driver structure
 *
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate and return the required xclk frequency
 */
static unsigned long mt9p012sensor_calc_xclk(struct i2c_client *c)
{
	struct mt9p012_sensor *sensor = i2c_get_clientdata(c);
	struct v4l2_fract *timeperframe = &sensor->timeperframe;
	struct v4l2_pix_format *pix = &sensor->pix;
	int qvga_120 = 0;
	

	if ((timeperframe->numerator == 0)
	|| (timeperframe->denominator == 0)) {
		/* supply a default nominal_timeperframe */
		timeperframe->numerator = 1;
		timeperframe->denominator = MT9P012_DEF_FPS;
	}

	sensor->fps = timeperframe->denominator/timeperframe->numerator;
	if ((sensor->fps == 120) && (pix->width <= VIDEO_WIDTH_4X_BINN) &&
				(pix->width > VIDEO_WIDTH_4X_BINN_SCALED))
		qvga_120 = 1;
	if (sensor->fps < MT9P012_MIN_FPS)
		sensor->fps = MT9P012_MIN_FPS;
	else if ((sensor->fps > MT9P012_MAX_FPS) && (!qvga_120))
		sensor->fps = MT9P012_MAX_FPS;

	timeperframe->numerator = 1;
	timeperframe->denominator = sensor->fps;
	if ((pix->width <= VIDEO_WIDTH_4X_BINN) && (sensor->fps > 15) &&
							(!qvga_120))
		xclk_current = MT9P012_XCLK_NOM_2;
	else
		xclk_current = MT9P012_XCLK_NOM_1;
		
	

	return xclk_current;
}

/**
 * mt9p012_configure - Configure the mt9p012 for the specified image mode
 * @s: pointer to standard V4L2 device structure
 *
 * Configure the mt9p012 for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the mt9p012.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int mt9p012_configure(struct v4l2_int_device *s)
{

	struct mt9p012_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &sensor->pix;
	struct i2c_client *client = sensor->i2c_client;
	enum image_size isize;
	unsigned int fps_index;
	int err;

	isize = mt9p012_find_isize(pix->width);

	/* common register initialization */
	err = mt9p012_write_regs(client, initial_list_delay, 1);
	if (err)
		return err;
	mdelay(5);

	err = mt9p012_write_regs(client, initial_list, I2C_INITIAL_LIST_SIZE);

	if (err)
		return err;

	fps_index = mt9p012_find_fps_index(sensor->fps, isize);

	/* configure image size and pixel format */
	if (pix->pixelformat == V4L2_PIX_FMT_SGRBG10) {
		if (sensor->fps == 120) {
		// no 120 fps mode
		} else {

			err = mt9p012_write_regs(client,
				mt9p012_reg_init[fps_index][isize].reg_list,
				mt9p012_reg_init[fps_index][isize].list_size);
		}
	} else if (pix->pixelformat == V4L2_PIX_FMT_PATT) {
	// no test pattern mode
	}
	if (err)
		return err;

	
	/* configure streaming ON */
	err = mt9p012_write_regs(client, stream_on_list,
						I2C_STREAM_ON_LIST_SIZE);
		
	mdelay(1);

	return err;
}

/**
 * mt9p012_detect - Detect if an mt9p012 is present, and if so which revision
 * @client: pointer to the i2c client driver structure
 *
 * Detect if an mt9p012 is present, and if so which revision.
 * A device is considered to be detected if the manufacturer ID (MIDH and MIDL)
 * and the product ID (PID) registers match the expected values.
 * Any value of the version ID (VER) register is accepted.
 * Here are the version numbers we know about:
 *	0x48 --> mt9p012 Revision 1 or mt9p012 Revision 2
 *	0x49 --> mt9p012 Revision 3
 * Returns a negative error number if no device is detected, or the
 * non-negative value of the version ID register if a device is detected.
 */
static int
mt9p012_detect(struct i2c_client *client)
{
	u32 model_id, mfr_id, rev;
	

	if (!client)
		return -ENODEV;
	if (mt9p012_read_reg(client, MT9P012_16BIT, REG_MODEL_ID, &model_id)){
		return -ENODEV;
	}
	if (mt9p012_read_reg(client, MT9P012_16BIT, REG_MODEL_ID, &model_id))
		return -ENODEV;

	if (mt9p012_read_reg(client, MT9P012_8BIT, REG_MANUFACTURER_ID,
				&mfr_id))
		return -ENODEV;
	if (mt9p012_read_reg(client, MT9P012_8BIT, REG_REVISION_NUMBER, &rev))
		return -ENODEV;

	dev_info(&client->dev, "model id detected 0x%x mfr 0x%x\n", model_id,
								mfr_id);
	if ((model_id != MT9P012_MOD_ID) || (mfr_id != MT9P012_MFR_ID)) {
		 //We didn't read the values we expected, so
		 //* this must not be an MT9P012.
		 
		dev_warn(&client->dev, "model id mismatch 0x%x mfr 0x%x\n",
			model_id, mfr_id);

		return -ENODEV;
	}
	

	return 0;

}

/**
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
				struct v4l2_queryctrl *qc)
{
	int i;
	

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;
		
	return 0;
}

/**
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	struct vcontrol *lvc;
	int i;
	

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case  V4L2_CID_EXPOSURE:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_GAIN:
		vc->value = lvc->current_value;
		break;
	}
	

	return 0;
}

/**
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	int retval = -EINVAL;
	int i;
	struct vcontrol *lvc;
	

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		retval = mt9p012sensor_set_exposure_time(vc->value, s, lvc);
		break;
	case V4L2_CID_GAIN:
		retval = mt9p012sensor_set_gain(vc->value, s, lvc);
		break;
	}
	

	return retval;
}


/**
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
				   struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;
	

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
	break;
	default:
		return -EINVAL;
	}

	fmt->flags = mt9p012_formats[index].flags;
	strlcpy(fmt->description, mt9p012_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = mt9p012_formats[index].pixelformat;
	

	return 0;
}

/**
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int ioctl_try_fmt_cap(struct v4l2_int_device *s,
			     struct v4l2_format *f)
{
	enum image_size isize;
	int ifmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct mt9p012_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix2 = &sensor->pix;

	isize = mt9p012_calc_size(pix->width, pix->height);

	pix->width = mt9p012_sizes[isize].width;
	pix->height = mt9p012_sizes[isize].height;
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == mt9p012_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = mt9p012_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_RGB555X:
	default:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}
	*pix2 = *pix;
		
	return 0;
}

/**
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int ioctl_s_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;
	

	size_updated = 1;
	rval = ioctl_try_fmt_cap(s, f);
	if (rval)
		return rval;
	else
		sensor->pix = *pix;
	

	return rval;
}

/**
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct mt9p012_sensor *sensor = s->priv;
	
	f->fmt.pix = sensor->pix;
	

	return 0;
}

/**
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->timeperframe;
	

	return 0;
}

/**
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	

	fps_updated = 1;
	sensor->timeperframe = *timeperframe;
	mt9p012sensor_calc_xclk(client);
	*timeperframe = sensor->timeperframe;
	

	return 0;
}

/**
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct mt9p012_sensor *sensor = s->priv;
	
	

	return sensor->pdata->priv_data_set(p);
}

/**
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	int rval;

	if (on == V4L2_POWER_OFF) {
		update_exp_time = 0;
		update_gain = 0;
		size_updated = 0;
		fps_updated = 0;
	}

	if ((on == V4L2_POWER_STANDBY) && (sensor->state == SENSOR_DETECTED))
		mt9p012_write_regs(c, stream_off_list,
						I2C_STREAM_OFF_LIST_SIZE);

	if (on != V4L2_POWER_ON)
		isp_set_xclk(0, MT9P012_USE_XCLKA);
	else
		isp_set_xclk(xclk_current, MT9P012_USE_XCLKA);


	rval = sensor->pdata->power_set(on);
	if (rval < 0) {
		dev_err(&c->dev, "Unable to set the power state: " DRIVER_NAME
								" sensor\n");
		isp_set_xclk(0, MT9P012_USE_XCLKA);
		return rval;
	}
	if ((current_power_state == V4L2_POWER_STANDBY) &&
					(on == V4L2_POWER_ON) &&
					(sensor->state == SENSOR_DETECTED))
		mt9p012_configure(s);

	if ((on == V4L2_POWER_ON) && (sensor->state == SENSOR_NOT_DETECTED)) {
		rval = mt9p012_detect(c);
		if (rval < 0) {
			dev_err(&c->dev, "Unable to detect " DRIVER_NAME
								" sensor\n");
			sensor->state = SENSOR_NOT_DETECTED;
			return rval;
		}
		sensor->state = SENSOR_DETECTED;
		sensor->ver = rval;
		pr_info(DRIVER_NAME " chip version 0x%02x detected\n",
								sensor->ver);
	}

	current_power_state = on;

	return 0;
}

/**
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call mt9p012_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach.  The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.  Returns 0 if
 * mt9p012 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	int err;
	

	err = mt9p012_detect(c);
	if (err < 0) {
		dev_err(&c->dev, "Unable to detect " DRIVER_NAME " sensor\n");
		return err;
	}

	sensor->ver = err;
	pr_info(DRIVER_NAME " chip version 0x%02x detected\n", sensor->ver);

	return 0;
}
/**
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 **/
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
					struct v4l2_frmsizeenum *frms)
{
	int ifmt;
	

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frms->pixel_format == mt9p012_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= 5)
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = mt9p012_sizes[frms->index].width;
	frms->discrete.height = mt9p012_sizes[frms->index].height;

	return 0;
}

const struct v4l2_fract mt9p012_frameintervals[] = {
	{  .numerator = 1, .denominator = 11 },
	{  .numerator = 1, .denominator = 15 },
	{  .numerator = 1, .denominator = 20 },
	{  .numerator = 1, .denominator = 25 },
	{  .numerator = 1, .denominator = 30 },
	{  .numerator = 1, .denominator = 120 },
};

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					struct v4l2_frmivalenum *frmi)
{
	int ifmt;
	

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frmi->pixel_format == mt9p012_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */

	if (((frmi->width == mt9p012_sizes[4].width) &&
				(frmi->height == mt9p012_sizes[4].height)) ||
				((frmi->width == mt9p012_sizes[3].width) &&
				(frmi->height == mt9p012_sizes[3].height))) {
		/* FIXME: The only frameinterval supported by 5MP and 3MP
		 * capture sizes is 1/11 fps
		 */
		if (frmi->index != 0)
			return -EINVAL;
	} else if ((frmi->width == mt9p012_sizes[1].width) &&
				(frmi->height == mt9p012_sizes[1].height)) {
		/* QVGA base size supports all framerates, including 120 fps!
		 */
		if (frmi->index >= 6)
			return -EINVAL;
	} else {
		if (frmi->index >= 5)
			return -EINVAL;
	}

	frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frmi->discrete.numerator =
				mt9p012_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				mt9p012_frameintervals[frmi->index].denominator;
	

	return 0;
}

static struct v4l2_int_ioctl_desc mt9p012_ioctl_desc[] = {
	{ .num = vidioc_int_enum_framesizes_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ .num = vidioc_int_enum_frameintervals_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
	{ .num = vidioc_int_dev_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ .num = vidioc_int_dev_exit_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_exit },
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_priv },
	{ .num = vidioc_int_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_init },
	{ .num = vidioc_int_enum_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ .num = vidioc_int_try_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_try_fmt_cap },
	{ .num = vidioc_int_g_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ .num = vidioc_int_s_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_fmt_cap },
	{ .num = vidioc_int_g_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ .num = vidioc_int_s_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_queryctrl },
	{ .num = vidioc_int_g_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_ctrl },
};

static struct v4l2_int_slave mt9p012_slave = {
	.ioctls = mt9p012_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9p012_ioctl_desc),
};

static struct v4l2_int_device mt9p012_int_device = {
	.module = THIS_MODULE,
	.name = DRIVER_NAME,
	.priv = &mt9p012,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &mt9p012_slave,
	},
};

/**
 * mt9p012_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int
mt9p012_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mt9p012_sensor *sensor = &mt9p012;
	int err;
	

	if (i2c_get_clientdata(client))
		return -EBUSY;

	sensor->pdata = client->dev.platform_data;

	if (!sensor->pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	sensor->v4l2_int_device = &mt9p012_int_device;
	sensor->i2c_client = client;

	i2c_set_clientdata(client, sensor);

	/* Make the default capture format QCIF V4L2_PIX_FMT_SGRBG10 */
	sensor->pix.width = VIDEO_WIDTH_4X_BINN_SCALED;
	sensor->pix.height = VIDEO_HEIGHT_4X_BINN_SCALED;
	sensor->pix.pixelformat = V4L2_PIX_FMT_SGRBG10;

	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err)
		i2c_set_clientdata(client, NULL);
	

	return err;
}

/**
 * mt9p012_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of mt9p012_probe().
 */
static int __exit
mt9p012_remove(struct i2c_client *client)
{
	struct mt9p012_sensor *sensor = i2c_get_clientdata(client);
	

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	i2c_set_clientdata(client, NULL);
	

	return 0;
}

static const struct i2c_device_id mt9p012_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mt9p012_id);

static struct i2c_driver mt9p012sensor_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mt9p012_probe,
	.remove = __exit_p(mt9p012_remove),
	.id_table = mt9p012_id,
};

static struct mt9p012_sensor mt9p012 = {
	.timeperframe = {
		.numerator = 1,
		.denominator = 15,
	},
	.state = SENSOR_NOT_DETECTED,
};

/**
 * mt9p012sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init mt9p012sensor_init(void)
{
	int err;
	update_exp_time = 0;
	update_gain = 0;
	size_updated = 0;
	fps_updated = 0;
		
	err = i2c_add_driver(&mt9p012sensor_i2c_driver);
	if (err) {
		printk(KERN_ERR "Failed to register" DRIVER_NAME ".\n");
		return err;
	}
	

	return 0;
}
late_initcall(mt9p012sensor_init);

/**
 * mt9p012sensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of mt9p012sensor_init.
 */
static void __exit mt9p012sensor_cleanup(void)
{
	i2c_del_driver(&mt9p012sensor_i2c_driver);
}
module_exit(mt9p012sensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mt9p012 camera sensor driver");

