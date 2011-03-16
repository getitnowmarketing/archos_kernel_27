/*
 * drivers/media/video/dw9710.h
 *
 * Register defines for Auto Focus device
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Contributors:
 * 	Troy Laramy <t-laramy@ti.com>
 * 	Mohit Jalori <mjalori@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */
#ifndef CAMAF_AD5820_H
#define CAMAF_AD5820_H

#include <media/v4l2-int-device.h>
#define AD5820_AF_I2C_ADDR	0x0C
#define AD5820_NAME 		"AD5820"
#define AD5820_I2C_RETRY_COUNT	5
#define MAX_FOCUS_POS	0x3FF

#define CAMAF_AD5820_DISABLE		0x1
#define CAMAF_AD5820_ENABLE		0x0
#define CAMAF_AD5820_POWERDN(ARG)	(((ARG) & 0x1) << 15)
#define CAMAF_AD5820_POWERDN_R(ARG)	(((ARG) >> 15) & 0x1)


// 8-bit input
#define CAMAF_AD5820_DATA(ARG)		(((ARG) & 0x3FF) << 4)
#define CAMAF_AD5820_DATA_R(ARG)	(((ARG) >> 4) & 0x3FF)

#define CAMAF_AD5820_MODE(ARG)		((ARG) & 0xF)
#define CAMAF_AD5820_MODE_R(ARG)	((ARG) & 0xF)

// 10-bit input
//#define CAMAF_AD5820_DATA(ARG)		(((ARG) & 0xFF) << 6)
//#define CAMAF_AD5820_DATA_R(ARG)	(((ARG) >> 6) & 0xFF)

#define CAMAF_FREQUENCY_EQ1(mclk)     	((u16)(mclk/16000))

/* ioctls definition */
#define		AF_IOC_BASE			       'R'
#define		AF_IOC_MAXNR				2

/*Ioctl options which are to be passed while calling the ioctl*/
#define	AF_SET_POSITION		_IOWR(AF_IOC_BASE, 1, int)
#define	AF_GET_POSITION		_IOWR(AF_IOC_BASE, 2, int)

/* State of lens */
#define LENS_DETECTED 		1
#define LENS_NOT_DETECTED	0

/* Focus control values */
#define DEF_LENS_POSN		0	/* 0x7F */
#define LENS_POSN_STEP		1

enum ad5820_drive_mode {
	AD5820_DRIVE_MODE_DIRECT_0,
	AD5820_DRIVE_MODE_LINEAR_50000,
	AD5820_DRIVE_MODE_LINEAR_100000,
	AD5820_DRIVE_MODE_LINEAR_200000,
	AD5820_DRIVE_MODE_LINEAR_400000,
	AD5820_DRIVE_MODE_LINEAR_800000,
	AD5820_DRIVE_MODE_LINEAR_1600000,
	AD5820_DRIVE_MODE_LINEAR_3200000,
	AD5820_DRIVE_MODE_DIRECT_1,
	AD5820_DRIVE_MODE_64_16_50000,
	AD5820_DRIVE_MODE_64_16_100000,
	AD5820_DRIVE_MODE_64_16_200000,
	AD5820_DRIVE_MODE_64_16_400000,
	AD5820_DRIVE_MODE_64_16_800000,
	AD5820_DRIVE_MODE_64_16_1600000,
	AD5820_DRIVE_MODE_64_16_3200000,
};

enum ad5820_drive_times {
	AD5820_TIME_0 = 0,
	AD5820_TIME_50000 = 50000,
	AD5820_TIME_100000 = 100000,
	AD5820_TIME_200000 = 200000,
	AD5820_TIME_400000 = 400000,
	AD5820_TIME_800000 = 800000,
	AD5820_TIME_1600000 = 1600000,
	AD5820_TIME_3200000 = 3200000,
};

/**
 * struct ad5820_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @priv_data_set: device private data (pointer) access function
 */
struct ad5820_platform_data {
	int (*power_set)(enum v4l2_power power);
	int (*priv_data_set)(void *);
};

/*
 * Sets the specified focus value [0(far) - 100(near)]
 */
int ad5820_af_setfocus(u16 posn);
int ad5820_af_getfocus(u16 *value);


#define AD5820_MAX_STEP_COUNT (128)

struct ad5820_micro_step
{
    __s32 lens_pos;
    __u32 time_sleep_ns;
    struct timespec exec_time;
} __attribute__((packed));

struct ad5820_micro_steps_list
{
    __u32                           count;
    struct ad5820_micro_step        steps[AD5820_MAX_STEP_COUNT];
} __attribute__((packed));

#endif /* End of of CAMAF_AD5820_H */

