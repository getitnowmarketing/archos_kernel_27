/*
 *   Definitions for MMA7456L Accelerometer chip
 *
 *   Copyright (c) by Jean-Christophe Rona <rona@archos.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef MMA7456L_H
#define MMA7456L_H

#include <linux/ioctl.h>

/*
 * MMA7456L registers
 */

#define REG_XOUTL			0x00
#define REG_XOUTH			0x01
#define REG_YOUTL			0x02
#define REG_YOUTH			0x03
#define REG_ZOUTL			0x04
#define REG_ZOUTH			0x05
#define REG_XOUT8			0x06
#define REG_YOUT8			0x07
#define REG_ZOUT8			0x08
#define REG_STATUS			0x09
#define REG_DETSRC			0x0A
#define REG_TOUT			0x0B
/* Reserved				0x0C */
#define REG_I2CAD			0x0D
#define REG_USRINF			0x0E
#define REG_WHOAMI			0x0F
#define REG_XOFFL			0x10
#define REG_XOFFH			0x11
#define REG_YOFFL			0x12
#define REG_YOFFH			0x13
#define REG_ZOFFL			0x14
#define REG_ZOFFH			0x15
#define REG_MCTL			0x16
#define REG_INTRST			0x17
#define REG_CTL1			0x18
#define REG_CTL2			0x19
#define REG_LDTH			0x1A
#define REG_PDTH			0x1B
#define REG_PW				0x1C
#define REG_LT				0x1D
#define REG_TW				0x1E
/* Reserved				0xIF */


/*
 * Mask and bit definitions of MMA7456L registers
 */

#define MMA7456L_MODE_SHIFT		0
#define MMA7456L_MODE_MASK		(0x03 << MMA7456L_MODE_SHIFT)
#define MMA7456L_MODE_STANDBY		(0x00 << MMA7456L_MODE_SHIFT)
#define MMA7456L_MODE_MEASURE		(0x01 << MMA7456L_MODE_SHIFT)
#define MMA7456L_MODE_LEVELDET		(0x02 << MMA7456L_MODE_SHIFT)
#define MMA7456L_MODE_PULSEDET		(0x03 << MMA7456L_MODE_SHIFT)

#define MMA7456L_RANGE_SHIFT		2
#define MMA7456L_RANGE_MASK		(0x03 << MMA7456L_RANGE_SHIFT)
#define MMA7456L_RANGE_2G		(0x01 << MMA7456L_RANGE_SHIFT)
#define MMA7456L_RANGE_4G		(0x02 << MMA7456L_RANGE_SHIFT)
#define MMA7456L_RANGE_8G		(0x00 << MMA7456L_RANGE_SHIFT)

#define MMA7456L_DATA_RDY_INT1		(0x1 << 6)

#define MMA7456L_STATUS_DRDY		(0x01)

#define MMA7456L_OUTPUT_8BITS		0
#define MMA7456L_OUTPUT_10BITS		1

#define MMA7456L_PULSE_DURATION_MASK	0xFF
#define MMA7456L_PULSE_THRESHOLD_MASK	0x7F
#define MMA7456L_LEVEL_THRESHOLD_MASK	0x7F


struct accel_data {
	short x;
	short y;
	short z;
};


/*
 * IOCTLs
 */

#define MMAIO				0xA1

#define MMA7456L_IOCTL_S_MODE		_IOW(MMAIO, 0x01, short)
#define MMA7456L_IOCTL_G_MODE		_IOR(MMAIO, 0x02, short)
#define MMA7456L_IOCTL_S_RANGE		_IOW(MMAIO, 0x03, short)
#define MMA7456L_IOCTL_G_RANGE		_IOR(MMAIO, 0x04, short)
#define MMA7456L_IOCTL_S_POLL_DELAY	_IOW(MMAIO, 0x05, int)
#define MMA7456L_IOCTL_G_POLL_DELAY	_IOR(MMAIO, 0x06, int)
#define MMA7456L_IOCTL_S_OUTPUT_LENGTH	_IOW(MMAIO, 0x07, short)
#define MMA7456L_IOCTL_G_OUTPUT_LENGTH	_IOR(MMAIO, 0x08, short)
#define MMA7456L_IOCTL_S_PULSE_DURATION	_IOW(MMAIO, 0x09, short)
#define MMA7456L_IOCTL_G_PULSE_DURATION	_IOR(MMAIO, 0x0A, short)
#define MMA7456L_IOCTL_S_PULSE_THRSHOLD	_IOW(MMAIO, 0x0B, short)
#define MMA7456L_IOCTL_G_PULSE_THRSHOLD	_IOR(MMAIO, 0x0C, short)
#define MMA7456L_IOCTL_GP_EVENT		_IOW(MMAIO, 0x0D, int)
#define MMA7456L_IOCTL_G_ACCEL_DATA	_IOR(MMAIO, 0x0E, struct accel_data)

struct mma7456l_pdata {
	int irq1;
	int irq2;
};

#endif
