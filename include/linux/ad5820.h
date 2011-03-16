/*
 *   Definitions for AD5820 Accelerometer chip
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

#ifndef AD5820_H
#define AD5820_H


/*
 * Mask and bit definitions
 */

#define AD5820_POWER_SHIFT		15
#define AD5820_POWER_MASK		(0x01 << AD5820_POWER_SHIFT)
#define AD5820_POWER_ON			(0x00 << AD5820_POWER_SHIFT)
#define AD5820_POWER_DOWN		(0x01 << AD5820_POWER_SHIFT)

#define AD5820_DAC_SHIFT		4
#define AD5820_DAC_MASK			(0x3FF << AD5820_DAC_SHIFT)

#define AD5820_SLEW_RATE_SHIFT		0
#define AD5820_SLEW_RATE_MASK		(0x07 << AD5820_SLEW_RATE_SHIFT)


/*
struct ad5820_pdata {
};
*/

#endif
