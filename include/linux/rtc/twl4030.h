/*
 * twl4030.h -- TWL4030 Real Time Clock interface
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Author: Jagadeesh Bhaskar Pakaravoor <j-pakaravoor@ti.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
* 2 of the License, or (at your option) any later version.
 */

/* Clear periodic interrupts */
#define RTC_IRQP_CLEAR		_IO('p', 0x13)
/* set periodic interrupts every second */
#define RTC_IRQP_SET_SEC	_IO('p', 0x15)
/* set periodic interrupts every minute */
#define RTC_IRQP_SET_MIN	_IO('p', 0x16)
/* set periodic interrupts every hour */
#define RTC_IRQP_SET_HR		_IO('p', 0x17)
/* set periodic interrupts every day */
#define RTC_IRQP_SET_DAY	_IO('p', 0x18)
