/* linux/include/asm-arm/arch-s3c2410/spi-gpio.h
 *
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 - SPI Controller platfrom_device info
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ARCH_MACH_SPIGPIO_H
#define __ARCH_MACH_SPIGPIO_H

struct spigpio_info {
	int	pin_clk;
	int	pin_mosi;
	int	pin_miso;
	int	pin_cs;
};

#endif /* __ARCH_MACH_SPIGPIO_H */
