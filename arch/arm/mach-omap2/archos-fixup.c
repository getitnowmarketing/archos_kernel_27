/*
 * linux/arch/arm/mach-omap2/board-archosg6-memory.c
 *
 * Copyright (C) Archos S.A.,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>

#ifdef CONFIG_ARCHOS_FIXUP
static char command_line[COMMAND_LINE_SIZE] __initdata = CONFIG_CMDLINE;

void __init fixup_archos(struct machine_desc *desc,
		struct tag *tags, char **cmdline, struct meminfo *mi)
{
	printk("fixup_archos: [%s]\n", command_line);
	*cmdline = command_line;
}
#endif
