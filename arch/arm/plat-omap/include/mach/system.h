/*
 * Copied from linux/include/asm-arm/arch-sa1100/system.h
 * Copyright (c) 1999 Nicolas Pitre <nico@cam.org>
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H
#include <linux/clk.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>

#include <mach/prcm.h>
#include <asm/proc-fns.h>

#ifndef CONFIG_MACH_VOICEBLUE
#define voiceblue_reset()		do {} while (0)
#endif

static inline void arch_idle(void)
{
	cpu_do_idle();
}

#ifndef CONFIG_MACH_ARCHOS
static inline void omap1_arch_reset(char mode)
{
	/*
	 * Workaround for 5912/1611b bug mentioned in sprz209d.pdf p. 28
	 * "Global Software Reset Affects Traffic Controller Frequency".
	 */
	if (cpu_is_omap5912()) {
		omap_writew(omap_readw(DPLL_CTL) & ~(1 << 4),
				 DPLL_CTL);
		omap_writew(0x8, ARM_RSTCT1);
	}

	if (machine_is_voiceblue())
		voiceblue_reset();
	else
		omap_writew(1, ARM_RSTCT1);
}

static inline void arch_reset(char mode)
{
	if (!cpu_class_is_omap2())
		omap1_arch_reset(mode);
	else
		omap_prcm_arch_reset(mode);
}
#else
extern void archos_reset_board(void);
static inline void arch_reset(char mode)
{
	archos_reset_board();
}
#endif

#endif
