/*
 * linux/arch/arm/mach-omap2/pm_34xx.c
 *
 * OMAP3 Power Management Routines
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <linux/io.h>
#include <linux/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>
#include <linux/mm.h>
#include <asm/mmu.h>
#include <asm/tlbflush.h>

#include <mach/irqs.h>
#include <mach/clock.h>
#include <mach/sram.h>
#include <mach/pm.h>
#include <linux/tick.h>
#include <mach/resource.h>
#include <mach/prcm_34xx.h>
#include <mach/cpu.h>
#ifdef CONFIG_OMAP34XX_OFFMODE
#include <mach/io.h>
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
#include <mach/sdrc.h>

#include "sdrc.h"
#include "prcm-regs.h"
#include "ti-compat.h"
#include "pm_34xx.h"

#ifdef CONFIG_MACH_ARCHOS
#include <mach/board-archos.h>
#endif

/* #define DEBUG_PM_34XX 1 */
#define DEBUG_PM_34XX_PADCONF 1

#ifdef DEBUG_PM_34XX
#  define DPRINTK(fmt, args...) printk(KERN_ERR "%s: " fmt, __FUNCTION__ , \
								## args)
#else
#  define DPRINTK(fmt, args...)
#endif

#ifdef OMAP3_START_RNG
u32 prepwst_core_rng = 0xFF;
#endif

extern int is_console_wakeup(void);
#define IOPAD_WKUP 1

unsigned long awake_time_end;

#ifdef DEBUG_PM_34XX_PADCONF
struct save_wkst {
	unsigned long WKST_WKUP;
	unsigned long WKST_PER;
	unsigned long WKST_USBHOST;
	unsigned long WKST1_CORE;
	unsigned long WKST3_CORE;	
	int interrupt;
};

struct save_wkst save_wkst;

#define CONTROL_PADCONF(name) {#name, (u32*)&CONTROL_PADCONF_##name, 0}

static struct padconf_dump {
	const char *name;
	u32 *addr;
	u32 value;
	u32 pre_value;
} padconf_dump[] = {
	CONTROL_PADCONF(GPMC_A2),
	CONTROL_PADCONF(GPMC_A2),
	CONTROL_PADCONF(GPMC_A4),
	CONTROL_PADCONF(GPMC_A6),
	CONTROL_PADCONF(GPMC_A8),
	CONTROL_PADCONF(GPMC_A10),
	CONTROL_PADCONF(GPMC_D1),
	CONTROL_PADCONF(GPMC_D3),
	CONTROL_PADCONF(GPMC_D5),
	CONTROL_PADCONF(GPMC_D7),
	CONTROL_PADCONF(GPMC_D9),
	CONTROL_PADCONF(GPMC_D11),
	CONTROL_PADCONF(GPMC_D13),
	CONTROL_PADCONF(GPMC_D15),
	CONTROL_PADCONF(GPMC_NCS1),
	CONTROL_PADCONF(GPMC_NCS3),
	CONTROL_PADCONF(GPMC_NCS5),
	CONTROL_PADCONF(GPMC_NCS7),
	CONTROL_PADCONF(GPMC_NADV_ALE),
	CONTROL_PADCONF(GPMC_NWE),
	CONTROL_PADCONF(GPMC_NBE1),
	CONTROL_PADCONF(GPMC_WAIT0),
	CONTROL_PADCONF(GPMC_WAIT2),
	CONTROL_PADCONF(DSS_PCLK),
	CONTROL_PADCONF(DSS_VSYNC),
	CONTROL_PADCONF(DSS_DATA0),
	CONTROL_PADCONF(DSS_DATA2),
	CONTROL_PADCONF(DSS_DATA4),
	CONTROL_PADCONF(DSS_DATA6),
	CONTROL_PADCONF(DSS_DATA8),
	CONTROL_PADCONF(DSS_DATA10),
	CONTROL_PADCONF(DSS_DATA12),
	CONTROL_PADCONF(DSS_DATA14),
	CONTROL_PADCONF(DSS_DATA16),
	CONTROL_PADCONF(DSS_DATA18),
	CONTROL_PADCONF(DSS_DATA20),
	CONTROL_PADCONF(DSS_DATA22),
	CONTROL_PADCONF(CAM_HS),
	CONTROL_PADCONF(CAM_XCLKA),
	CONTROL_PADCONF(CAM_FLD),
	CONTROL_PADCONF(CAM_D1),      
	CONTROL_PADCONF(CAM_D3),       
	CONTROL_PADCONF(CAM_D5),       
	CONTROL_PADCONF(CAM_D7),       
	CONTROL_PADCONF(CAM_D9),       
	CONTROL_PADCONF(CAM_D11),       
	CONTROL_PADCONF(CAM_WEN),
	CONTROL_PADCONF(CSI2_DX0),
	CONTROL_PADCONF(CSI2_DX1),
	CONTROL_PADCONF(MCBSP2_FSX),
	CONTROL_PADCONF(MCBSP2_DR),
	CONTROL_PADCONF(MMC1_CLK),
	CONTROL_PADCONF(MMC1_DAT0),
	CONTROL_PADCONF(MMC1_DAT2),
	CONTROL_PADCONF(MMC1_DAT4),
	CONTROL_PADCONF(MMC1_DAT6),
	CONTROL_PADCONF(MMC2_CLK),
	CONTROL_PADCONF(MMC2_DAT0),
	CONTROL_PADCONF(MMC2_DAT2),
	CONTROL_PADCONF(MMC2_DAT4),
	CONTROL_PADCONF(MMC2_DAT6),
	CONTROL_PADCONF(MCBSP3_DX),
	CONTROL_PADCONF(MCBSP3_CLKX),
	CONTROL_PADCONF(UART2_CTS),
	CONTROL_PADCONF(UART2_TX),
	CONTROL_PADCONF(UART1_TX),
	CONTROL_PADCONF(UART1_CTS),
	CONTROL_PADCONF(MCBSP4_CLKX),
	CONTROL_PADCONF(MCBSP4_DX),
	CONTROL_PADCONF(MCBSP1_CLKR),
	CONTROL_PADCONF(MCBSP1_DX),
	CONTROL_PADCONF(MCBSP_CLKS),
	CONTROL_PADCONF(MCBSP1_CLKX),
	CONTROL_PADCONF(UART3_RTS_SD),
	CONTROL_PADCONF(UART3_TX_IRTX),
	CONTROL_PADCONF(HSUSB0_STP),
	CONTROL_PADCONF(HSUSB0_NXT),
	CONTROL_PADCONF(HSUSB0_DATA1),
	CONTROL_PADCONF(HSUSB0_DATA3),
	CONTROL_PADCONF(HSUSB0_DATA5),
	CONTROL_PADCONF(HSUSB0_DATA7),
	CONTROL_PADCONF(I2C1_SDA),
	CONTROL_PADCONF(I2C2_SDA),
	CONTROL_PADCONF(I2C3_SDA),
	CONTROL_PADCONF(MCSPI1_CLK),
	CONTROL_PADCONF(MCSPI1_SOMI),
	CONTROL_PADCONF(MCSPI1_CS1),
	CONTROL_PADCONF(MCSPI1_CS3),
	CONTROL_PADCONF(MCSPI2_SIMO),
	CONTROL_PADCONF(MCSPI2_CS0),
	CONTROL_PADCONF(SYS_NIRQ),
	CONTROL_PADCONF(SAD2D_MCAD0),
	CONTROL_PADCONF(SAD2D_MCAD2),
	CONTROL_PADCONF(SAD2D_MCAD4),
	CONTROL_PADCONF(SAD2D_MCAD6),
	CONTROL_PADCONF(SAD2D_MCAD8),
	CONTROL_PADCONF(SAD2D_MCAD10),
	CONTROL_PADCONF(SAD2D_MCAD12),
	CONTROL_PADCONF(SAD2D_MCAD14),
	CONTROL_PADCONF(SAD2D_MCAD16),
	CONTROL_PADCONF(SAD2D_MCAD18),
	CONTROL_PADCONF(SAD2D_MCAD20),
	CONTROL_PADCONF(SAD2D_MCAD22),
	CONTROL_PADCONF(SAD2D_MCAD24),
	CONTROL_PADCONF(SAD2D_MCAD26),
	CONTROL_PADCONF(SAD2D_MCAD28),
	CONTROL_PADCONF(SAD2D_MCAD30),
	CONTROL_PADCONF(SAD2D_MCAD32),
	CONTROL_PADCONF(SAD2D_MCAD34),
	CONTROL_PADCONF(SAD2D_MCAD36),
	CONTROL_PADCONF(SAD2D_NRESPWRON),
	CONTROL_PADCONF(SAD2D_ARMNIRQ),
	CONTROL_PADCONF(SAD2D_SPINT),
	CONTROL_PADCONF(SAD2D_DMAREQ0),
	CONTROL_PADCONF(SAD2D_DMAREQ2),
	CONTROL_PADCONF(SAD2D_NTRST),
	CONTROL_PADCONF(SAD2D_TDO),
	CONTROL_PADCONF(SAD2D_TCK),
	CONTROL_PADCONF(SAD2D_MSTDBY),
	CONTROL_PADCONF(SAD2D_IDLEACK),
	CONTROL_PADCONF(SAD2D_SWRITE),
	CONTROL_PADCONF(SAD2D_SREAD),
	CONTROL_PADCONF(SAD2D_SBUSFLAG),
	CONTROL_PADCONF(SDRC_CKE1),
	CONTROL_PADCONF(ETK_CLK),    
	CONTROL_PADCONF(ETK_D0),      
	CONTROL_PADCONF(ETK_D2),       
	CONTROL_PADCONF(ETK_D4),       
	CONTROL_PADCONF(ETK_D6),       
	CONTROL_PADCONF(ETK_D8),         
	CONTROL_PADCONF(ETK_D10),       
	CONTROL_PADCONF(ETK_D12),      
	CONTROL_PADCONF(ETK_D14),      
	CONTROL_PADCONF(I2C4_SCL),
	CONTROL_PADCONF(SYS_32K),
	CONTROL_PADCONF(SYS_NRESWARM),
	CONTROL_PADCONF(SYS_BOOT1),
	CONTROL_PADCONF(SYS_BOOT3),
	CONTROL_PADCONF(SYS_BOOT5),
	CONTROL_PADCONF(SYS_OFF_MODE),
	CONTROL_PADCONF(JTAG_NTRST),
	CONTROL_PADCONF(JTAG_TMS_TMSC),
	CONTROL_PADCONF(JTAG_EMU0),
	CONTROL_PADCONF(SAD2D_SWAKEUP),
	CONTROL_PADCONF(JTAG_TDO),
};

static void control_padconf_save(void)
{
	int i;
	for (i=0; i<sizeof(padconf_dump)/sizeof(struct padconf_dump); i++) {
		padconf_dump[i].value = *(padconf_dump[i].addr);
	}
}

static void control_padconf_dump(void)
{
	int i;
	for (i=0; i<sizeof(padconf_dump)/sizeof(struct padconf_dump); i++) {
		printk("%20s: 0x%08x <- 0x%08x\n", padconf_dump[i].name, ((unsigned long)padconf_dump[i].value) & 0xc000c000, ((unsigned long)padconf_dump[i].pre_value) & 0xc000c000);
	}
}
void control_padconf_pre_save(void)
{
	int i;
	for (i=0; i<sizeof(padconf_dump)/sizeof(struct padconf_dump); i++) {
		padconf_dump[i].pre_value = *(padconf_dump[i].addr);
	}
}
#else
void control_padconf_pre_save(void) {};
#endif

static void restore_control_register(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c1, c0, 0" : : "r" (val));
}

/* Function to restore the table entry that was modified for enabling MMU*/
static void restore_table_entry(void)
{
	u32 *scratchpad_address;
	u32 previous_value, control_reg_value;
	u32 *address;
	/* Get virtual address of SCRATCHPAD */
	scratchpad_address = (u32 *) io_p2v(SCRATCHPAD);
	/* Get address of entry that was modified */
	address = (u32 *) *(scratchpad_address + TABLE_ADDRESS_OFFSET);
	/* Get the previous value which needs to be restored */
	previous_value = *(scratchpad_address + TABLE_VALUE_OFFSET);
	/* Convert address to virtual address */
	address = __va(address);
	/* Restore table entry */
	*address = previous_value;
	/* Flush TLB */
	flush_tlb_all();
	control_reg_value = *(scratchpad_address + CONTROL_REG_VALUE_OFFSET);
	/* Restore control register*/
	/* This will enable caches and prediction */
	restore_control_register(control_reg_value);
}

static void (*_omap_sram_idle)(u32 *addr, int save_state);

void omap_sram_idle(void)
{
	/* Variable to tell what needs to be saved and restored
	 * in omap_sram_idle*/
	/* save_state = 0 => Nothing to save and restored */
	/* save_state = 1 => Only L1 and logic lost */
	/* save_state = 2 => Only L2 lost */
	/* save_state = 3 => L1, L2 and logic lost */
	int save_state = 0;
	if (!_omap_sram_idle)
		return;
	switch (target_state.mpu_state) {
	case PRCM_MPU_ACTIVE:
	case PRCM_MPU_INACTIVE:
	case PRCM_MPU_CSWR_L2RET:
		/* No need to save context */
		save_state = 0;
		break;
	case PRCM_MPU_OSWR_L2RET:
		/* L1 and Logic lost */
		save_state = 1;
		break;
	case PRCM_MPU_CSWR_L2OFF:
		/* Only L2 lost */
		save_state = 2;
		break;
	case PRCM_MPU_OSWR_L2OFF:
	case PRCM_MPU_OFF:
		/* L1, L2 and logic lost */
		save_state = 3;
		break;
	default:
		/* Invalid state */
		printk(KERN_ERR "Invalid mpu state in sram_idle\n");
		return;
	}
	_omap_sram_idle(context_mem, save_state);
	/* Restore table entry modified during MMU restoration */
	if (((PM_PREPWSTST_MPU & 0x7) == 0x0) ||
			((PM_PREPWSTST_MPU & 0x7) == 0x1)) {
		restore_table_entry();
	}
}

static int omap3_pm_prepare(void)
{
	int error = 0;
	/* We cannot sleep in idle until we have resumed */
	saved_idle = pm_idle;
	pm_idle = NULL;
	return error;
}

/* Save and retore global configuration in NEON domain */
void omap3_save_neon_context(void)
{
	/*Nothing to do here */
	return;
}

extern void vfp_enable(void);
void omap3_restore_neon_context(void)
{
#ifdef CONFIG_VFP
	vfp_enable();
#endif
}

/* Save and restore global configuration in PER domain */
void omap3_save_per_context(void)
{
	/* Only GPIO save is done here */
	omap_gpio_save();
	omap_uart_save_ctx(2);
}

void omap3_restore_per_context(void)
{
	/* Only GPIO restore is done here */
	omap_gpio_restore();
	prcm_wait_for_clock(PRCM_UART3);
	omap_uart_restore_ctx(2);
}

u32 omap3_save_secure_ram_context(u32 target_core_state)
{
	u32 ret = 0;
	if ((target_core_state  > PRCM_CORE_CSWR_MEMRET) &&
		(target_core_state != PRCM_CORE_OSWR_MEMRET)) {
		if (!is_device_type_gp()) {
			/* Disable dma irq before calling
			* secure rom code API
			*/
			omap_dma_disable_irq(0);
			omap_dma_disable_irq(1);
			ret = _omap_save_secure_sram((u32 *)__pa(sdram_mem));
		}
	}
	return ret;
}

#ifdef OMAP3_START_RNG
u32 omap3_start_rng(void)
{
	u32 ret = -1;
	if (!is_device_type_gp())
		ret = _omap_start_rng();
	return ret;
}
#endif /* #ifdef OMAP3_START_RNG */

void omap3_push_sram_functions(void)
{
	_omap_sram_idle = omap_sram_push(omap34xx_cpu_suspend,
				omap34xx_cpu_suspend_sz);
	if (!is_device_type_gp()) {
		_omap_save_secure_sram = omap_sram_push(save_secure_ram_context,
			save_secure_ram_context_sz);
#ifdef OMAP3_START_RNG
		_omap_start_rng = omap_sram_push(
			start_new_rng, start_new_rng_sz);
#endif
	}
}

/* Configuration that is OS specific is done here */
void omap3_restore_core_settings(void)
{
	prcm_lock_iva_dpll(current_vdd1_opp);
	restore_sram_functions();
	omap3_push_sram_functions();
	omap_uart_restore_ctx(0);
	omap_uart_restore_ctx(1);
}

static int omap3_pm_suspend(void)
{
	int ret;
	u32 prcm_state, prepwstst;

#ifdef CONFIG_OMAP_SMARTREFLEX
	disable_smartreflex(SR1_ID);
	disable_smartreflex(SR2_ID);
#endif

	local_irq_disable();
	local_fiq_disable();

	PM_PREPWSTST_MPU = 0xFF;
	PM_PREPWSTST_CORE = 0xFF;
	PM_PREPWSTST_NEON = 0xFF;
	PM_PREPWSTST_PER = 0xFF;
#ifdef CONFIG_CORE_OFF
	if (enable_off) {
		omap_uart_save_ctx(0);
		omap_uart_save_ctx(1);
	}
#endif
#ifdef CONFIG_MPU_OFF
	/* On ES 2.0, if scratchpad is populated with valid
	* pointer, warm reset does not work
	* So populate scratchpad restore address only in
	* cpuidle and suspend calls
	*/
	*(scratchpad_restore_addr) = restore_pointer_address;
#endif

#ifdef CONFIG_OMAP34XX_OFFMODE
	context_restore_update(DOM_PER);
	context_restore_update(DOM_CORE1);
	prcm_state = PRCM_OFF;
#else
	prcm_state = PRCM_RET;
#endif

	target_state.iva2_state = prcm_state;
	target_state.gfx_state = prcm_state;
	target_state.dss_state = prcm_state;
	target_state.cam_state = prcm_state;
	target_state.per_state = prcm_state;
	target_state.usbhost_state = prcm_state;
	target_state.neon_state = prcm_state;

#ifdef CONFIG_MPU_OFF
	if (enable_off)
		target_state.mpu_state = PRCM_MPU_OFF;
	else
		target_state.mpu_state = PRCM_MPU_CSWR_L2RET;
#else
	target_state.mpu_state = PRCM_MPU_CSWR_L2RET;
#endif

	if (target_state.neon_state == PRCM_OFF)
		omap3_save_neon_context();

	if (target_state.per_state == PRCM_OFF)
		omap3_save_per_context();

#ifdef CONFIG_CORE_OFF
	if (enable_off)
		target_state.core_state = PRCM_CORE_OFF;
	else
		target_state.core_state = PRCM_CORE_CSWR_MEMRET;
#else
	target_state.core_state = PRCM_CORE_CSWR_MEMRET;
#endif

	if (core_off_notification != NULL)
		core_off_notification(PRCM_TRUE);

	if (target_state.core_state >=  PRCM_CORE_OSWR_MEMRET) {
		prcm_save_core_context(target_state.core_state);
		omap_uart_save_ctx(0);
		omap_uart_save_ctx(1);
	}

	ret = prcm_set_chip_power_mode(&target_state, OMAP3_WAKEUP);

#ifdef CONFIG_MPU_OFF
	*(scratchpad_restore_addr) = 0;
#endif
	if (target_state.neon_state == PRCM_OFF)
		omap3_restore_neon_context();
	PM_PREPWSTST_NEON = 0xFF;

	if (target_state.per_state == PRCM_OFF)
		omap3_restore_per_context();
	PM_PREPWSTST_PER = 0xFF;
#ifdef CONFIG_CORE_OFF
	if (enable_off) {
		omap3_restore_core_settings();
	}
#else
	if (target_state.core_state >= PRCM_CORE_OSWR_MEMRET) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		context_restore_update(DOM_CORE1);
#endif
		prcm_restore_core_context(target_state.core_state);
		omap_uart_restore_ctx(0);
		omap_uart_restore_ctx(1);
	}
#if 0
	if ((target_state.core_state > PRCM_CORE_CSWR_MEMRET) &&
			(target_state.core_state != PRCM_CORE_OSWR_MEMRET)) {
			restore_sram_functions();
			omap3_push_sram_functions();
		}
#endif
#endif
	local_fiq_enable();
	local_irq_enable();

	if (core_off_notification != NULL)
		core_off_notification(PRCM_FALSE);

#ifdef OMAP3_START_RNG
	if (!is_device_type_gp()) {
		/* Start RNG after interrupts are enabled
		 * and only when CORE OFF was successful
		 */
		if (!(prepwst_core_rng & 0x3)) {
			ret = omap3_start_rng();
			if (ret)
				printk(KERN_INFO"Failed to generate"
						"new RN in susp %x\n", ret);
			prepwst_core_rng = 0xFF; /* Clearing the status */
		}
	}
#endif

	printk(KERN_INFO "\nSuspend/Resume Result: %s!!\n",
		(ret == PRCM_PASS) ? "SUCCESS" : "FAIL");

	prcm_state = target_state.mpu_state;
	prepwstst = PM_PREPWSTST_MPU & 0x3;
	printk(KERN_INFO "MPU state : TARGET-%4s PREPWST[0x%x]-%4s\n",
		(prcm_state == PRCM_MPU_OFF) ? "OFF" :
		(prcm_state >= PRCM_MPU_CSWR_L2RET) ? "RET" : "ON",
		PM_PREPWSTST_MPU,
		(prepwstst == 0x0) ? "OFF" :
		(prepwstst == 0x1) ? "RET" : "ON");

	prcm_state = target_state.core_state;
	prepwstst = PM_PREPWSTST_CORE & 0x3;
	printk(KERN_INFO "CORE state: TARGET-%4s PREPWST[0x%x]-%4s\n",
		(prcm_state == PRCM_CORE_OFF) ? "OFF" :
		(prcm_state >= PRCM_CORE_CSWR_MEMRET) ? "RET":"ON",
		PM_PREPWSTST_CORE,
		(prepwstst == 0x0) ? "OFF" :
		(prepwstst == 0x1) ? "RET" : "ON");

	PM_PREPWSTST_CORE = PM_PREPWSTST_CORE;
	PM_PREPWSTST_MPU = PM_PREPWSTST_MPU;

#ifdef CONFIG_OMAP_SMARTREFLEX
	enable_smartreflex(SR1_ID);
	enable_smartreflex(SR2_ID);
#endif

#ifdef DEBUG_PM_34XX_PADCONF
printk("PM_WKST3_CORE:     %08x\n", PM_WKST3_CORE);
printk("PM_PWSTST_USBHOST: %08x\n", PM_PWSTST_USBHOST);
printk("PM_PWSTST_USBHOST: %08x\n", PM_PWSTST_USBHOST);
printk("PM_PWSTST_PER:     %08x\n", PM_PWSTST_PER);
printk("PM_PWSTST_DSS:     %08x\n", PM_PWSTST_DSS);
printk("PM_WKST_WKUP:      %08x\n", PM_WKST_WKUP);

printk("WKST_WKUP:       %08x\n", save_wkst.WKST_WKUP);
printk("WKST_PER:        %08x\n", save_wkst.WKST_PER);
printk("WKST_USBHOST:    %08x\n", save_wkst.WKST_USBHOST);
printk("WKST1_CORE:      %08x\n", save_wkst.WKST1_CORE);
printk("WKST3_CORE:      %08x\n", save_wkst.WKST3_CORE);
printk("interrupt:       %08x\n", save_wkst.interrupt);

	control_padconf_dump();
#endif	
	return 0;
}

static int omap3_pm_enter(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap3_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int omap3_pm_valid(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = 1;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void omap3_pm_finish(void)
{
	pm_idle = saved_idle;
	return;
}

static struct platform_suspend_ops omap_pm_ops = {
	.valid		= omap3_pm_valid,
	.prepare	= omap3_pm_prepare,
	.enter		= omap3_pm_enter,
	.finish		= omap3_pm_finish,
};

/* PRCM Interrupt Handler */
irqreturn_t prcm_interrupt_handler (int irq, void *dev_id)
{
	u32 wkst_wkup = 0;
	u32 wkst1_core = 0;
	u32 wkst3_core = 0;
	u32 wkst_usbhost = 0;
	u32 wkst_per = 0;
	u32 errst_vc = PRM_VC_TIMEOUTERR_ST | PRM_VC_RAERR_ST |
							PRM_VC_SAERR_EN;
	u32 fclk = 0;
	u32 iclk = 0;

#ifdef DEBUG_PM_34XX_PADCONF
	save_wkst.WKST_WKUP = PM_WKST_WKUP;
	save_wkst.WKST_PER = PM_WKST_PER;
	save_wkst.WKST_USBHOST = PM_WKST_USBHOST;
	save_wkst.WKST1_CORE = PM_WKST1_CORE;
	save_wkst.WKST3_CORE = PM_WKST3_CORE;
	save_wkst.interrupt++;
	
	control_padconf_save();
#endif	

	do {
		if (PM_WKST_WKUP) {
			wkst_wkup = PM_WKST_WKUP;
#ifdef IOPAD_WKUP
			/*
			 * Resetting UART1 inactivity
			 * timeout during IO_PAD wakeup.
			 */
			if ((wkst_wkup & 0x100) && is_console_wakeup())
				awake_time_end = jiffies +
					 msecs_to_jiffies(UART_TIME_OUT);

#endif /* #ifdef IOPAD_WKUP */

			iclk = CM_ICLKEN_WKUP;
			fclk = CM_FCLKEN_WKUP;
			/*
			 * Make sure to read the WKST REG here. Might be a case
			 * where in WKST is set between the beginning of
			 * interrupt to till this point. Doing the same for all
			 * Power domains.
			 */
			while (PM_WKST_WKUP) {
				CM_ICLKEN_WKUP |= PM_WKST_WKUP;
				CM_FCLKEN_WKUP |= PM_WKST_WKUP;
				PM_WKST_WKUP = PM_WKST_WKUP;
			}
			CM_ICLKEN_WKUP = iclk;
			CM_FCLKEN_WKUP = fclk;
		}
		if (PM_WKST1_CORE) {
			wkst1_core = PM_WKST1_CORE;
			iclk = CM_ICLKEN1_CORE;
			fclk = CM_FCLKEN1_CORE;
			while (PM_WKST1_CORE) {
				CM_ICLKEN1_CORE |= PM_WKST1_CORE;
				CM_FCLKEN1_CORE |= PM_WKST1_CORE;
				PM_WKST1_CORE = PM_WKST1_CORE;
			}
			CM_ICLKEN1_CORE = iclk;
			CM_FCLKEN1_CORE = fclk;
		}
		if (PM_WKST3_CORE) {
			wkst3_core = PM_WKST3_CORE;
			iclk = CM_ICLKEN3_CORE;
			fclk = CM_FCLKEN3_CORE;
			while (PM_WKST3_CORE) {
				CM_ICLKEN3_CORE |= PM_WKST3_CORE;
				CM_FCLKEN3_CORE |= PM_WKST3_CORE;
				PM_WKST3_CORE = PM_WKST3_CORE;
			}
			CM_ICLKEN3_CORE = iclk;
			CM_FCLKEN3_CORE = fclk;
		}
		if (PM_WKST_USBHOST) {
			wkst_usbhost = PM_WKST_USBHOST;
			iclk = CM_ICLKEN_USBHOST;
			fclk = CM_FCLKEN_USBHOST;
			while (PM_WKST_USBHOST) {
				CM_ICLKEN_USBHOST |= PM_WKST_USBHOST;
				CM_FCLKEN_USBHOST |= PM_WKST_USBHOST;
				PM_WKST_USBHOST = PM_WKST_USBHOST;
			}
			CM_ICLKEN_USBHOST = iclk;
			CM_FCLKEN_USBHOST = fclk;
		}
		if (PM_WKST_PER) {
			wkst_per = PM_WKST_PER;
			iclk = CM_ICLKEN_PER;
			fclk = CM_FCLKEN_PER;
			while (PM_WKST_PER) {
				CM_ICLKEN_PER |= PM_WKST_PER;
				CM_FCLKEN_PER |= PM_WKST_PER;
				PM_WKST_PER = PM_WKST_PER;
			}
			CM_ICLKEN_PER = iclk;
			CM_FCLKEN_PER = fclk;
		}

		if (!(wkst_wkup | wkst1_core | wkst3_core | wkst_usbhost |
								wkst_per)) {
			if (!(PRM_IRQSTATUS_MPU & errst_vc)) {
				printk(KERN_ERR "%x,%x,%x,%x\n",
					PRM_IRQSTATUS_MPU, wkst_wkup,
					wkst1_core, wkst_per);
				printk(KERN_ERR "Spurious PRCM interrupt\n");
			}
		}

#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
		if (PRM_IRQSTATUS_MPU & PRM_VC_TIMEOUTERR_ST)
			printk(KERN_ERR "PRCM : Voltage Controller timeout\n");
		if (PRM_IRQSTATUS_MPU & PRM_VC_RAERR_ST)
			printk(KERN_ERR "PRCM : Voltage Controller register "
						"address acknowledge error\n");
		if (PRM_IRQSTATUS_MPU & PRM_VC_SAERR_ST)
			printk(KERN_ERR "PRCM : Voltage Controller slave "
						"address acknowledge error\n");
#endif /* #ifdef CONFIG_OMAP_VOLT_SR_BYPASS */

		if (PRM_IRQSTATUS_MPU)
			PRM_IRQSTATUS_MPU |= 0x3;

	} while (PRM_IRQSTATUS_MPU);

	return IRQ_HANDLED;
}

int __init omap3_pm_init(void)
{
	int ret;
	printk(KERN_ERR "Power Management for TI OMAP.\n");

	/* Setup the delay for power supply setup delay*/
	/* should be enough since power supply spec gives 750 µs */
	/* VOLTOFFSET is delay from wakeup event to sys_off_mode toggling*/
 	PRM_VOLTOFFSET = 0x41;
	/* VOLTSETUP2 is delay between sys_off_mode toggling and end of the clock gating*/
	/* After these delays, the clock is propagated to the full chip*/
 	PRM_VOLTSETUP2 = 0x64;
	/* PRM_CLKSETUP=PRM_VOLTOFFSET+PRM_VOLTSETUP2*/
	PRM_CLKSETUP = PRM_VOLTOFFSET + PRM_VOLTSETUP2;

	omap3_push_sram_functions();
	suspend_set_ops(&omap_pm_ops);

	/* In case of cold boot, clear scratchpad */
	if (RM_RSTST_CORE & 0x1)
		clear_scratchpad_contents();
#ifdef CONFIG_MPU_OFF
	save_scratchpad_contents();
#endif
	if (!is_device_type_gp()) {
		sdram_mem = kmalloc(sizeof(struct sram_mem), GFP_KERNEL);
		if (!sdram_mem)
			printk(KERN_ERR "Memory allocation failed when"
				"allocating for secure sram context");
	}
	memret1 = (struct res_handle *)resource_get("corememresret1",
							"core_mem1ret");
	memret2 = (struct res_handle *)resource_get("corememresret2",
							"core_mem2ret");
	logret1 = (struct res_handle *)resource_get("corelogicret",
							"core_logicret");

	ret = request_irq(PRCM_MPU_IRQ, (irq_handler_t)prcm_interrupt_handler,
				IRQF_DISABLED, "prcm", NULL);
	if (ret) {
		printk(KERN_ERR "request_irq failed to register for 0x%x\n",
			PRCM_MPU_IRQ);
		return -1;
	}

	/* Adjust the system OPP based during bootup using
	* KConfig option
	*/
	if (p_vdd1_clk == NULL) {
		p_vdd1_clk = clk_get(NULL, "virt_vdd1_prcm_set");
		if (p_vdd1_clk == NULL) {
			printk(KERN_ERR "Unable to get the VDD1 clk\n");
			return -1;
		}
	}

	if (p_vdd2_clk == NULL) {
		p_vdd2_clk = clk_get(NULL, "virt_vdd2_prcm_set");
		if (p_vdd2_clk == NULL) {
			printk(KERN_ERR "Unable to get the VDD2 clk\n");
			return -1;
		}
	}

#ifdef CONFIG_MPU_OFF
	clear_scratchpad_contents();
	save_scratchpad_contents();
#endif
	PRM_IRQSTATUS_MPU = 0x3FFFFFD;
	PRM_IRQENABLE_MPU = 0x1;
#ifdef IOPAD_WKUP
	/* Enabling the IO_PAD PRCM interrupts */
	PRM_IRQENABLE_MPU |= 0x200;
#endif /* #ifdef IOPAD_WKUP */

#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
	/* Enabling the VOLTAGE CONTROLLER PRCM interrupts */
	PRM_IRQENABLE_MPU |= PRM_VC_TIMEOUTERR_EN | PRM_VC_RAERR_EN|
				PRM_VC_SAERR_EN;
#endif /* #ifdef CONFIG_OMAP_VOLT_SR_BYPASS */
	/* omap3_pm_sysfs_init();*/
	return 0;
}

/* Clears the scratchpad contents in case of cold boot- called during bootup*/
void clear_scratchpad_contents(void)
{
	u32 max_offset = SCRATCHPAD_ROM_OFFSET;
	u32 offset = 0;
	u32 v_addr = io_p2v(SCRATCHPAD_ROM);

	/* Check if it is a cold reboot */
	if ((PRM_RSTST & 0x1) && !is_sil_rev_equal_to(OMAP3430_REV_ES3_0)) {
		for ( ; offset <= max_offset; offset += 0x4)
			__raw_writel(0x0, (v_addr + offset));
		PRM_RSTST |= 0x1;
	}
}

void save_to_scratchpad(u32 offset, u32 value)
{
	u32 *scratchpad_address;

	scratchpad_address = (u32 *) io_p2v(SCRATCHPAD);
	*(scratchpad_address + offset) = value;
}

/* Populate the scratchpad structure with restore structure */
void save_scratchpad_contents(void)
{
	volatile u32 *scratchpad_address;
	u32 *restore_address;
	u32 *sdram_context_address;
	/* Get virtual address of SCRATCHPAD */
	scratchpad_address = (u32 *) io_p2v(SCRATCHPAD);
	/* Get Restore pointer to jump to while waking up from OFF */
	if (is_sil_rev_less_than(OMAP3430_REV_ES3_0))
		restore_address = get_restore_pointer();
	else
		restore_address = get_es3_restore_pointer();
	/* Convert it to physical address */
	restore_address = (u32 *) virt_to_phys(restore_address);
	/* Get address where registers are saved in SDRAM */
	sdram_context_address = (u32 *) virt_to_phys(context_mem);
	/* Booting configuration pointer*/
	*(scratchpad_address++) = 0x0;
	/* Public restore pointer */
	/* On ES 2.0, if scratchpad is populated with valid
	* pointer, warm reset does not work
	* So populate scratchpad restore address only in
	* cpuidle and suspend calls
	*/
	scratchpad_restore_addr = scratchpad_address;
	restore_pointer_address = (u32) restore_address;
	*(scratchpad_address++) = restore_pointer_address;
	/* Secure ram restore pointer */
	if (is_device_type_gp())
		*(scratchpad_address++) = 0x0;
	else
		*(scratchpad_address++) = (u32) __pa(sdram_mem);
	/* SDRC Module semaphore */
	*(scratchpad_address++);
	/* PRCM Block Offset */
	*(scratchpad_address++) = 0x2C;
	/* SDRC Block Offset */
	*(scratchpad_address++) = 0x64;
	/* Empty */
	/* Offset 0x8*/
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	/* Offset 0xC*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x10*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x14*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x18*/
	/* PRCM Block */
	*(scratchpad_address++) = PRM_CLKSRC_CTRL;
	*(scratchpad_address++) = PRM_CLKSEL;
	*(scratchpad_address++) = CM_CLKSEL_CORE;
	*(scratchpad_address++) = CM_CLKSEL_WKUP;
	*(scratchpad_address++) = CM_CLKEN_PLL;
	*(scratchpad_address++) = CM_AUTOIDLE_PLL;
	*(scratchpad_address++) = CM_CLKSEL1_PLL;
	*(scratchpad_address++) = CM_CLKSEL2_PLL;
	*(scratchpad_address++) = CM_CLKSEL3_PLL;
	*(scratchpad_address++) = CM_CLKEN_PLL_MPU;
	*(scratchpad_address++) = CM_AUTOIDLE_PLL_MPU;
	*(scratchpad_address++) = CM_CLKSEL1_PLL_MPU;
	*(scratchpad_address++) = CM_CLKSEL2_PLL_MPU;
	*(scratchpad_address++) = 0x0;
	/* SDRC Block */
	*(scratchpad_address++) = ((sdrc_read_reg(SDRC_CS_CFG) &
					0xFFFF) << 16) |
					(sdrc_read_reg(SDRC_SYSCONFIG) &
					0xFFFF);
	*(scratchpad_address++) = ((sdrc_read_reg(SDRC_ERR_TYPE) &
					0xFFFF) << 16) |
					(sdrc_read_reg(SDRC_SHARING) &
					0xFFFF);
	*(scratchpad_address++) = sdrc_read_reg(SDRC_DLLA_CTRL);
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = sdrc_read_reg(SDRC_POWER);
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = sdrc_read_reg(SDRC_MCFG_0);
	*(scratchpad_address++) = sdrc_read_reg(SDRC_MR_0) & 0xFFFF;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = sdrc_read_reg(SDRC_ACTIM_CTRL_A_0);
	*(scratchpad_address++) = sdrc_read_reg(SDRC_ACTIM_CTRL_B_0);
	*(scratchpad_address++) = sdrc_read_reg(SDRC_RFR_CTRL_0);
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = sdrc_read_reg(SDRC_MCFG_1);
	*(scratchpad_address++) = sdrc_read_reg(SDRC_MR_1) & 0xFFFF;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = sdrc_read_reg(SDRC_ACTIM_CTRL_A_1);
	*(scratchpad_address++) = sdrc_read_reg(SDRC_ACTIM_CTRL_B_1);
	*(scratchpad_address++) = sdrc_read_reg(SDRC_RFR_CTRL_1);
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = (u32) sdram_context_address;
}

__initcall(omap3_pm_init);
