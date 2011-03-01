/*
 * linux/arch/arm/mach-omap3/cpuidle_34xx.c
 *
 * OMAP3 CPU IDLE Routines
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
 * Based on pm.c for omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <linux/jiffies.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/console.h>
#include <mach/pm.h>
#include <mach/prcm_34xx.h>
#include <mach/dma.h>
#include <mach/sdrc.h>
#include "sdrc.h"
#include "prcm-regs.h"
#include "pm_idle_34xx.h"
#include "ti-compat.h"

/* #define DEBUG_HW_SUP 1 */

#ifdef CONFIG_ENABLE_SWLATENCY_MEASURE
extern u32 wakeup_start_32ksync;
#endif /* #ifdef CONFIG_ENABLE_SWLATENCY_MEASURE */

struct system_power_state target_idlestate;
struct omap3_processor_cx omap3_power_states[OMAP3_MAX_STATES];

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define LAST_IDLE_STATE_ARR_SIZE 10
static u32 mpu_oswr_cnt;
static u32 mpu_cswr_cnt;
static u32 mpu_off_cnt;
static u32 core_oswr_cnt;
static u32 core_cswr_cnt;
static u32 core_off_cnt;
static u32 per_off;

struct idle_state {
	u32 mpu_state;
	u32 core_state;
	u32 fclks;
	u32 iclks;
	u8 iva_state;
};

static struct idle_state last_idle_state[LAST_IDLE_STATE_ARR_SIZE];
u32 arr_wrptr;

#ifdef CONFIG_ENABLE_SWLATENCY_MEASURE

#define SW_LATENCY_ARR_SIZE 500
struct sw_latency {
	u32 cstate;
	u32 sleep_start;
	u32 sleep_end;
	u32 wkup_start;
	u32 wkup_sw;
	u32 wkup_end;
};
static struct sw_latency sw_latency_arr[SW_LATENCY_ARR_SIZE];
u32 swlat_arr_wrptr;
#endif /* #ifdef CONFIG_ENABLE_SWLATENCY_MEASURE */

static void *omap_pm_prepwst_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *omap_pm_prepwst_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void omap_pm_prepwst_stop(struct seq_file *m, void *v)
{
}

char *pstates [4] =	{
			"OFF",
			"RET",
			"INACT",
			"ON"
		};

int omap_pm_prepwst_show(struct seq_file *m, void *v)
{
	u32 arr_rdptr, arr_cnt;

	if (!enable_debug) {
		printk(KERN_ERR "\nPlease enable debug using"
					"/sys/power/enable_debug\n");
		return 0;
	}
#ifdef CONFIG_ENABLE_SWLATENCY_MEASURE
	arr_rdptr = swlat_arr_wrptr;
	arr_cnt = SW_LATENCY_ARR_SIZE;

	seq_printf(m, "Sleep and wakeup latency using 32k sync timer.\n");
	seq_printf(m, "state, sleep_start, sleep_end, wkup_start,\
							 wkup_end\n\n");
	while (arr_cnt--) {
		if (arr_rdptr == 0)
			arr_rdptr = LAST_IDLE_STATE_ARR_SIZE;

		arr_rdptr--;

		seq_printf(m, "%X,%u,%u,%u,%u\n",
			sw_latency_arr[arr_rdptr].cstate,
			sw_latency_arr[arr_rdptr].sleep_start,
			sw_latency_arr[arr_rdptr].sleep_end,
			sw_latency_arr[arr_rdptr].wkup_start,
			sw_latency_arr[arr_rdptr].wkup_end);
	}
#endif /* #ifdef CONFIG_ENABLE_SWLATENCY_MEASURE */

	seq_printf(m, "Previous power state for MPU +CORE\n");

	arr_rdptr = arr_wrptr;
	arr_cnt = LAST_IDLE_STATE_ARR_SIZE;

#ifdef DEBUG_HW_SUP
	seq_printf(m, "PM_PWSTST_CORE %x\n", PM_PWSTST_CORE);
	seq_printf(m, "PM_PREPWSTST_CORE %x\n", PM_PREPWSTST_CORE);
	seq_printf(m, "CM_CLKSTCTRL_CORE %x\n\n", CM_CLKSTCTRL_CORE);

	seq_printf(m, "PM_PWSTST_IVA2 %x\n", PM_PWSTST_IVA2);
	seq_printf(m, "PM_PREPWSTST_IVA2 %x\n", PM_PREPWSTST_IVA2);
	seq_printf(m, "CM_CLKSTCTRL_IVA2 %x\n", CM_CLKSTCTRL_IVA2);
	seq_printf(m, "CM_CLKSTST_IVA2 %x\n\n", CM_CLKSTST_IVA2);

	seq_printf(m, "PM_PWSTST_NEON %x\n", PM_PWSTST_NEON);
	seq_printf(m, "PM_PREPWSTST_NEON %x\n", PM_PREPWSTST_NEON);
	seq_printf(m, "CM_CLKSTCTRL_NEON %x\n\n", CM_CLKSTCTRL_NEON);

	seq_printf(m, "PM_PWSTST_PER %x\n", PM_PWSTST_PER);
	seq_printf(m, "PM_PREPWSTST_PER %x\n", PM_PREPWSTST_PER);
	seq_printf(m, "CM_CLKSTCTRL_PER %x\n", CM_CLKSTCTRL_PER);
	seq_printf(m, "CM_CLKSTST_PER %x\n\n", CM_CLKSTST_PER);

	seq_printf(m, "PM_PWSTST_DSS %x\n", PM_PWSTST_DSS);
	seq_printf(m, "PM_PREPWSTST_DSS %x\n", PM_PREPWSTST_DSS);
	seq_printf(m, "CM_CLKSTCTRL_DSS %x\n", CM_CLKSTCTRL_DSS);
	seq_printf(m, "CM_CLKSTST_DSS %x\n\n", CM_CLKSTST_DSS);

	seq_printf(m, "PM_PWSTST_USBHOST %x\n", PM_PWSTST_USBHOST);
	seq_printf(m, "PM_PREPWSTST_USBHOST %x\n", PM_PREPWSTST_USBHOST);
	seq_printf(m, "CM_CLKSTCTRL_USBHOST %x\n", CM_CLKSTCTRL_USBHOST);
	seq_printf(m, "CM_CLKSTST_USBHOST %x\n\n", CM_CLKSTST_USBHOST);

	seq_printf(m, "PM_PWSTST_SGX %x\n", PM_PWSTST_SGX);
	seq_printf(m, "PM_PREPWSTST_SGX %x\n", PM_PREPWSTST_SGX);
	seq_printf(m, "CM_CLKSTCTRL_SGX %x\n", CM_CLKSTCTRL_SGX);
	seq_printf(m, "CM_CLKSTST_SGX %x\n\n", CM_CLKSTST_SGX);

	seq_printf(m, "PM_PWSTST_CAM %x\n", PM_PWSTST_CAM);
	seq_printf(m, "PM_PREPWSTST_CAM %x\n", PM_PREPWSTST_CAM);
	seq_printf(m, "CM_CLKSTCTRL_CAM %x\n", CM_CLKSTCTRL_CAM);
	seq_printf(m, "CM_CLKSTST_CAM %x\n\n", CM_CLKSTST_CAM);
#endif /* #ifdef DEBUG_HW_SUP */

	while (arr_cnt--) {
		if (arr_rdptr == 0)
			arr_rdptr = LAST_IDLE_STATE_ARR_SIZE;

		arr_rdptr--;
		seq_printf(m, "MPU = %x - %s,  CORE = %x - %s, IVA = %x - %s,"
				"fclks: %x, iclks: %x\n",
				last_idle_state[arr_wrptr].mpu_state,
				pstates [last_idle_state[arr_wrptr].mpu_state &
				0x3], last_idle_state[arr_wrptr].core_state,
				pstates [last_idle_state[arr_wrptr].core_state &
				0x3], last_idle_state[arr_wrptr].iva_state,
				pstates [last_idle_state[arr_wrptr].iva_state &
				0x3], last_idle_state[arr_wrptr].fclks,
				last_idle_state[arr_wrptr].iclks
		);
	}
	seq_printf(m, "\nMPU-CSWR=%d, MPU-OSWR=%d, MPU-OFF=%d, PER_OFF=%d,"
		" CORE-CSWR=%d, CORE-OSWR=%d, CORE-OFF=%d\n\n",
		mpu_cswr_cnt, mpu_oswr_cnt, mpu_off_cnt, per_off, core_cswr_cnt,
		core_oswr_cnt, core_off_cnt);

	return 0;
}

static struct seq_operations omap_pm_prepwst_op = {
	.start = omap_pm_prepwst_start,
	.next  = omap_pm_prepwst_next,
	.stop  = omap_pm_prepwst_stop,
	.show  = omap_pm_prepwst_show
};

static int omap_pm_prepwst_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &omap_pm_prepwst_op);
}

static struct file_operations proc_pm_prepwst_ops = {
	.open	   = omap_pm_prepwst_open,
	.read	   = seq_read,
	.llseek	 = seq_lseek,
	.release	= seq_release,
};

/* This API creates a proc entry for shared resources. */
int create_pmproc_entry(void)
{
	struct proc_dir_entry *entry;

	/* Create a proc entry for shared resources */
	entry = create_proc_entry("pm_prepwst", 0, NULL);
	if (entry) {
		entry->proc_fops = &proc_pm_prepwst_ops;
		printk(KERN_ERR "create_proc_entry succeeded\n");
	} else
		printk(KERN_ERR "create_proc_entry failed\n");

	return 0;
}

static void store_prepwst(void)
{
	if (!enable_debug)
		return;

	last_idle_state[arr_wrptr].mpu_state = PM_PREPWSTST_MPU;
	last_idle_state[arr_wrptr].core_state = PM_PREPWSTST_CORE;

	if ((PM_PREPWSTST_MPU & 0x7) == 0x1)
		mpu_oswr_cnt++;
	if ((PM_PREPWSTST_MPU & 0x7) == 0x5)
		mpu_cswr_cnt++;
	else if ((PM_PREPWSTST_MPU & 0x7) == 0x0)
		mpu_off_cnt++;

	if ((PM_PREPWSTST_CORE & 0x7) == 0x1)
		core_oswr_cnt++;
	if ((PM_PREPWSTST_CORE & 0x7) == 0x5)
		core_cswr_cnt++;
	else if ((PM_PREPWSTST_CORE & 0x7) == 0x0)
		core_off_cnt++;

	last_idle_state[arr_wrptr].fclks =
		((CM_FCLKEN1_CORE & CORE_FCLK_MASK) |
		(CM_FCLKEN_SGX & SGX_FCLK_MASK) |
		(CM_FCLKEN_DSS & DSS_FCLK_MASK) |
		(CM_FCLKEN_USBHOST & USBHOST_FCLK_MASK) |
		(CM_FCLKEN3_CORE & CORE3_FCLK_MASK)
		| (CM_FCLKEN_CAM & CAM_FCLK_MASK) |
		(CM_FCLKEN_PER & PER_FCLK_MASK));

	last_idle_state[arr_wrptr].iclks =
		(CORE1_ICLK_VALID &
		(CM_ICLKEN1_CORE & ~CM_AUTOIDLE1_CORE)) |
		(CORE2_ICLK_VALID & (CM_ICLKEN2_CORE & ~CM_AUTOIDLE2_CORE)) |
		(CORE3_ICLK_VALID & (CM_ICLKEN3_CORE & ~CM_AUTOIDLE3_CORE)) |
		(SGX_ICLK_VALID & (CM_ICLKEN_SGX)) |
		(USBHOST_ICLK_VALID &
			(CM_ICLKEN_USBHOST & ~CM_AUTOIDLE_USBHOST)) |
		(DSS_ICLK_VALID & (CM_ICLKEN_DSS & ~CM_AUTOIDLE_DSS)) |
		(CAM_ICLK_VALID & (CM_ICLKEN_CAM & ~CM_AUTOIDLE_CAM)) |
		(PER_ICLK_VALID & (CM_ICLKEN_PER & ~CM_AUTOIDLE_PER)) |
		(WKUP_ICLK_VALID & (CM_ICLKEN_WKUP & ~CM_AUTOIDLE_WKUP));

	prcm_get_power_domain_state(DOM_IVA2,
				&(last_idle_state[arr_wrptr].iva_state));
	arr_wrptr++;

	if (arr_wrptr == LAST_IDLE_STATE_ARR_SIZE)
		arr_wrptr = 0;
}
#endif /*#ifdef CONFIG_PROC_FS */

static int pre_uart_inactivity(void)
{
	int driveruart_managed = 0;
	if (are_driveromap_uarts_active(&driveruart_managed)) {
		awake_time_end = jiffies + msecs_to_jiffies(UART_TIME_OUT);
		return 0;
	}

	if (time_before(jiffies, awake_time_end))
		return 0;
	return 1;
}

static void post_uart_inactivity(void)
{
	if (awake_time_end == 0)
		awake_time_end = jiffies;
}

static int omap3_idle_bm_check(void)
{
	u32 core_dep = 0;
	u8 state;

	do {
	/* Check if any modules other than debug uart and gpios are active*/
	core_dep = (CM_FCLKEN1_CORE & CORE_FCLK_MASK) |
			(CM_FCLKEN_SGX & SGX_FCLK_MASK) |
		(CM_FCLKEN_CAM & CAM_FCLK_MASK) | (CM_FCLKEN_PER &
							PER_FCLK_MASK) |
		(CM_FCLKEN_USBHOST & USBHOST_FCLK_MASK) |
		(CM_FCLKEN3_CORE & CORE3_FCLK_MASK);
	/* To allow core retention during LPR scenario */
#ifdef CONFIG_OMAP_DISPLAY
	if (!lpr_enabled)
		core_dep |= (CM_FCLKEN_DSS & DSS_FCLK_MASK);
#endif

	if (core_dep)
		break;

	/* Check if any modules have ICLK bit enabled and interface clock */
	/* autoidle disabled						*/
	core_dep |= (CORE1_ICLK_VALID & (CM_ICLKEN1_CORE & ~CM_AUTOIDLE1_CORE));
	/* Check for secure modules which have only ICLK */
	/* Do not check for rng module.It has been ensured that
	 * if rng is active cpu idle will never be entered
	*/
	core_dep |= (CORE2_ICLK_VALID & CM_ICLKEN2_CORE & ~4);

	if (core_dep)
		break;

	/* Enabling SGX ICLK will prevent CORE ret*/
	core_dep |= SGX_ICLK_VALID & (CM_ICLKEN_SGX);
	core_dep |= (CORE3_ICLK_VALID & (CM_ICLKEN3_CORE & ~CM_AUTOIDLE3_CORE));
	core_dep |= (USBHOST_ICLK_VALID & (CM_ICLKEN_USBHOST &
			~CM_AUTOIDLE_USBHOST));
	core_dep |= (DSS_ICLK_VALID & (CM_ICLKEN_DSS & ~CM_AUTOIDLE_DSS));
	core_dep |= (CAM_ICLK_VALID & (CM_ICLKEN_CAM & ~CM_AUTOIDLE_CAM));
	core_dep |= (PER_ICLK_VALID & (CM_ICLKEN_PER & ~CM_AUTOIDLE_PER));
	core_dep |= (WKUP_ICLK_VALID & (CM_ICLKEN_WKUP & ~CM_AUTOIDLE_WKUP));

	if (core_dep)
		break;

	/* Check if IVA power domain is ON */
	prcm_get_power_domain_state(DOM_IVA2, &state);

	if (state == PRCM_ON) {
		core_dep |= 1;
		break;
	}

	/* Check if a DMA transfer is active */
	if (omap_dma_running()) {
		core_dep |= 1;
		break;
	}

	/* Check if debug UART is active */
	if (!pre_uart_inactivity())
		core_dep |= 1;

	} while (0);

	return core_dep;
}

/* Correct target state based on inactivity timer expiry, etc */
static void correct_target_state(void)
{
	switch (target_state.mpu_state) {
	case PRCM_MPU_ACTIVE:
	case PRCM_MPU_INACTIVE:
		target_state.neon_state = PRCM_ON;
		break;
	case PRCM_MPU_CSWR_L2RET:
	case PRCM_MPU_OSWR_L2RET:
	case PRCM_MPU_CSWR_L2OFF:
	case PRCM_MPU_OSWR_L2OFF:
		target_state.neon_state = PRCM_RET;
		break;
	case PRCM_MPU_OFF:
		target_state.neon_state = PRCM_OFF;
		if (!enable_off) {
			target_state.mpu_state = PRCM_MPU_CSWR_L2RET;
			target_state.neon_state = PRCM_RET;
		}
		break;
	}

	if (target_state.core_state > PRCM_CORE_INACTIVE) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		/* Core can be put to RET/OFF - This means
		 * PER can be put to off if its inactivity timer
		 * has expired
		 */
		if (perdomain_timer_pending())
			target_state.per_state = PRCM_RET;
		else
			target_state.per_state = PRCM_OFF;

		if (target_state.core_state == PRCM_CORE_OFF) {
			if (coredomain_timer_pending())
				target_state.core_state = PRCM_CORE_CSWR_MEMRET;
			if (CM_FCLKEN_DSS & DSS_FCLK_MASK)
				target_state.core_state = PRCM_CORE_CSWR_MEMRET;
			if (!enable_off) {
				target_state.core_state = PRCM_CORE_CSWR_MEMRET;
				target_state.per_state = PRCM_RET;
			}
		}
#else
		target_state.per_state = PRCM_RET;
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
	} else
		target_state.per_state = PRCM_ON;
}

static int omap3_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
	struct omap3_processor_cx *cx;
	u8 cur_per_state, cur_neon_state, pre_neon_state, pre_per_state;
	struct timespec ts_preidle, ts_postidle, ts_idle;
	u32 fclken_core, iclken_core, fclken_per, iclken_per;
	u32 sdrcpwr_val, sdrc_power_register = 0x0;
	int wakeup_latency;
	int core_sleep_flg = 0;
	u32 per_ctx_saved = 0;
	int ret = -1;
#ifdef CONFIG_ENABLE_SWLATENCY_MEASURE
	int idle_status = 0;
#endif

	local_irq_disable();
	local_fiq_disable();

	if (need_resched()) {
		local_irq_enable();
		local_fiq_enable();
		return 0;
	}

#ifdef CONFIG_ENABLE_SWLATENCY_MEASURE
	sw_latency_arr[swlat_arr_wrptr].sleep_start =
				 omap_32k_sync_timer_read();
#endif
	PM_PREPWSTST_MPU = 0xFF;
	PM_PREPWSTST_CORE = 0xFF;
	PM_PREPWSTST_NEON = 0xFF;
	PM_PREPWSTST_PER = 0xFF;

	cx = cpuidle_get_statedata(state);

	target_state.mpu_state = cx->mpu_state;
	target_state.core_state = cx->core_state;

	/* take a time marker for residency */
	getnstimeofday(&ts_preidle);

	if (cx->type == OMAP3_STATE_C0) {
		omap_sram_idle();
		goto return_sleep_time;
	}

	if (cx->type > OMAP3_STATE_C1)
		sched_clock_idle_sleep_event(); /* about to enter deep idle */

	correct_target_state();
	wakeup_latency = cx->wakeup_latency;
	if (target_state.core_state != cx->core_state) {
		/* Currently, this can happen only for core_off */
		/* Adjust wakeup latency to that of core_cswr state */
		/* Hard coded now and needs to be made more generic */
		/* omap3_power_states[4] is CSWR for core */
		wakeup_latency = omap3_power_states[4].wakeup_latency;
	}

	/* Reprogram next wake up tick to adjust for wake latency */
	if (wakeup_latency > 1000) {
		struct tick_device *d = tick_get_device(smp_processor_id());
		ktime_t adjust, next, now = ktime_get();
		if (ktime_to_ns(ktime_sub(d->evtdev->next_event, now)) >
			(wakeup_latency * 1000 + NSEC_PER_MSEC)) {
			adjust = ktime_set(0, (wakeup_latency * 1000));
			next = ktime_sub(d->evtdev->next_event, adjust);
			clockevents_program_event(d->evtdev, next, now);
		}
	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2)
		goto return_sleep_time;

	prcm_get_power_domain_state(DOM_PER, &cur_per_state);
	prcm_get_power_domain_state(DOM_NEON, &cur_neon_state);

	fclken_core = CM_FCLKEN1_CORE;
	iclken_core = CM_ICLKEN1_CORE;
	fclken_per = CM_FCLKEN_PER;
	iclken_per = CM_ICLKEN_PER;

	/* If target state if core_off, save registers
	 * before changing anything
	 */
	if (target_state.core_state >= PRCM_CORE_OSWR_MEMRET) {
		prcm_save_registers(&target_state);
		omap_uart_save_ctx(0);
		omap_uart_save_ctx(1);
	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2)
		goto return_sleep_time;

	/* Program MPU and NEON to target state */
	if (target_state.mpu_state > PRCM_MPU_ACTIVE) {
		if ((cur_neon_state == PRCM_ON) &&
			(target_state.neon_state != PRCM_ON)) {

		if (target_state.neon_state == PRCM_OFF)
			omap3_save_neon_context();

#ifdef CONFIG_HW_SUP_TRANS
		/* Facilitating SWSUP RET, from HWSUP mode */
			prcm_set_clock_domain_state(DOM_NEON,
					PRCM_NO_AUTO, PRCM_FALSE);
			prcm_set_power_domain_state(DOM_NEON, PRCM_ON,
						PRCM_FORCE);
#endif
			prcm_force_power_domain_state(DOM_NEON,
				target_state.neon_state);
		}
#ifdef CONFIG_MPU_OFF
		/* Populate scrathpad restore address */
		*(scratchpad_restore_addr) = restore_pointer_address;
#endif
		if (target_state.core_state > PRCM_CORE_CSWR_MEMRET) {
			ret = omap3_save_secure_ram_context(
						target_state.core_state);
			if (ret)
				printk(KERN_ERR "omap3_save_secure_ram_context"
						"failed in idle %x\n", ret);
			if (core_off_notification != NULL)
				core_off_notification(PRCM_TRUE);
		}
		prcm_set_mpu_domain_state(target_state.mpu_state);
	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2)
		goto restore;

	/* Program CORE and PER to target state */
	if (target_state.core_state > PRCM_CORE_ACTIVE) {
		/* Log core sleep attmept */
		core_sleep_flg = 1;


#ifdef CONFIG_OMAP_SMARTREFLEX
		disable_smartreflex(SR1_ID);
		disable_smartreflex(SR2_ID);
#endif
		/* Workaround for Silicon Errata 1.64 */
		if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
			if (CM_CLKOUT_CTRL & 0x80)
				CM_CLKOUT_CTRL &= ~(0x80);
		}

		prcm_set_core_domain_state(target_state.core_state);
		/* Enable Autoidle for GPT1 explicitly - Errata 1.4 */
		CM_AUTOIDLE_WKUP |= 0x1;
		/* Disable UART-1,2 */
		CM_FCLKEN1_CORE &= ~0x6000;
		/* Disable HSUSB OTG ICLK explicitly*/
		CM_ICLKEN1_CORE &= ~0x10;
		/* Enabling IO_PAD capabilities */
		PM_WKEN_WKUP |= 0x100;
		if (cur_per_state == PRCM_ON && cx->type >= OMAP3_STATE_C3 &&
					!(CM_FCLKEN_PER & PER_FCLK_MASK)) {
			/* In ES3.1, Enable IO Daisy chain */
			if (is_sil_rev_greater_than(OMAP3430_REV_ES3_0)) {
				PM_WKEN_WKUP |= 0x10000;
				/* Wait for daisy chain to be ready */
				while ((PM_WKST_WKUP & 0x10000) == 0x0)
					;
				/* clear the status */
				PM_WKST_WKUP &= ~0x10000;
			}
			omap3_save_per_context();
			prcm_set_power_domain_state(DOM_PER, PRCM_OFF,
							 PRCM_AUTO);
			per_ctx_saved = 1;
			CM_FCLKEN_PER = 0;
			CM_ICLKEN_PER = 0;
		}
	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2)
		goto restore;

	if (target_state.core_state == PRCM_CORE_OFF) {
		if (!is_device_type_gp() &&
				is_sil_rev_greater_than(OMAP3430_REV_ES2_1)) {

			/* es3 series bug */
			sdrc_power_register = sdrc_read_reg(SDRC_POWER);
			sdrcpwr_val = sdrc_power_register &
						~(SDRC_PWR_AUTOCOUNT_MASK |
						SDRC_PWR_CLKCTRL_MASK);
			lock_scratchpad_sem();
			sdrcpwr_val |= 0x120;
			save_to_scratchpad(SCRATHPAD_SDRCPWR_OFFSET,
							sdrcpwr_val);
			unlock_scratchpad_sem();
		}
	}

#ifdef CONFIG_ENABLE_SWLATENCY_MEASURE
	sw_latency_arr[swlat_arr_wrptr].sleep_end = omap_32k_sync_timer_read();
	idle_status++;
#endif
	omap_sram_idle();

	if (target_state.core_state == PRCM_CORE_OFF) {
		if (!is_device_type_gp() &&
				is_sil_rev_greater_than(OMAP3430_REV_ES2_1))
		sdrc_write_reg(sdrc_power_register, SDRC_POWER);
	}

restore:
	/* In case of ES3.1, disable IO daisy chain */
	if (is_sil_rev_greater_than(OMAP3430_REV_ES3_0) && per_ctx_saved)
		PM_WKEN_WKUP &= ~(0x10000);

	/* Disabling IO_PAD capabilities */
	if (core_sleep_flg)
		PM_WKEN_WKUP &= ~(0x100);

	/* Disabling IO_PAD capabilities */
	PM_WKEN_WKUP &= ~(0x100);
#ifdef OMAP3_START_RNG
	/*Capture the PM_PREPWSTST_CORE to be used later
	* for starting the RNG (Random Number Generator)*/
	prepwst_core_rng = PM_PREPWSTST_CORE;
#endif

	CM_FCLKEN1_CORE = fclken_core;
	CM_ICLKEN1_CORE = iclken_core;

	if (target_state.mpu_state > PRCM_MPU_ACTIVE) {
#ifdef CONFIG_MPU_OFF
		/* On ES 2.0, if scrathpad is populated with valid
		* pointer, warm reset does not work
		* So populate scrathpad restore address only in
		* cpuidle and suspend calls
		*/
		*(scratchpad_restore_addr) = 0x0;
#endif
		prcm_set_mpu_domain_state(PRCM_MPU_ACTIVE);
		if ((cur_neon_state == PRCM_ON) &&
			(target_state.mpu_state > PRCM_MPU_INACTIVE)) {
			prcm_force_power_domain_state(DOM_NEON, cur_neon_state);
			prcm_get_pre_power_domain_state(DOM_NEON,
				&pre_neon_state);

		if (pre_neon_state == PRCM_OFF)
			omap3_restore_neon_context();

#ifdef CONFIG_HW_SUP_TRANS
			prcm_set_power_domain_state(DOM_NEON, PRCM_ON,
							PRCM_AUTO);
#endif
		}
	}

	/* Continue core restoration part, only if Core-Sleep is attempted */
	if ((target_state.core_state > PRCM_CORE_ACTIVE) && core_sleep_flg) {
		prcm_set_core_domain_state(PRCM_CORE_ACTIVE);

#ifdef CONFIG_OMAP_SMARTREFLEX
		enable_smartreflex(SR1_ID);
		enable_smartreflex(SR2_ID);
#endif

		if (target_state.core_state >= PRCM_CORE_OSWR_MEMRET) {
#ifdef CONFIG_OMAP34XX_OFFMODE
			context_restore_update(DOM_CORE1);
#endif
			prcm_restore_registers(&target_state);
			prcm_restore_core_context(target_state.core_state);
			omap3_restore_core_settings();
			}
		/* Errata 1.4
		* if the timer device gets idled which is when we
		* are cutting the timer ICLK which is when we try
		* to put Core to RET.
		* Wait Period = 2 timer interface clock cycles +
		* 1 timer functional clock cycle
		* Interface clock = L4 clock. For the computation L4
		* clock is assumed at 50MHz (worst case).
		* Functional clock = 32KHz
		* Wait Period = 2*10^-6/50 + 1/32768 = 0.000030557 = 30.557uSec
		* Roundingoff the delay value to a safer 50uSec
		*/
		omap_udelay(GPTIMER_WAIT_DELAY);
		CM_AUTOIDLE_WKUP &= ~(0x1);
		if (core_off_notification != NULL)
			core_off_notification(PRCM_FALSE);
	}

	if (cur_per_state == PRCM_ON) {
		CM_FCLKEN_PER = fclken_per;
		CM_ICLKEN_PER = iclken_per;
		prcm_get_pre_power_domain_state(DOM_PER, &pre_per_state);
		if (pre_per_state == PRCM_OFF && per_ctx_saved) {
			if (enable_debug)
				per_off++;
			omap3_restore_per_context();
			post_uart_inactivity();
#ifdef CONFIG_OMAP34XX_OFFMODE
			context_restore_update(DOM_PER);
#endif
		}
	}

	pr_debug("MPU state:%x,CORE state:%x\n", PM_PREPWSTST_MPU,
							PM_PREPWSTST_CORE);
	store_prepwst();

return_sleep_time:
	getnstimeofday(&ts_postidle);
	ts_idle = timespec_sub(ts_postidle, ts_preidle);

	if (cx->type > OMAP3_STATE_C1)
		sched_clock_idle_wakeup_event(timespec_to_ns(&ts_idle));

#ifdef CONFIG_ENABLE_SWLATENCY_MEASURE
	if (idle_status) {
		sw_latency_arr[swlat_arr_wrptr].wkup_end =
					omap_32k_sync_timer_read();
		sw_latency_arr[swlat_arr_wrptr].wkup_start =
					wakeup_start_32ksync;

		sw_latency_arr[swlat_arr_wrptr].cstate =
			((PM_PREPWSTST_MPU & 0x3) << 2) |
			 (PM_PREPWSTST_CORE & 0x3) |
			 (omap_readl(0x48306CB0) << 16);

		swlat_arr_wrptr++;

		if (swlat_arr_wrptr == SW_LATENCY_ARR_SIZE)
			swlat_arr_wrptr = 0;
	}
#endif

	local_irq_enable();
	local_fiq_enable();

#ifdef OMAP3_START_RNG
	if (!is_device_type_gp()) {
		/*Start RNG after interrupts are enabled
		 * and only when CORE OFF was successful
		 */
		if (!(prepwst_core_rng & 0x3)) {
			ret = omap3_start_rng();
			if (ret)
				printk(KERN_INFO"Failed to generate new"
						" RN in idle %x\n", ret);
			prepwst_core_rng = 0xFF;
		}
	}
#endif

	return (u32)timespec_to_ns(&ts_idle)/1000;
}

static int omap3_enter_idle_bm(struct cpuidle_device *dev,
				struct cpuidle_state *state)
{
	if ((state->flags & CPUIDLE_FLAG_CHECK_BM) && omap3_idle_bm_check()) {
		if (dev->safe_state) {
			/* fix cpuidle accounting for state change */
			dev->last_state = dev->safe_state;
			return dev->safe_state->enter(dev, dev->safe_state);
		} else {
			omap_sram_idle();
			return 0;
		}
	}
	return omap3_enter_idle(dev, state);
}

DEFINE_PER_CPU(struct cpuidle_device, omap3_idle_dev);

void omap_init_power_states(void)
{
	omap3_power_states[OMAP3_STATE_C0].valid = 1;
	omap3_power_states[OMAP3_STATE_C0].type = OMAP3_STATE_C0;
	omap3_power_states[OMAP3_STATE_C0].sleep_latency = 0;
	omap3_power_states[OMAP3_STATE_C0].wakeup_latency = 0;
	omap3_power_states[OMAP3_STATE_C0].threshold = 0;
	omap3_power_states[OMAP3_STATE_C0].mpu_state = PRCM_MPU_ACTIVE;
	omap3_power_states[OMAP3_STATE_C0].core_state = PRCM_CORE_ACTIVE;
	omap3_power_states[OMAP3_STATE_C0].flags = CPUIDLE_FLAG_SHALLOW;

	omap3_power_states[OMAP3_STATE_C1].valid = 1;
	omap3_power_states[OMAP3_STATE_C1].type = OMAP3_STATE_C1;
	omap3_power_states[OMAP3_STATE_C1].sleep_latency = 10;
	omap3_power_states[OMAP3_STATE_C1].wakeup_latency = 10;
	omap3_power_states[OMAP3_STATE_C1].threshold = 30;
	omap3_power_states[OMAP3_STATE_C1].mpu_state = PRCM_MPU_ACTIVE;
	omap3_power_states[OMAP3_STATE_C1].core_state = PRCM_CORE_ACTIVE;
	omap3_power_states[OMAP3_STATE_C1].flags = CPUIDLE_FLAG_TIME_VALID |
		CPUIDLE_FLAG_SHALLOW;

	omap3_power_states[OMAP3_STATE_C2].valid = 1;
	omap3_power_states[OMAP3_STATE_C2].type = OMAP3_STATE_C2;
	omap3_power_states[OMAP3_STATE_C2].sleep_latency = 50;
	omap3_power_states[OMAP3_STATE_C2].wakeup_latency = 50;
	omap3_power_states[OMAP3_STATE_C2].threshold = 300;
	omap3_power_states[OMAP3_STATE_C2].mpu_state = PRCM_MPU_CSWR_L2RET;
	omap3_power_states[OMAP3_STATE_C2].core_state = PRCM_CORE_ACTIVE;
	omap3_power_states[OMAP3_STATE_C2].flags = CPUIDLE_FLAG_TIME_VALID |
			CPUIDLE_FLAG_BALANCED;

	omap3_power_states[OMAP3_STATE_C3].valid = 1;
	omap3_power_states[OMAP3_STATE_C3].type = OMAP3_STATE_C3;
	omap3_power_states[OMAP3_STATE_C3].sleep_latency = 1500;
	omap3_power_states[OMAP3_STATE_C3].wakeup_latency = 1800;
	omap3_power_states[OMAP3_STATE_C3].threshold = 13500;
#ifdef CONFIG_MPU_OFF
	omap3_power_states[OMAP3_STATE_C3].mpu_state = PRCM_MPU_OFF;
#else
	omap3_power_states[OMAP3_STATE_C3].mpu_state = PRCM_MPU_CSWR_L2RET;
#endif
	omap3_power_states[OMAP3_STATE_C3].core_state = PRCM_CORE_ACTIVE;
	omap3_power_states[OMAP3_STATE_C3].flags = CPUIDLE_FLAG_TIME_VALID |
			CPUIDLE_FLAG_BALANCED;

	omap3_power_states[OMAP3_STATE_C4].valid = 1;
	omap3_power_states[OMAP3_STATE_C4].type = OMAP3_STATE_C4;
	omap3_power_states[OMAP3_STATE_C4].sleep_latency = 2500;
	omap3_power_states[OMAP3_STATE_C4].wakeup_latency = 7500;
	omap3_power_states[OMAP3_STATE_C4].threshold = 13800;
	omap3_power_states[OMAP3_STATE_C4].mpu_state = PRCM_MPU_CSWR_L2RET;
	omap3_power_states[OMAP3_STATE_C4].core_state = PRCM_CORE_CSWR_MEMRET;
	omap3_power_states[OMAP3_STATE_C4].flags = CPUIDLE_FLAG_TIME_VALID |
		CPUIDLE_FLAG_BALANCED | CPUIDLE_FLAG_CHECK_BM;

	omap3_power_states[OMAP3_STATE_C5].valid = 1;
	omap3_power_states[OMAP3_STATE_C5].type = OMAP3_STATE_C5;
	omap3_power_states[OMAP3_STATE_C5].sleep_latency = 3000;
	omap3_power_states[OMAP3_STATE_C5].wakeup_latency = 8500;
	omap3_power_states[OMAP3_STATE_C5].threshold = 350000;
#ifdef CONFIG_MPU_OFF
	omap3_power_states[OMAP3_STATE_C5].mpu_state = PRCM_MPU_OFF;
#else
	omap3_power_states[OMAP3_STATE_C5].mpu_state = PRCM_MPU_CSWR_L2RET;
#endif
	omap3_power_states[OMAP3_STATE_C5].core_state = PRCM_CORE_CSWR_MEMRET;
	omap3_power_states[OMAP3_STATE_C5].flags = CPUIDLE_FLAG_TIME_VALID |
		CPUIDLE_FLAG_BALANCED | CPUIDLE_FLAG_CHECK_BM;

#ifdef CONFIG_OMAP34XX_OSWR
	omap3_power_states[OMAP3_STATE_C6].valid = 1;
#else
	omap3_power_states[OMAP3_STATE_C6].valid = 0;
#endif
	omap3_power_states[OMAP3_STATE_C6].type = OMAP3_STATE_C6;
	omap3_power_states[OMAP3_STATE_C6].sleep_latency = 4000;
	omap3_power_states[OMAP3_STATE_C6].wakeup_latency = 9000;
	omap3_power_states[OMAP3_STATE_C6].threshold = 380000;
	omap3_power_states[OMAP3_STATE_C6].mpu_state = PRCM_MPU_OSWR_L2OFF;
	omap3_power_states[OMAP3_STATE_C6].core_state = PRCM_CORE_OSWR_MEMOFF;
	omap3_power_states[OMAP3_STATE_C6].flags = CPUIDLE_FLAG_TIME_VALID |
			CPUIDLE_FLAG_DEEP | CPUIDLE_FLAG_CHECK_BM;

#ifdef CONFIG_OMAP34XX_OSWR
	omap3_power_states[OMAP3_STATE_C7].valid = 1;
#else
	omap3_power_states[OMAP3_STATE_C7].valid = 0;
#endif
	omap3_power_states[OMAP3_STATE_C7].type = OMAP3_STATE_C7;
	omap3_power_states[OMAP3_STATE_C7].sleep_latency = 8000;
	omap3_power_states[OMAP3_STATE_C7].wakeup_latency = 25000;
	omap3_power_states[OMAP3_STATE_C7].threshold = 450000;
#ifdef CONFIG_MPU_OFF
	omap3_power_states[OMAP3_STATE_C7].mpu_state = PRCM_MPU_OFF;
#else
	omap3_power_states[OMAP3_STATE_C7].mpu_state = PRCM_MPU_CSWR_L2RET;
#endif
	omap3_power_states[OMAP3_STATE_C7].core_state = PRCM_CORE_OSWR_MEMOFF;
	omap3_power_states[OMAP3_STATE_C7].flags = CPUIDLE_FLAG_TIME_VALID |
			CPUIDLE_FLAG_DEEP | CPUIDLE_FLAG_CHECK_BM;

#ifdef CONFIG_CORE_OFF_CPUIDLE
	omap3_power_states[OMAP3_STATE_C8].valid = 1;
#else
	omap3_power_states[OMAP3_STATE_C8].valid = 0;
#endif
	omap3_power_states[OMAP3_STATE_C8].type = OMAP3_STATE_C8;
	omap3_power_states[OMAP3_STATE_C8].sleep_latency = 10000;
	omap3_power_states[OMAP3_STATE_C8].wakeup_latency = 30000;
	omap3_power_states[OMAP3_STATE_C8].threshold = 500000;
	omap3_power_states[OMAP3_STATE_C8].mpu_state = PRCM_MPU_OFF;
	omap3_power_states[OMAP3_STATE_C8].core_state = PRCM_CORE_OFF;
	omap3_power_states[OMAP3_STATE_C8].flags = CPUIDLE_FLAG_TIME_VALID |
			CPUIDLE_FLAG_DEEP | CPUIDLE_FLAG_CHECK_BM;
}

struct cpuidle_driver omap3_idle_driver = {
	.name = 	"omap3_idle",
	.owner = 	THIS_MODULE,
};

int omap3_idle_init(void)
{
	int i, count = 0;
	struct omap3_processor_cx *cx;
	struct cpuidle_state *state;
	struct cpuidle_device *dev;

	omap_init_power_states();
	cpuidle_register_driver(&omap3_idle_driver);

	dev = &per_cpu(omap3_idle_dev, smp_processor_id());

	for (i = 0; i < OMAP3_MAX_STATES; i++) {
		cx = &omap3_power_states[i];
		state = &dev->states[count];

		if (!cx->valid)
			continue;
		cpuidle_set_statedata(state, cx);
		state->exit_latency = cx->sleep_latency + cx->wakeup_latency;
		state->target_residency = cx->threshold;
		state->flags = cx->flags;
		state->enter = (state->flags & CPUIDLE_FLAG_CHECK_BM) ?
			omap3_enter_idle_bm : omap3_enter_idle;
		if (cx->type == OMAP3_STATE_C2)
			dev->safe_state = state;
		sprintf(state->name, "C%d", count+1);
		count++;
	}

	if (!count)
		return -EINVAL;
	dev->state_count = count;

	if (cpuidle_register_device(dev)) {
		printk(KERN_ERR "%s: CPUidle register device failed\n",
			__FUNCTION__);
		return -EIO;
	}
#ifdef CONFIG_PROC_FS
	create_pmproc_entry();
#endif  /* #ifdef CONFIG_PROC_FS */

	/* Initialize UART inactivity time */
	awake_time_end = jiffies + msecs_to_jiffies(UART_TIME_OUT);

	return 0;
}
__initcall(omap3_idle_init);
