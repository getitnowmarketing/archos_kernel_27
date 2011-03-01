/*
 * linux/arch/arm/mach-omap2/resource_34xx.c
 * OMAP34XX Shared Resource Framework
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * History:
 *
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/notifier.h>
#include <mach/resource.h>
#include <mach/clock.h>
#include <linux/io.h> /* __raw_writel. Should be removed */
#include <mach/io.h> /* omap_writel. Should be removed */
#include <linux/delay.h>
#include "resource_34xx.h"
#include "ti-compat.h"
#include "prcm-regs.h"
#include <mach/power_companion.h>

u32 current_vdd1_opp = PRCM_VDD1_OPP3;
u32 current_vdd2_opp = PRCM_VDD2_OPP3;

/* global variables which can be used in idle_thread */
short	core_active = LOGICAL_UNUSED;

#ifdef CONFIG_OMAP34XX_OFFMODE
/* An array of struct which contains the context attributes for each domain. */
static struct domain_ctxsvres_status domain_ctxsvres[PRCM_NUM_DOMAINS];

/*  This function is called by the DeviceDriver when it has done context save.
 */
void context_save_done(struct clk *clk)
{
	struct domain_ctxsvres_status *ctxsvres;
	u32 prcm_id, domain_id, device_id, device_bitpos;

	prcm_id = clk->prcmid;
	domain_id = DOMAIN_ID(prcm_id);
	device_id = DEV_BIT_POS(prcm_id);
	device_bitpos = 1 << device_id;

	spin_lock(&svres_reg_lock);
	ctxsvres = &domain_ctxsvres[domain_id];

	ctxsvres->context_saved |= device_bitpos;
	spin_unlock(&svres_reg_lock);
}
EXPORT_SYMBOL(context_save_done);

/*  This function is called by the DeviceDriver to check whether context restore
 *  is required.
 */
int context_restore_required(struct clk *clk)
{
	struct domain_ctxsvres_status *ctxsvres;
	u32 prcm_id, domain_id, device_id, device_bitpos;
	int ret;
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res = (struct resource_handle *) clk;
	prcm_id = res->clk->prcmid;
#else
	prcm_id = clk->prcmid;
#endif
	ret = 0;

	domain_id = DOMAIN_ID(prcm_id);
	device_id = DEV_BIT_POS(prcm_id);
	device_bitpos = 1 << device_id;

	spin_lock(&svres_reg_lock);
	ctxsvres = &domain_ctxsvres[domain_id];

	if (ctxsvres->context_restore & device_bitpos) {
		ret = 1;
		ctxsvres->context_saved &= ~device_bitpos;
		ctxsvres->context_restore &= ~device_bitpos;
	}
	spin_unlock(&svres_reg_lock);

	return ret;
}
EXPORT_SYMBOL(context_restore_required);

/*  This function is called to update the context attributes when power domain
 *  is put to OFF.
 */
void context_restore_update(unsigned long domain_id)
{
	struct domain_ctxsvres_status *ctxsvres;

	spin_lock(&svres_reg_lock);
	ctxsvres = &domain_ctxsvres[domain_id];
	ctxsvres->context_restore = ctxsvres->context_saved;

	if (domain_id == DOM_CORE1) {
		ctxsvres = &domain_ctxsvres[DOM_CORE2];
		ctxsvres->context_restore = ctxsvres->context_saved;
		ctxsvres = &domain_ctxsvres[DOM_CORE3];
		ctxsvres->context_restore = ctxsvres->context_saved;
	}
	spin_unlock(&svres_reg_lock);
}
EXPORT_SYMBOL(context_restore_update);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

int __init omap2_resource_init(void)
{
/* make sure max_level for vdd1 and vdd2 are updated */
	vdd1_opp.max_levels = max_vdd1_opp + 1;
	vdd2_opp.max_levels = max_vdd2_opp + 1;
	return resource_init(res_list);
}

/* Activation function to change target level for a Power Domain resource */
/* Transtitions from RET to OFF and OFF to RET are not allowed 		  */
int activate_power_res(struct shared_resource *resp,
		       unsigned short current_level,
		       unsigned short target_level)
{
	int ret = 0;
	unsigned long prcm_id = resp->prcm_id;

	/* Common code for transitioning to RET/OFF. */
	switch (target_level) {
	case POWER_DOMAIN_RET:
	case POWER_DOMAIN_OFF:
#ifdef CONFIG_HW_SUP_TRANS
		ret = prcm_set_clock_domain_state(prcm_id, PRCM_NO_AUTO,
								PRCM_FALSE);
		if (ret != PRCM_PASS) {
			printk(KERN_ERR"FAILED in clock domain to NO_AUTO for"
						" domain %lu \n", prcm_id);
			return -1;
		}

		switch (prcm_id) {
		case DOM_DSS:
		case DOM_CAM:
		case DOM_PER:
		case DOM_USBHOST:
		case DOM_SGX:
			prcm_clear_sleep_dependency(prcm_id,
							PRCM_SLEEPDEP_EN_MPU);
			prcm_clear_wkup_dependency(prcm_id, PRCM_WKDEP_EN_MPU);
			break;
		}
#endif /* CONFIG_HW_SUP_TRANS */
		if (current_level != POWER_DOMAIN_ON) {
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_ON, PRCM_FORCE);
			if (ret != PRCM_PASS)
				return ret;
		}
		break;
	}
	switch (target_level) {
	case POWER_DOMAIN_ON:

			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_ON, PRCM_FORCE);
#ifdef CONFIG_HW_SUP_TRANS
		switch (prcm_id) {
		case DOM_DSS:
		case DOM_CAM:
		case DOM_PER:
		case DOM_USBHOST:
		case DOM_SGX:
			if (prcm_set_wkup_dependency(prcm_id,
					PRCM_WKDEP_EN_MPU) != PRCM_PASS) {
				printk("Domain %lu : wakeup dependency could"
						 " not be set\n", prcm_id);
				return -1;
			}
			if (prcm_set_sleep_dependency(prcm_id,
					PRCM_SLEEPDEP_EN_MPU) != PRCM_PASS) {
				printk("Domain %lu : sleep dependency could"
					" not be set\n", prcm_id);
				return -1;
			}
			/* NO BREAK */
		case DOM_NEON:
		case DOM_IVA2:
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_ON, PRCM_AUTO);
			break;
		}
#endif /* #ifdef CONFIG_HW_SUP_TRANS */
			break;
	case POWER_DOMAIN_RET:
#ifdef CONFIG_OMAP34XX_OFFMODE
		ret =  prcm_set_power_domain_state(prcm_id,
					POWER_DOMAIN_RET, PRCM_FORCE);
		break;
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
	case POWER_DOMAIN_OFF:

#ifdef CONFIG_OMAP34XX_OFFMODE
		if (prcm_id == DOM_PER)
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_RET, PRCM_FORCE);
		else
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_OFF, PRCM_FORCE);

		context_restore_update(prcm_id);
#else
		ret = prcm_set_power_domain_state(prcm_id,
					POWER_DOMAIN_RET, PRCM_FORCE);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
		break;

	default:
		ret = PRCM_FAIL;
	}
	return ret;
}

/* Activation function to change target level for a Logical resource */
int activate_logical_res(struct shared_resource *resp,
			 unsigned short current_level,
			 unsigned short target_level)
{
	unsigned long prcm_id = resp->prcm_id;

	switch (prcm_id) {
	case DOM_CORE1:
		core_active = (target_level)?LOGICAL_USED:LOGICAL_UNUSED;
		break;
	default:
		return -1;
	}
	return 0;
}

int activate_autoidle_resource(struct shared_resource *resp,
			unsigned short current_level,
			unsigned short target_level)
{
	unsigned long prcm_id = resp->prcm_id;
	int ret;
	ret = prcm_dpll_clock_auto_control(prcm_id, target_level);
	if (ret == PRCM_FAIL) {
		DPRINTK("Invalid DPLL Autoidle resource state\n");
		return -1;
	}
	return 0;
}

int validate_autoidle_resource(struct shared_resource *resp,
			unsigned short current_level,
			unsigned short target_level)
{
	int invalidlevelmin = 2, invalidlevelmax = 4;
	if (target_level >= resp->max_levels) {
		printk(KERN_ERR"Invalid Target Level\n");
		return -1;
	}
	if (strncmp(resp->name, "core_autoidle_res", 17) == 0)
	if (target_level >= invalidlevelmin &&
			target_level <= invalidlevelmax) {
		printk(KERN_ERR"Invalid Target Level\n");
		return -1;
	}
	return 0;
}

#ifdef CONFIG_TWL4030_CORE
int activate_triton_power_res(struct shared_resource *resp,
	unsigned short current_level,
	unsigned short target_level)
{
	int result;

	lock_scratchpad_sem();
/*
 * This need not be done for ZOOM2 as MMC works fine.
 * Also if CLK REQ is set to be always asserted we see
 * SYS OFF hanging
 */
#ifndef CONFIG_SYSOFFMODE
	/* Disable clk_req togling, if card inserted in MMC slot1 */
	if (resp->prcm_id == TWL4030_VMMC1_ID) {
		if (target_level == 0)
			PRM_CLKSRC_CTRL |= 0x18;
		else
			PRM_CLKSRC_CTRL &= ~0x18;
	}
#endif

#ifdef CONFIG_CORE_OFF
	save_scratchpad_contents();
#endif
	unlock_scratchpad_sem();

	result = twl4030_ldo_set_voltage(resp->prcm_id, target_level);
	return result;
}

int validate_triton_power_res(struct shared_resource *resp,
	unsigned short current_level,
	unsigned short target_level)
{
	if (target_level >= resp->max_levels)
		return -1;
	return 0;
}
#endif /* #ifdef CONFIG_TWL4030_CORE */


/* Validate if the power domain transition from current to target level is*/
/* valid							     	*/
int validate_power_res(struct shared_resource *res,
			unsigned short current_level,
			unsigned short target_level)
{
	DPRINTK("Current_level = %d, Target_level = %d\n",
			current_level, target_level);

	if ((target_level >= res->max_levels)) {
		DPRINTK("Invalid target_level requested\n");
		return -1;
	}

	/* RET to OFF and OFF to RET transitions not allowed for IVA*/
	if (res->prcm_id == DOM_IVA2) {
		if ((current_level < POWER_DOMAIN_ON) &&
				(target_level != POWER_DOMAIN_ON)) {
			DPRINTK("%d to %d transitions for"
					" Power Domain IVA are not allowed\n",
					current_level, target_level);
			return -1;
		}
	}

	return 0;
}

/* Validate if the logical resource transition is valid */
int validate_logical_res(struct shared_resource *res,
				unsigned short current_level,
				unsigned short target_level)
{
	if (target_level >= res->max_levels)
		return -1;
	return 0;
}


/* Turn off unused power domains during bootup
 * If OFFMODE macro is enabled, all power domains are turned off during bootup
 * If OFFMODE macro is not enabled, all power domains except IVA and GFX
 * are put to retention and IVA,GFX are put to off
 * NEON is not put to off because it is required by VFP
 */
int turn_power_domains_off(void)
{
	struct shared_resource **resp;
	u8 state;
	u32 prcmid;

	/* TODO:
	 * Move this and the USB otg force-idle/stdby
	 * to a quirk function.
	 * Setting otg bits here seems to break. There seems to be some
	 * dependency with other domains being off before we set otg
	 * to force-idle/stdby.
	 */
	for (resp = res_list; *resp; resp++) {
		if (list_empty(&((*resp)->users_list))) {
			if ((*resp)->res_type == RES_POWER_DOMAIN) {
				prcmid = (*resp)->prcm_id;
				prcm_get_power_domain_state(prcmid, &state);
				if (state == PRCM_ON) {
#ifdef CONFIG_OMAP34XX_OFFMODE
					if (prcmid == DOM_PER)
						state = PRCM_ON;
					else
						state = PRCM_OFF;
#else
					if ((prcmid == DOM_IVA2) ||
							(prcmid == DOM_SGX))
						state = PRCM_OFF;
					else
						state = PRCM_RET;
#endif
					if (prcmid == DOM_NEON)
						state = PRCM_ON;
					prcm_force_power_domain_state
						((*resp)->prcm_id, state);
				}
			}
		}
	}
	return 0;
}

#ifdef CONFIG_AUTO_POWER_DOMAIN_CTRL
late_initcall(turn_power_domains_off);
#endif
