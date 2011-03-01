/*
 * linux/arch/arm/mach-omap2/constraint_34xx.c
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
#include <linux/clk.h>
#include <linux/pm_qos_params.h>
#ifdef CONFIG_TRACK_RESOURCES
#include <linux/device.h>
#endif

#include <mach/clock.h>
#include <mach/resource.h>
#include <mach/prcm_34xx.h>
#include "pm_idle_34xx.h"

#define S800M   800000000
#define S720M   720000000
#define S600M   600000000
#define S550M   550000000
#define S500M   500000000
#define S250M   250000000
#define S125M   125000000
#define S19M    19200000
#define S120M   120000000
#define S477M   476000000
#define S381M   381000000
#define S191M   190000000
#define S96M    96000000
#define S166M	166000000
#define S83M	83000000
#define S66M    66500000
#define S133M   133000000
#define S266M   266000000
#define S293M   293000000
#define S320M   320000000

/* #define DEBUG_CONS_FRWK 1 */
#ifdef DEBUG_CONS_FRWK
#define DPRINTK(fmt, args...)\
 printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

#ifdef CONFIG_TRACK_RESOURCES
/* device name needed for resource tracking layer */
struct device_driver vdd1_opp_drv = {
	.name =  "vdd1_opp",
};

struct device vdd1_opp_dev = {
	.driver = &vdd1_opp_drv,
};

struct device_driver vdd2_opp_drv = {
	.name =  "vdd2_opp",
};

struct device vdd2_opp_dev = {
	.driver = &vdd2_opp_drv,
};
#endif

extern struct omap3_processor_cx omap3_power_states[];

struct res_handle *vdd1_res;
struct res_handle *vdd1_opp_co;
struct res_handle *vdd2_opp_co;
unsigned int vdd1_users;
struct res_handle *vdd2_res;

u32 valid_rate;

unsigned int vdd1_arm_dsp_freq[7][4] = {
	{125, 90, CO_VDD1_OPP1, PRCM_VDD1_OPP1},
	{250, 180, CO_VDD1_OPP2, PRCM_VDD1_OPP2},
	{500, 360, CO_VDD1_OPP3, PRCM_VDD1_OPP3},
	{550, 396, CO_VDD1_OPP4, PRCM_VDD1_OPP4},
	{600, 430, CO_VDD1_OPP5, PRCM_VDD1_OPP5},
	{720, 520, CO_VDD1_OPP6, PRCM_VDD1_OPP6},
	{800, 600, CO_VDD1_OPP7, PRCM_VDD1_OPP7},
};

unsigned int vdd2_core_freq[3][3] = {
	{0, CO_VDD2_OPP1, PRCM_VDD2_OPP1},
	{83, CO_VDD2_OPP2, PRCM_VDD2_OPP2},
	{166, CO_VDD2_OPP3, PRCM_VDD2_OPP3},
};

#define scale_volt_then_freq  (cur_opp_no < target_opp_no)
unsigned int rnd_rate_vdd1[7] = {
	S125M, S250M, S500M, S550M, S600M, S720M, S800M
};
unsigned int rnd_rate_vdd2[3] = {
	0, S83M, S166M
};

#define request_vdd2_co	(target_value > CO_VDD1_OPP2)
#define release_vdd2_co	(target_value <= CO_VDD1_OPP2)
#define vdd2_co_val	CO_VDD2_OPP3

unsigned int vdd1_arm_prenotifications;
unsigned int vdd1_arm_postnotifications;
unsigned int vdd1_dsp_prenotifications;
unsigned int vdd1_dsp_postnotifications;

struct atomic_notifier_head freq_arm_pre_notifier_list;
struct atomic_notifier_head freq_arm_post_notifier_list;
struct atomic_notifier_head freq_dsp_pre_notifier_list;
struct atomic_notifier_head freq_dsp_post_notifier_list;

/* WARNING: id_to_* has to be in sync with domains numbering in prcm.h */
static char *id_to_lname[] = {"lat_iva2", "lat_mpu", "lat_core1", "lat_core1",
			     "lat_3d", NULL, "lat_dss", "lat_cam", "lat_per",
				NULL, "lat_neon",
				NULL, NULL,
			      "lat_core1", "lat_usb",
};

int nb_arm_freq_prenotify_func(struct notifier_block *n, unsigned long event,
					void *ptr)
{
	atomic_notifier_call_chain(&freq_arm_pre_notifier_list,
		vdd1_arm_dsp_freq[event-1][0], NULL);
	return 0;
}

int nb_arm_freq_postnotify_func(struct notifier_block *n, unsigned long event,
					void *ptr)
{
	atomic_notifier_call_chain(&freq_arm_post_notifier_list,
		vdd1_arm_dsp_freq[event-1][0], NULL);
	return 0;
}

int nb_dsp_freq_prenotify_func(struct notifier_block *n, unsigned long event,
					void *ptr)
{
	atomic_notifier_call_chain(&freq_dsp_pre_notifier_list,
		vdd1_arm_dsp_freq[event-1][1], NULL);
	return 0;
}

int nb_dsp_freq_postnotify_func(struct notifier_block *n, unsigned long event,
					void *ptr)
{
	atomic_notifier_call_chain(&freq_dsp_post_notifier_list,
		vdd1_arm_dsp_freq[event-1][1], NULL);
	return 0;
}

static struct notifier_block nb_arm_freq_prenotify = {
	nb_arm_freq_prenotify_func,
	NULL,
};

static struct notifier_block nb_arm_freq_postnotify = {
	nb_arm_freq_postnotify_func,
	NULL,
};

static struct notifier_block nb_dsp_freq_prenotify = {
	nb_dsp_freq_prenotify_func,
	NULL,
};

static struct notifier_block nb_dsp_freq_postnotify = {
	nb_dsp_freq_postnotify_func,
	NULL,
};

#ifdef CONFIG_TRACK_RESOURCES
#define R(clk) (res->clk)
#else
#define R(clk) (clk)
#endif

/* To set the opp for vdd1 */
unsigned int vdd1_opp_setting(u32 target_opp_no)
{
	unsigned int cur_opp_no, target_vdd1_opp;
#ifdef CONFIG_TRACK_RESOURCES
	struct device *dev = &vdd1_opp_dev;
	struct resource_handle *res = NULL;
#else
	struct clk *clk = NULL;
#endif

	target_vdd1_opp = vdd1_arm_dsp_freq[target_opp_no-1][3];
	cur_opp_no = get_opp_no(current_vdd1_opp);

#ifdef CONFIG_TRACK_RESOURCES
	res = (struct resource_handle *) clk_get(dev, "virt_vdd1_prcm_set");
#else
	clk = clk_get(NULL, "virt_vdd1_prcm_set");
#endif

	if (R(clk) == NULL)
		printk(KERN_ERR "Unable to get clk virt_vdd1_prcm_set\n");

	if (scale_volt_then_freq) {
		prcm_do_voltage_scaling(target_vdd1_opp, current_vdd1_opp);
		valid_rate = clk_round_rate(R(clk),
			rnd_rate_vdd1[target_opp_no-1]);
		R(clk)->set_rate(R(clk), valid_rate);
	} else {
		valid_rate = clk_round_rate(R(clk),
			rnd_rate_vdd1[target_opp_no-1]);
		R(clk)->set_rate(R(clk), valid_rate);
		prcm_do_voltage_scaling(target_vdd1_opp, current_vdd1_opp);
	}

#ifdef CONFIG_TRACK_RESOURCES
	clk_put(res);
#else
	clk_put(clk);
#endif
	return target_vdd1_opp;
}

/* To set the opp value for vdd2 */
unsigned int vdd2_opp_setting(u32 target_opp_no)
{
	unsigned int cur_opp_no, target_vdd2_opp;
#ifdef CONFIG_TRACK_RESOURCES
	struct device *dev = &vdd2_opp_dev;
	struct resource_handle *res = NULL;
#else
	struct clk *clk = NULL;
#endif

	target_vdd2_opp = vdd2_core_freq[target_opp_no-1][2];
	cur_opp_no = get_opp_no(current_vdd2_opp);

#ifdef CONFIG_TRACK_RESOURCES
	res = (struct resource_handle *) clk_get(dev, "virt_vdd2_prcm_set");
#else
	clk = clk_get(NULL, "virt_vdd2_prcm_set");
#endif

	if (R(clk) == NULL)
		printk(KERN_ERR "Unable to get clk virt_vdd2_prcm_set\n");

	if (scale_volt_then_freq) {
		prcm_do_voltage_scaling(target_vdd2_opp, current_vdd2_opp);
		valid_rate = clk_round_rate(R(clk),
			rnd_rate_vdd2[target_opp_no-1]);
		R(clk)->set_rate(R(clk), valid_rate);
	} else {
		valid_rate = clk_round_rate(R(clk),
			rnd_rate_vdd2[target_opp_no-1]);
		R(clk)->set_rate(R(clk), valid_rate);
		prcm_do_voltage_scaling(target_vdd2_opp, current_vdd2_opp);
	}

#ifdef CONFIG_TRACK_RESOURCES
	clk_put(res);
#else
	clk_put(clk);
#endif
	return target_vdd2_opp;
}

u8 pm_qos_req_added = 0;

int activate_constraint(struct shared_resource *resp,
			unsigned short current_value,
			unsigned short target_value)
{
	int ind;
	unsigned long type = resp->prcm_id;

	DPRINTK("CUR_VAL = %d, TAR_VAL = %d\n", current_value, target_value);
	if (type == RES_LATENCY_CO) {

		/* Remove previous latencies set by omap_lt_co */
		if (pm_qos_req_added) {
			pm_qos_remove_requirement(PM_QOS_CPU_DMA_LATENCY,
							"omap_lt_co");
			pm_qos_req_added = 0;
		}
		
		if (target_value == CO_UNUSED)
			return 0;

		if (target_value <= CO_LATENCY_MPUOFF_COREOSWR) {
			pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY,
			"omap_lt_co",
			omap3_power_states[target_value].sleep_latency +
			omap3_power_states[target_value].wakeup_latency + 1);
			pm_qos_req_added = 1;
		} else {
			pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY,
				"omap_lt_co", target_value);
			pm_qos_req_added = 1;
		}
	} else if ((type == PRCM_ARMFREQ_CONSTRAINT)) {
		for (ind = 0; ind < max_vdd1_opp; ind++) {
			if (vdd1_arm_dsp_freq[ind][0] >= target_value) {
				resource_request(vdd1_res,
					vdd1_arm_dsp_freq[ind][2]);
				break;
			}
		}
	} else if ((type == PRCM_DSPFREQ_CONSTRAINT)) {
		for (ind = 0; ind < max_vdd1_opp; ind++) {
			if (vdd1_arm_dsp_freq[ind][1] >= target_value) {
				resource_request(vdd1_res,
					vdd1_arm_dsp_freq[ind][2]);
				break;
			}
		}
	} else if (type == PRCM_VDD1_CONSTRAINT) {
		if (request_vdd2_co) {
			if (vdd2_res == NULL) {
				vdd2_res = resource_get("co_fwk", "vdd2_opp");
				resource_request(vdd2_res, vdd2_co_val);
			}
		}
		current_vdd1_opp = vdd1_opp_setting(target_value);
		if (release_vdd2_co) {
			if (vdd2_res != NULL) {
				resource_release(vdd2_res);
				resource_put(vdd2_res);
				vdd2_res = NULL;
			}
		}
	} else if (type == PRCM_VDD2_CONSTRAINT) {
		current_vdd2_opp = vdd2_opp_setting(target_value);
	}
	return 0;
}

int activate_pd_constraint(struct shared_resource *resp,
			   unsigned short current_value,
			   unsigned short target_value)
{
	struct res_handle_node *node;

	DPRINTK("CUR_VAL = %d, TAR_VAL = %d\n", current_value, target_value);

	if (list_empty(&resp->linked_res_handles))
		return 0;

	node = list_first_entry(&resp->linked_res_handles,
				struct res_handle_node,
				node);

	switch (target_value) {
	case CO_LATENCY_ON:
		/* Allows only ON power doamin state */
		return resource_request(node->handle, POWER_DOMAIN_ON);
		break;
	case CO_LATENCY_RET:
		/* Allows ON and RET power domain states */
		return resource_request(node->handle,
					POWER_DOMAIN_RET);
		break;
	case CO_UNUSED:
		/* Removing the constraints */
		return resource_release(node->handle);
		break;
	default:
		/* do nothing - allows all power domain states */
		break;
	}
	return 0;
}

int validate_constraint(struct shared_resource *res,
				unsigned short current_value,
				unsigned short target_value)
{
	if (target_value == DEFAULT_LEVEL)
		return 0;
	if (res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		if ((target_value < min_arm_freq) ||
				(target_value > max_arm_freq)) {
			printk(KERN_ERR "Invalid ARM frequency requested\n");
			return -1;
		}
	} else if (res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		if ((target_value < min_dsp_freq) ||
				(target_value > max_dsp_freq)) {
			printk(KERN_ERR "Invalid DSP frequency requested\n");
			return -1;
		}
	} else if (res->prcm_id == PRCM_VDD1_CONSTRAINT) {
		if ((target_value < min_vdd1_opp) ||
				(target_value > max_vdd1_opp)) {
			printk(KERN_ERR "Invalid VDD1 OPP requested\n");
			return -1;
		}
	} else if (res->prcm_id == PRCM_VDD2_CONSTRAINT) {
		if ((target_value < min_vdd2_opp) ||
				(target_value > max_vdd2_opp)) {
			printk(KERN_ERR "Invalid VDD2 OPP requested\n");
			return -1;
		}
	}
	return 0;
}

/* Wrappers to handle Constraints */
struct constraint_handle *constraint_get(const char *name,
					 struct constraint_id *cnstr_id)
{
	char *id;
	struct clk *clk;
	switch (cnstr_id->type) {
	case RES_CLOCK_RAMPUP_CO:
		clk = (struct clk *)cnstr_id->data;
			if ((clk == NULL) || IS_ERR(clk))
			return ERR_PTR(-ENOENT);

		id = id_to_lname[DOMAIN_ID(clk->prcmid) - 1];
		break;

	case RES_FREQ_CO:
		DPRINTK("Freq constraint %s requested\n",
			(char *)cnstr_id->data);
		if (vdd1_users == 0)
			vdd1_res = resource_get("co_fwk", "vdd1_opp");
			vdd1_users++;


			id = (char *)cnstr_id->data;
		break;

	default:
		id = (char *)cnstr_id->data;
		break;
	}
	/* Just calls the shared resource api */
	return (struct constraint_handle *) resource_get(name, id);
}
EXPORT_SYMBOL(constraint_get);

int constraint_set(struct constraint_handle *constraint,
					unsigned short constraint_level)
{
	return resource_request((struct res_handle *)constraint,
							constraint_level);
}
EXPORT_SYMBOL(constraint_set);

int constraint_remove(struct constraint_handle *constraint)
{
	return resource_release((struct res_handle *)constraint);
}
EXPORT_SYMBOL(constraint_remove);

void constraint_put(struct constraint_handle *constraint)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if ((strcmp("arm_freq", res_h->res->name) == 0)
		|| (strcmp("dsp_freq", res_h->res->name) == 0)) {
		vdd1_users--;
		if (vdd1_users == 0) {
			resource_release(vdd1_res);
			resource_put(vdd1_res);
			vdd1_res = NULL;
		}
	}
	resource_put((struct res_handle *)constraint);
}
EXPORT_SYMBOL(constraint_put);

void constraint_register_pre_notification(struct constraint_handle *constraint,
			 struct notifier_block *nb, unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		if (!vdd1_arm_prenotifications)
			resource_register_pre_notification(vdd1_res,
				&nb_arm_freq_prenotify, max_vdd1_opp+1);
		atomic_notifier_chain_register(&freq_arm_pre_notifier_list, nb);
		vdd1_arm_prenotifications++;
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		if (!vdd1_dsp_prenotifications)
			resource_register_pre_notification(vdd1_res,
				&nb_dsp_freq_prenotify, max_vdd1_opp+1);

		atomic_notifier_chain_register(&freq_dsp_pre_notifier_list, nb);
		vdd1_dsp_prenotifications++;
	} else {
		resource_register_pre_notification(
			(struct res_handle *)constraint, nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_register_pre_notification);

void constraint_register_post_notification(struct constraint_handle *constraint,
		struct notifier_block *nb, unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		if (!vdd1_arm_postnotifications)
			resource_register_post_notification(vdd1_res,
				&nb_arm_freq_postnotify, max_vdd1_opp+1);


		atomic_notifier_chain_register(&freq_arm_post_notifier_list,
									nb);
		vdd1_arm_postnotifications++;
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		if (!vdd1_dsp_postnotifications)
			resource_register_post_notification(vdd1_res,
				&nb_dsp_freq_postnotify, max_vdd1_opp+1);

		atomic_notifier_chain_register(&freq_dsp_post_notifier_list,
									nb);
		vdd1_dsp_postnotifications++;
	} else {
		resource_register_post_notification(
				(struct res_handle *)constraint,
				nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_register_post_notification);

void constraint_unregister_pre_notification(
		struct constraint_handle *constraint,
		struct notifier_block *nb,
		 unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		vdd1_arm_prenotifications--;
		atomic_notifier_chain_unregister(&freq_arm_pre_notifier_list,
									nb);
		if (!vdd1_arm_prenotifications)
			resource_unregister_pre_notification(vdd1_res,
			&nb_arm_freq_prenotify, max_vdd1_opp+1);
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		vdd1_dsp_prenotifications--;
		atomic_notifier_chain_unregister(&freq_dsp_pre_notifier_list,
									nb);
		if (!vdd1_dsp_prenotifications)
			resource_unregister_pre_notification(vdd1_res,
				&nb_dsp_freq_prenotify, max_vdd1_opp+1);
	} else {
		resource_unregister_pre_notification(
				(struct res_handle *)constraint,
				nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_unregister_pre_notification);

void constraint_unregister_post_notification(
		struct constraint_handle *constraint,
		struct notifier_block *nb, unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		vdd1_arm_postnotifications--;
		atomic_notifier_chain_unregister(&freq_arm_post_notifier_list,
									nb);
		if (!vdd1_arm_postnotifications)
			resource_unregister_post_notification(vdd1_res,
				&nb_arm_freq_postnotify, max_vdd1_opp+1);
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		vdd1_dsp_postnotifications--;
		atomic_notifier_chain_unregister(&freq_dsp_post_notifier_list,
									nb);
		if (!vdd1_dsp_postnotifications)
			resource_unregister_post_notification(vdd1_res,
				&nb_dsp_freq_postnotify, max_vdd1_opp+1);
		} else {
		resource_unregister_post_notification(
			(struct res_handle *)constraint,
							nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_unregister_post_notification);

int constraint_get_level(struct constraint_handle *constraint)
{
	return resource_get_level((struct res_handle *)constraint);
}
EXPORT_SYMBOL(constraint_get_level);
