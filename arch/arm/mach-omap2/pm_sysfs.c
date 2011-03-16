/*
 * linux/arch/arm/mach-omap2/pm_sysfs.c
 *
 * OMAP3 PM sysfs management source
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Based on pm.c for omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/suspend.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/pm_qos_params.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <mach/pm.h>
#include <mach/resource.h>
#include <mach/prcm_34xx.h>
#include "pm_idle_34xx.h"

/* Defines the default value of frame buffer timeout*/
int fb_timeout_val = 30; /*30 seconds*/
int enable_debug;
int enable_off = 1;
int enable_overdrive;

extern u32 current_vdd1_opp;
extern u32 current_vdd2_opp;
extern int enable_off;
extern struct omap3_processor_cx omap3_power_states[];

static struct constraint_handle *vdd1_opp_cons;
static struct constraint_handle *vdd2_opp_cons;
static struct constraint_handle *arm_freq_co;
static struct constraint_handle *dsp_freq_co;

static struct constraint_id cnstr_id1 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd1_opp",
};

static struct constraint_id cnstr_id2 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd2_opp",
};

static struct constraint_id cnstr_id3 = {
	.type = RES_FREQ_CO,
	.data = (void *)"arm_freq",
};

static struct constraint_id cnstr_id4 = {
	.type = RES_FREQ_CO,
	.data = (void *)"dsp_freq",
};

static ssize_t vdd_opp_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t vdd_opp_store(struct kobject *k, struct kobj_attribute *,
				const char *buf, size_t n);
static ssize_t vdd_freq_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t vdd_freq_store(struct kobject *k, struct kobj_attribute *,
				const char *buf, size_t n);

static struct kobj_attribute vdd1_opp_attr =
	__ATTR(vdd1_opp_value, 0644, vdd_opp_show, vdd_opp_store);

static struct kobj_attribute vdd2_opp_attr =
	__ATTR(vdd2_opp_value, 0644, vdd_opp_show, vdd_opp_store);

static struct kobj_attribute arm_freq_attr =
	__ATTR(arm_freq_value, 0644, vdd_freq_show, vdd_freq_store);

static struct kobj_attribute dsp_freq_attr =
	__ATTR(dsp_freq_value, 0644, vdd_freq_show, vdd_freq_store);

static ssize_t vdd_opp_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	if (attr == &vdd1_opp_attr)
		return sprintf(buf, "%x\n", (unsigned int)get_opp_no(
							current_vdd1_opp));
	else if (attr == &vdd2_opp_attr)
		return sprintf(buf, "%x\n", (unsigned int)get_opp_no(
							current_vdd2_opp));
	else
		return -EINVAL;
}

static ssize_t vdd_opp_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1)
		return -EINVAL;

	if (attr == &vdd1_opp_attr) {
		if (value < min_vdd1_opp || value > max_vdd1_opp) {
			printk(KERN_ERR "vdd_opp_store: Invalid value\n");
			return -EINVAL;
		}
		if (value == get_opp_no(current_vdd1_opp)) {
			pr_debug("Target aand current opp values are same\n");
		} else {
			if (value != 1) {
				if (vdd1_opp_cons == NULL)
					vdd1_opp_cons = constraint_get("pm_fwk",
						&cnstr_id1);
					constraint_set(vdd1_opp_cons, value);
			} else {
				if (vdd1_opp_cons != NULL) {
					constraint_remove(vdd1_opp_cons);
					constraint_put(vdd1_opp_cons);
					vdd1_opp_cons = NULL;
				}
			}
		}
		return n;
	} else if (attr == &vdd2_opp_attr) {
		if (value < min_vdd2_opp || value > max_vdd2_opp) {
			printk(KERN_ERR "vdd_opp_store: Invalid value\n");
			return -EINVAL;
		}
		if (value == get_opp_no(current_vdd2_opp)) {
			pr_debug("Target and current opp values are same\n");
		} else {
			if (value != 1) {
				if (vdd2_opp_cons == NULL)
					vdd2_opp_cons = constraint_get("pm_fwk",
						&cnstr_id2);
				constraint_set(vdd2_opp_cons, value);
			} else {
				if (vdd2_opp_cons != NULL) {
					constraint_remove(vdd2_opp_cons);
					constraint_put(vdd2_opp_cons);
					vdd2_opp_cons = NULL;
				}
			}
		}
	} else {
		return -EINVAL;
	}
	return n;
}

static ssize_t vdd_freq_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	u32 arm_speed, dsp_speed;

	if (attr == &arm_freq_attr) {
		prcm_get_processor_speed(DOM_MPU, &arm_speed);
		return sprintf(buf, "%d\n", arm_speed);
	} else if (attr == &dsp_freq_attr) {
		prcm_get_processor_speed(PRCM_IVA2, &dsp_speed);
		return sprintf(buf, "%d\n", dsp_speed);
	} else
		return -EINVAL;
}

static ssize_t vdd_freq_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t n)
{

	u32 target_arm_freq, target_dsp_freq;

	if (attr == &arm_freq_attr) {
		sscanf(buf, "%u", &target_arm_freq);
		if ((target_arm_freq <  min_arm_freq) ||
					(target_arm_freq  > max_arm_freq)) {
			printk(KERN_ERR "MPU frequency Invalid value\n");
			return -EINVAL;
		}

		if (target_arm_freq == curr_arm_freq) {
			pr_debug("Target and current Arm frequency is same\n");
		} else {
			if (target_arm_freq != min_arm_freq) {
				if (arm_freq_co == NULL)
					arm_freq_co = constraint_get("arm freq",
						&cnstr_id3);
					constraint_set(arm_freq_co,
						target_arm_freq);
			} else {
				if (arm_freq_co != NULL) {
					constraint_remove(arm_freq_co);
					constraint_put(arm_freq_co);
					arm_freq_co = NULL;
				}
			}
		}
		return n;
	} else if (attr == &dsp_freq_attr) {
		sscanf(buf, "%u", &target_dsp_freq);
		if ((target_dsp_freq < min_dsp_freq) ||
				(target_dsp_freq  > max_dsp_freq)) {
			printk(KERN_ERR "DSP freqency :Invalid value\n");
			return -EINVAL;
		}
		if (target_dsp_freq  == curr_dsp_freq) {
			pr_debug("Target and current dsp frequency is same\n");
		} else {
			if (target_dsp_freq != min_dsp_freq) {
				if (dsp_freq_co == NULL)
					dsp_freq_co = constraint_get("dsp freq",
							&cnstr_id4);
					constraint_set(dsp_freq_co,
							target_dsp_freq);
			} else {
				if (dsp_freq_co != NULL) {
					constraint_remove(dsp_freq_co);
					constraint_put(dsp_freq_co);
					dsp_freq_co = NULL;
				}
			}

		}
		return n;
	} else
		return -EINVAL;
}

static ssize_t pm_off_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t pm_off_store(struct kobject *k, struct kobj_attribute *,
			const char *buf, size_t n);
static struct kobj_attribute pm_off_attr =
	__ATTR(enable_mpucoreoff, 0644, pm_off_show, pm_off_store);

static ssize_t pm_off_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%hu\n", enable_off);
}

static ssize_t pm_off_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	unsigned short value;
	if (sscanf(buf, "%hu", &value) != 1 ||
		((value != 0) && (value != 1))) {
		printk(KERN_ERR "off_enable: Invalid value\n");
		return -EINVAL;
	}
	enable_off = value;
	return n;
}

static ssize_t debugflag_show(struct kobject *, struct kobj_attribute *,
								char *);
static ssize_t debugflag_store(struct kobject *k, struct kobj_attribute *,
			const char *buf, size_t n);
static struct kobj_attribute debugflag_attr =
	__ATTR(enable_debug, 0644, debugflag_show, debugflag_store);

static ssize_t debugflag_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%hu\n", enable_debug);
}

static ssize_t debugflag_store(struct kobject *kobj, struct kobj_attribute
			*attr, const char *buf, size_t n)
{
	unsigned short value;
	if (sscanf(buf, "%hu", &value) != 1 ||
		((value != 0) && (value != 1))) {
		printk(KERN_ERR "debug_enable: Invalid value\n");
		return -EINVAL;
	}
	enable_debug = value;
	return n;
}

static ssize_t overdrive_show(struct kobject *, struct kobj_attribute *,
								char *);
static ssize_t overdrive_store(struct kobject *, struct kobj_attribute *,
				const char *buf, size_t n);
static struct kobj_attribute overdriveflag_attr =
	__ATTR(enable_overdrive, 0644, overdrive_show, overdrive_store);

static ssize_t overdrive_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%hu\n", enable_overdrive);
}

static ssize_t overdrive_store(struct kobject *kobj, struct kobj_attribute
				*attr, const char *buf, size_t n)
{
	unsigned short value;
	if (sscanf(buf, "%hu", &value) != 1 ||
		((value != 0) && (value != 1))) {
		printk(KERN_ERR "enable_overdrive: Invalid value\n");
		return -EINVAL;
	}
	enable_overdrive = value;
	return n;
}

int omap_is_overdrive(void)
{
	return enable_overdrive;
}
EXPORT_SYMBOL(omap_is_overdrive);

static ssize_t fbtimeout_show(struct kobject *, struct kobj_attribute *,
								char *);
static ssize_t fbtimeout_store(struct kobject *k, struct kobj_attribute *,
			const char *buf, size_t n);
static struct kobj_attribute fbtimeout_attr =
	__ATTR(fb_timeout_value, 0644, fbtimeout_show, fbtimeout_store);

static ssize_t fbtimeout_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%hu\n", fb_timeout_val);
}

static ssize_t fbtimeout_store(struct kobject *kobj, struct kobj_attribute
			*attr, const char *buf, size_t n)
{
	unsigned short value;
	if (sscanf(buf, "%hu", &value) != 1) {
		printk(KERN_ERR "fb_timeout_store: Invalid value\n");
		return -EINVAL;
	}
	fb_timeout_val = value;
	set_blank_interval(fb_timeout_val);
	return n;
}

#ifdef CONFIG_CPU_IDLE

#ifdef CONFIG_PREVENT_MPU_RET
int  cpuidle_deepest_st = 1;
#elif defined(CONFIG_PREVENT_CORE_RET)
int  cpuidle_deepest_st = 3;
#else
int cpuidle_deepest_st = MAX_SUPPORTED_STATES - 1;
#endif

static ssize_t idle_state_show(struct kobject *, struct kobj_attribute *,
			char *);
static ssize_t idle_state_store(struct kobject *, struct kobj_attribute *,
			const char *buf, size_t n);

static struct kobj_attribute cpuidle_deepest_state_attr =
	__ATTR(cpuidle_deepest_state, 0644, idle_state_show, idle_state_store);

static ssize_t idle_state_show(struct kobject *kobj, struct kobj_attribute
			*attr, char *buf)
{
	return sprintf(buf, "%hu\n", cpuidle_deepest_st);
}

static ssize_t idle_state_store(struct kobject *kobj, struct kobj_attribute
			*attr, const char *buf, size_t n)
{
	unsigned short state;

	if (sscanf(buf, "%hu", &state) != 1 ||
		(state > (MAX_SUPPORTED_STATES - 1))) {
		printk(KERN_ERR "idle_sleep_store: Invalid value\n");
		return -EINVAL;
	}

	pm_qos_remove_requirement(PM_QOS_CPU_DMA_LATENCY, "omap_pm_sys");
	pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY, "omap_pm_sys",
				omap3_power_states[state].sleep_latency +
				omap3_power_states[state].wakeup_latency + 1);
	cpuidle_deepest_st = state;
	return n;
}
#endif

static int __init omap3_pm_sysfs_init(void)
{
	int l;
	u32 vdd1_opp_no, vdd2_opp_no;

#if defined(CONFIG_OMAP3ES2_VDD1_OPP1)
	vdd1_opp_no = PRCM_VDD1_OPP1;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP2)
	vdd1_opp_no = PRCM_VDD1_OPP2;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP3)
	vdd1_opp_no = PRCM_VDD1_OPP3;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP4)
	vdd1_opp_no = PRCM_VDD1_OPP4;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP5)
	vdd1_opp_no = PRCM_VDD1_OPP5;
#endif

#ifdef CONFIG_OMAP3ES2_VDD2_OPP2
	vdd2_opp_no = PRCM_VDD2_OPP2;
#elif defined(CONFIG_OMAP3ES2_VDD2_OPP3)
	vdd2_opp_no = PRCM_VDD2_OPP3;
#endif

	/* Request for VDD1 and VDD2 OPP constraints */
	vdd1_opp_cons = constraint_get("pm_fwk", &cnstr_id1);
	constraint_set(vdd1_opp_cons, get_opp_no(vdd1_opp_no));

	vdd2_opp_cons = constraint_get("pm_fwk", &cnstr_id2);
	constraint_set(vdd2_opp_cons, get_opp_no(vdd2_opp_no));

#ifdef CONFIG_CPU_IDLE
	pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY, "omap_pm_sys",
		omap3_power_states[cpuidle_deepest_st].sleep_latency +
		omap3_power_states[cpuidle_deepest_st].wakeup_latency + 1);
#endif

	/* Sysfs entry for setting opp for vdd1 */
	l = sysfs_create_file(power_kobj, &vdd1_opp_attr.attr);
	/* Sysfs entry for setting opp for vdd2 */
	l = sysfs_create_file(power_kobj, &vdd2_opp_attr.attr);
	/* Sysfs entry for setting arm frequency */
	l = sysfs_create_file(power_kobj, &arm_freq_attr.attr);
	/* Sysfs entry for setting dsp frequency */
	l = sysfs_create_file(power_kobj, &dsp_freq_attr.attr);
	/* Sysfs entry for enabling/disabling OFF mode support */
	l = sysfs_create_file(power_kobj, &pm_off_attr.attr);
	/* Sysfs entry for enabling/disabling debug support */
	l = sysfs_create_file(power_kobj, &debugflag_attr.attr);
	/* Sysfs entry for controlling LCD inactivity */
	l = sysfs_create_file(power_kobj, &fbtimeout_attr.attr);
	/* Sysfs entry for controlling OMAP overdrive mode */
	l = sysfs_create_file(power_kobj, &overdriveflag_attr.attr);
#ifdef CONFIG_CPU_IDLE
	/* Sysfs entry for setting cpuidle deepest state */
	l = sysfs_create_file(power_kobj, &cpuidle_deepest_state_attr.attr);
#endif
	if (l)
		printk(KERN_ERR "subsys_create_file failed: %d\n", l);
	return 0;
}

late_initcall(omap3_pm_sysfs_init);
