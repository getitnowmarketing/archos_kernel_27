/*
 * SMS/SDRC (SDRAM controller) common code for OMAP2/3
 *
 * Copyright (C) 2005, 2008 Texas Instruments Inc.
 * Copyright (C) 2005, 2008 Nokia Corporation
 *
 * Tony Lindgren <tony@atomide.com>
 * Paul Walmsley
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#undef DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/sysdev.h>

#include <mach/common.h>
#include <mach/clock.h>
#include <mach/sram.h>
#include <mach/control.h>

#include "prm.h"

#include <mach/sdrc.h>
#include "sdrc.h"

static struct omap_sdrc_params *sdrc_init_params;

void __iomem *omap2_sdrc_base;
void __iomem *omap2_sms_base;

static struct sysdev_class sdrc_sysclass = {
	.name = "sdrc",
};

static struct sys_device sdrc_sysdev = {
	.id = 0,
	.cls = &sdrc_sysclass,
};

/* low data bits drive strength */
static ssize_t show_lowdata_drivestrength(struct sys_device *dev, 
		struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", !!(omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO0) & (1UL<<31)));
}
static ssize_t store_lowdata_drivestrength(struct sys_device *dev, 
		struct sysdev_attribute *attr, const char *buf, size_t count)
{
	int low_high = !!simple_strtol(buf, NULL, 10);

	omap_ctrl_writel( (omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO0) & ~(1UL<<31)) | (low_high << 31),
			OMAP343X_CONTROL_PROG_IO0);
	
	return count;
}
static struct sysdev_attribute attr_lowdata_drivestrength = {
	.attr = { .name = "lowdata_drivestrength", .mode = S_IRUGO|S_IWUSR },
	.show = show_lowdata_drivestrength,
	.store = store_lowdata_drivestrength,
};

/* high data bits drive strength */
static ssize_t show_highdata_drivestrength(struct sys_device *dev, 
		struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", !!(omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO0) & (1UL<<30)));
}
static ssize_t store_highdata_drivestrength(struct sys_device *dev, 
		struct sysdev_attribute *attr, const char *buf, size_t count)
{
	int low_high = !!simple_strtol(buf, NULL, 10);

	omap_ctrl_writel( (omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO0) & ~(1UL<<30)) | (low_high << 30),
			OMAP343X_CONTROL_PROG_IO0);
	
	return count;
}
static struct sysdev_attribute attr_highdata_drivestrength = {
	.attr = { .name = "highdata_drivestrength", .mode = S_IRUGO|S_IWUSR },
	.show = show_highdata_drivestrength,
	.store = store_highdata_drivestrength,
};

static ssize_t show_addrctrl_drivestrength(struct sys_device *dev,
		struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", !!(omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO0) & (1UL<<29)));
}
static ssize_t store_addrctrl_drivestrength(struct sys_device *dev, 
		struct sysdev_attribute *attr, const char *buf, size_t count)
{
	int low_high = !!simple_strtol(buf, NULL, 10);

	omap_ctrl_writel( (omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO0) & ~(1UL<<29)) | (low_high << 29),
			OMAP343X_CONTROL_PROG_IO0);
	
	return count;
}
static struct sysdev_attribute attr_addrctrl_drivestrength = {
	.attr = { .name = "addrctrl_drivestrength", .mode = S_IRUGO|S_IWUSR },
	.show = show_addrctrl_drivestrength,
	.store = store_addrctrl_drivestrength,
};

static ssize_t show_ncs0_drivestrength(struct sys_device *dev, 
		struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", !!(omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO0) & (1UL<<28)));
}
static ssize_t store_ncs0_drivestrength(struct sys_device *dev, 
		struct sysdev_attribute *attr, const char *buf, size_t count)
{
	int low_high = !!simple_strtol(buf, NULL, 10);

	omap_ctrl_writel( (omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO0) & ~(1UL<<28)) | (low_high << 28),
			OMAP343X_CONTROL_PROG_IO0);
	
	return count;
}
static struct sysdev_attribute attr_ncs0_drivestrength = {
	.attr = { .name = "ncs0_drivestrength", .mode = S_IRUGO|S_IWUSR },
	.show = show_ncs0_drivestrength,
	.store = store_ncs0_drivestrength,
};

static ssize_t show_ncs1_drivestrength(struct sys_device *dev,
		struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", !!(omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO0) & (1UL<<27)));
}
static ssize_t store_ncs1_drivestrength(struct sys_device *dev, 
		struct sysdev_attribute *attr, const char *buf, size_t count)
{
	int low_high = !!simple_strtol(buf, NULL, 10);

	omap_ctrl_writel( (omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO0) & ~(1UL<<27)) | (low_high << 27),
			OMAP343X_CONTROL_PROG_IO0);
	
	return count;
}
static struct sysdev_attribute attr_ncs1_drivestrength = {
	.attr = { .name = "ncs1_drivestrength", .mode = S_IRUGO|S_IWUSR },
	.show = show_ncs1_drivestrength,
	.store = store_ncs1_drivestrength,
};

/**
 * omap2_sdrc_get_params - return SDRC register values for a given clock rate
 * @r: SDRC clock rate (in Hz)
 *
 * Return pre-calculated values for the SDRC_ACTIM_CTRLA,
 * SDRC_ACTIM_CTRLB, SDRC_RFR_CTRL, and SDRC_MR registers, for a given
 * SDRC clock rate 'r'.  These parameters control various timing
 * delays in the SDRAM controller that are expressed in terms of the
 * number of SDRC clock cycles to wait; hence the clock rate
 * dependency. Note that sdrc_init_params must be sorted rate
 * descending.  Also assumes that both chip-selects use the same
 * timing parameters.  Returns a struct omap_sdrc_params * upon
 * success, or NULL upon failure.
 */
struct omap_sdrc_params *omap2_sdrc_get_params(unsigned long r)
{
	struct omap_sdrc_params *sp;

	sp = sdrc_init_params;

	while (sp->rate != r)
		sp++;

	if (!sp->rate)
		return NULL;

	return sp;
}


void __init omap2_set_globals_sdrc(struct omap_globals *omap2_globals)
{
	omap2_sdrc_base = omap2_globals->sdrc;
	omap2_sms_base = omap2_globals->sms;
}

/* turn on smart idle modes for SDRAM scheduler and controller */
void __init omap2_sdrc_init(struct omap_sdrc_params *sp)
{
	u32 l;

	l = sms_read_reg(SMS_SYSCONFIG);
	l &= ~(0x3 << 3);
	l |= (0x2 << 3);
	sms_write_reg(l, SMS_SYSCONFIG);

	l = sdrc_read_reg(SDRC_SYSCONFIG);
	l &= ~(0x3 << 3);
	l |= (0x2 << 3);
	sdrc_write_reg(l, SDRC_SYSCONFIG);

	sdrc_init_params = sp;
}

static int __init omap2_sdrc_sysdev_init(void)
{
	sysdev_class_register(&sdrc_sysclass);
	
	sysdev_register(&sdrc_sysdev);
	sysdev_create_file(&sdrc_sysdev, &attr_lowdata_drivestrength);
	sysdev_create_file(&sdrc_sysdev, &attr_highdata_drivestrength);
	sysdev_create_file(&sdrc_sysdev, &attr_addrctrl_drivestrength);
	sysdev_create_file(&sdrc_sysdev, &attr_ncs0_drivestrength);
	sysdev_create_file(&sdrc_sysdev, &attr_ncs1_drivestrength);
	return 0;
}

arch_initcall(omap2_sdrc_sysdev_init);
