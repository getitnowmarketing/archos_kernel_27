/*
 * linux/arch/arm/mach-omap2/board-sdp-hsmmc.c
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c/twl4030.h>

#include <mach/control.h>
#include <mach/hardware.h>
#include <mach/mmc.h>
#include <mach/board.h>
#include <mach/resource.h>

#if defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

#define VMMC1_DEV_GRP		0x27
#define P1_DEV_GRP		0x20
#define VMMC1_DEDICATED		0x2A
#define VSEL_3V			0x02
#define VSEL_18V		0x00
#define TWL_GPIO_IMR1A		0x1C
#define TWL_GPIO_ISR1A		0x19
#define LDO_CLR			0x00
#define VSEL_S2_CLR		0x40
#define GPIO_0_BIT_POS		(1 << 0)
#define GPIO_1_BIT_POS		(1 << 1)
#define VSIM_DEV_GRP		0x37
#define VSIM_DEDICATED		0x3A
#define VMMC2_DEV_GRP		0x2B
#define VMMC2_DEDICATED		0x2E

struct res_handle *rhandlemmc1;
struct res_handle *rhandlemmc2;
struct res_handle *rhandlevsim;

#define OMAP2_CONTROL_DEVCONF1		(OMAP243X_CTRL_BASE \
					+ OMAP243X_CONTROL_DEVCONF1)
#define OMAP3_CONTROL_DEVCONF0		(OMAP343X_CTRL_BASE \
					+ OMAP343X_CONTROL_DEVCONF0)
#define OMAP3_CONTROL_DEVCONF1		(OMAP343X_CTRL_BASE \
					+ OMAP343X_CONTROL_DEVCONF1)

#define OMAP2_CONTROL_DEVCONF0_LBCLK	(1 << 24)
#define OMAP2_CONTROL_DEVCONF1_ACTOV	(1 << 31)

#define OMAP2_CONTROL_PBIAS_VMODE	(1 << 0)
#define OMAP2_CONTROL_PBIAS_VMODE1	(1 << 8)
#define OMAP2_CONTROL_PBIAS_PWRDNZ	(1 << 1)
#define OMAP2_CONTROL_PBIAS_SCTRL	(1 << 2)
#define OMAP2_CONTROL_PBIAS_PWRDNZ1	(1 << 9)
#define OMAP2_CONTROL_PBIAS_SCTRL1	(1 << 10)


static const int mmc1_cd_gpio = OMAP_MAX_GPIO_LINES;		/* HACK!! */
static const int mmc2_cd_gpio = OMAP_MAX_GPIO_LINES + 1;

static int hsmmc_card_detect(int irq)
{
	return gpio_get_value_cansleep(mmc1_cd_gpio);
}

#ifdef CONFIG_OMAP_HS_MMC2
static int hsmmc2_card_detect(int irq)
{
	return gpio_get_value_cansleep(mmc2_cd_gpio);
}
#endif

/*
 * MMC Slot Initialization.
 */
static int hsmmc_late_init(struct device *dev)
{
	int ret = 0;

	/*
	 * Configure TWL4030 GPIO parameters for MMC hotplug irq
	 */
	ret = gpio_request(mmc1_cd_gpio, "mmc0_cd");
	if (ret)
		goto err;

	ret = twl4030_set_gpio_debounce(0, true);
	if (ret)
		goto err;

	return ret;

err:
	dev_err(dev, "Failed to configure TWL4030 GPIO IRQ\n");
	return ret;
}

static void hsmmc_cleanup(struct device *dev)
{
	gpio_free(mmc1_cd_gpio);
}

#ifdef CONFIG_OMAP_HS_MMC2
static int hsmmc2_late_init(struct device *dev)
{
	int ret = 0;

	/*
	 * Configure TWL4030 GPIO parameters for MMC2 hotplug irq
	 */
	ret = gpio_request(mmc2_cd_gpio, "mmc1_cd");
	if (ret)
		goto err;

	ret = twl4030_set_gpio_debounce(1, true);
	if (ret)
		goto err;

	return ret;

err:
	dev_err(dev, "Failed to configure TWL4030 GPIO IRQ for MMC2\n");
	return ret;
}

static void hsmmc2_cleanup(struct device *dev)
{
	gpio_free(mmc2_cd_gpio);
}
#endif

#ifdef CONFIG_PM

/*
 * To mask and unmask MMC Card Detect Interrupt
 * mask : 1
 * unmask : 0
 */
static int mask_cd_interrupt(int mask)
{
	u8 reg = 0, ret = 0;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg, TWL_GPIO_IMR1A);
	if (ret)
		goto err;

	reg = (mask == 1) ? (reg | GPIO_0_BIT_POS) : (reg & ~GPIO_0_BIT_POS);

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, reg, TWL_GPIO_IMR1A);
	if (ret)
		goto err;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg, TWL_GPIO_ISR1A);
	if (ret)
		goto err;

	reg = (mask == 1) ? (reg | GPIO_0_BIT_POS) : (reg & ~GPIO_0_BIT_POS);

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, reg, TWL_GPIO_ISR1A);
	if (ret)
		goto err;

err:
	return ret;
}

static int hsmmc_suspend(struct device *dev, int slot)
{
	int ret = 0;

	disable_irq(TWL4030_GPIO_IRQ_NO(0));
	ret = mask_cd_interrupt(1);

	return ret;
}

static int hsmmc_resume(struct device *dev, int slot)
{
	int ret = 0;

	enable_irq(TWL4030_GPIO_IRQ_NO(0));
	ret = mask_cd_interrupt(0);

	return ret;
}

#ifdef CONFIG_OMAP_HS_MMC2
/*
 * To mask and unmask MMC Card Detect Interrupt
 * mask : 1
 * unmask : 0
 */
static int mask_cd2_interrupt(int mask)
{
	u8 reg = 0, ret = 0;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg, TWL_GPIO_IMR1A);
	if (ret != 0)
		goto err;

	reg = (mask == 1) ? (reg | GPIO_1_BIT_POS) : (reg & ~GPIO_1_BIT_POS);

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, reg, TWL_GPIO_IMR1A);
	if (ret != 0)
		goto err;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg, TWL_GPIO_ISR1A);
	if (ret != 0)
		goto err;

	reg = (mask == 1) ? (reg | GPIO_1_BIT_POS) : (reg & ~GPIO_1_BIT_POS);

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, reg, TWL_GPIO_ISR1A);
	if (ret != 0)
		goto err;
err:
	return ret;
}

static int hsmmc2_suspend(struct device *dev, int slot)
{
	int ret = 0;

	disable_irq(TWL4030_GPIO_IRQ_NO(1));
	ret = mask_cd2_interrupt(1);

	return ret;
}

static int hsmmc2_resume(struct device *dev, int slot)
{
	int ret = 0;

	enable_irq(TWL4030_GPIO_IRQ_NO(1));
	ret = mask_cd2_interrupt(0);

	return ret;
}
#endif

#endif

static int hsmmc_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	u32 vdd_sel = 0, devconf = 0, reg = 0;
	int ret = 0;

	/* REVISIT: Using address directly till the control.h defines
	 * are settled.
	 */
#if defined(CONFIG_ARCH_OMAP2430)
	#define OMAP2_CONTROL_PBIAS 0x490024A0
#else
	#define OMAP2_CONTROL_PBIAS 0x48002520
#endif

	if (power_on) {
		if (cpu_is_omap24xx())
			devconf = omap_readl(OMAP2_CONTROL_DEVCONF1);
		else
			devconf = omap_readl(OMAP3_CONTROL_DEVCONF0);

		switch (1 << vdd) {
		case MMC_VDD_33_34:
		case MMC_VDD_32_33:
			vdd_sel = VSEL_3V;
			if (cpu_is_omap24xx())
				devconf |= OMAP2_CONTROL_DEVCONF1_ACTOV;
			break;
		case MMC_VDD_165_195:
			vdd_sel = VSEL_18V;
			if (cpu_is_omap24xx())
				devconf &= ~OMAP2_CONTROL_DEVCONF1_ACTOV;
		}

		if (cpu_is_omap24xx())
			omap_writel(devconf, OMAP2_CONTROL_DEVCONF1);
		else
			omap_writel(devconf | OMAP2_CONTROL_DEVCONF0_LBCLK,
				    OMAP3_CONTROL_DEVCONF0);

		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg |= OMAP2_CONTROL_PBIAS_SCTRL;
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg &= ~OMAP2_CONTROL_PBIAS_PWRDNZ;
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg |= OMAP2_CONTROL_PBIAS_SCTRL1;
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg &= ~OMAP2_CONTROL_PBIAS_PWRDNZ1;
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		ret = resource_request(rhandlemmc1, (vdd_sel == VSEL_3V ?
			T2_VMMC1_3V00 : T2_VMMC1_1V85));
		if (ret != 0)
			goto err;

		/* Enable VSIM to support MMC 8-bit on ES2 */
		if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0)) {
			ret = resource_request(rhandlevsim,
				(vdd_sel == VSEL_3V ?
				T2_VSIM_3V00 : T2_VSIM_1V80));
			if (ret != 0)
				return ret;
		}
		msleep(100);
		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg = (vdd_sel == VSEL_18V) ?
			(((reg | 0x0606) & ~0x1) & ~(1<<8))
			: (reg | 0x0707);
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		return ret;

	} else {
		/* Power OFF */

		/* For MMC1, Toggle PBIAS before every power up sequence */
		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg &= ~OMAP2_CONTROL_PBIAS_PWRDNZ;
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		if (rhandlemmc1 != NULL) {
			ret = resource_release(rhandlemmc1);
			if (ret != 0)
				goto err;
		}

		if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0)
					 && (rhandlevsim != NULL)) {
			ret = resource_release(rhandlevsim);
			if (ret != 0)
				goto err;
		}

		if (is_sil_rev_equal_to(OMAP3430_REV_ES3_0)) {
			ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				LDO_CLR, VSIM_DEV_GRP);
			if (ret)
				goto err;
			ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				LDO_CLR, VSIM_DEDICATED);
			if (ret)
				goto err;
		}

		/* 100ms delay required for PBIAS configuration */
		msleep(100);
		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg |= (OMAP2_CONTROL_PBIAS_VMODE |
			OMAP2_CONTROL_PBIAS_VMODE1 |
			OMAP2_CONTROL_PBIAS_PWRDNZ |
			OMAP2_CONTROL_PBIAS_PWRDNZ1 |
			OMAP2_CONTROL_PBIAS_SCTRL |
			OMAP2_CONTROL_PBIAS_SCTRL1);
		omap_writel(reg, OMAP2_CONTROL_PBIAS);
	}

	return 0;

err:
	return 1;
}

#ifdef CONFIG_OMAP_HS_MMC2
static int hsmmc2_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	u32 vdd_sel = 0, devconf = 0;
	int ret = 0;

	if (power_on) {
		if (cpu_is_omap24xx())
			devconf = omap_readl(OMAP2_CONTROL_DEVCONF1);
		else
			devconf = omap_readl(OMAP3_CONTROL_DEVCONF1);

		switch (1 << vdd) {
		case MMC_VDD_33_34:
		case MMC_VDD_32_33:
			vdd_sel = VSEL_3V;
			if (cpu_is_omap24xx())
				devconf = (devconf | (1 << 31));
			break;
		case MMC_VDD_165_195:
			vdd_sel = VSEL_18V;
			if (cpu_is_omap24xx())
				devconf = (devconf & ~(1 << 31));
		}

		if (cpu_is_omap24xx())
			omap_writel(devconf, OMAP2_CONTROL_DEVCONF1);
		else
			omap_writel(devconf | 1 << 6, OMAP3_CONTROL_DEVCONF1);

		ret = resource_request(rhandlemmc2, vdd_sel);
		if (ret != 0)
			goto err;

		return ret;

	} else {
		if (rhandlemmc2 != NULL) {
			ret = resource_release(rhandlemmc2);
			if (ret != 0)
				goto err;
		}
		return ret;
	}
err:
	return 1;
}
#endif

#ifdef CONFIG_OMAP_HS_MMC3
static int hsmmc3_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	/* Power to the slot is hard wired */
	return 0;
}
#endif

static struct omap_mmc_platform_data mmc1_data = {
	.nr_slots			= 1,
	.init				= hsmmc_late_init,
	.cleanup			= hsmmc_cleanup,
#ifdef CONFIG_PM
	.suspend			= hsmmc_suspend,
	.resume				= hsmmc_resume,
#endif
	.dma_mask			= 0xffffffff,
	.slots[0] = {
		.wire8			= 1,
		.set_power		= hsmmc_set_power,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34 |
						MMC_VDD_165_195,
		.name			= "first slot",

		.card_detect_irq        = TWL4030_GPIO_IRQ_NO(0),
		.card_detect            = hsmmc_card_detect,
	},
};

#ifdef CONFIG_OMAP_HS_MMC2
static struct omap_mmc_platform_data mmc2_data = {
	.nr_slots			= 1,
	.init				= hsmmc2_late_init,
	.cleanup			= hsmmc2_cleanup,
#ifdef CONFIG_PM
	.suspend			= hsmmc2_suspend,
	.resume				= hsmmc2_resume,
#endif
	.dma_mask			= 0xffffffff,
	.slots[0] = {
		.wire4			= 1,
		.set_power		= hsmmc2_set_power,
		.ocr_mask		= MMC_VDD_165_195,
		.name			= "first slot",

		.card_detect_irq        = TWL4030_GPIO_IRQ_NO(1),
		.card_detect            = hsmmc2_card_detect,
	},
};
#endif

#ifdef CONFIG_OMAP_HS_MMC3
static struct omap_mmc_platform_data mmc3_data = {
	.nr_slots			= 1,
	.init				= NULL,
	.cleanup			= NULL,
	.dma_mask			= 0xffffffff,
	.slots[0] = {
		.wire4			= 1,
		.set_power		= hsmmc3_set_power,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34,
		.name			= "first slot",

		.card_detect_irq        = 0,
		.card_detect            = NULL,
	},
};
#endif

static struct omap_mmc_platform_data *hsmmc_data[OMAP34XX_NR_MMC];

void __init hsmmc_init(void)
{
	rhandlemmc1 = resource_get("hsmmc1", "t2_vmmc1");
	rhandlemmc2 = resource_get("hsmmc2", "t2_vmmc2");
	rhandlevsim = resource_get("vsim", "t2_vsim");

	hsmmc_data[0] = &mmc1_data;
#ifdef CONFIG_OMAP_HS_MMC2
	hsmmc_data[1] = &mmc2_data;
#endif
#ifdef CONFIG_OMAP_HS_MMC3
	hsmmc_data[2] = &mmc3_data;
#endif
	omap2_init_mmc(hsmmc_data, OMAP34XX_NR_MMC);
}

#else

void __init hsmmc_init(void)
{

}

#endif
