/*
 * linux/arch/arm/mach-omap2/archos-hsmmc.c
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
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/mmc.h>
#include <mach/board.h>
#include <mach/archos-gpio.h>
#include <mach/prcm.h>
#include <mach/mux.h>
#include <mach/board-archos.h>

#if defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

#define VMMC1_DEV_GRP		0x27
#define P1_DEV_GRP		0x20
#define VMMC1_DEDICATED		0x2A
#define VSEL_3V			0x02
#define VSEL_18V		0x00
#define LDO_CLR			0x00
#define VSEL_S2_CLR		0x40
#define GPIO_0_BIT_POS		(1 << 0)
#define GPIO_1_BIT_POS		(1 << 1)
#define VSIM_DEV_GRP		0x37
#define VSIM_DEDICATED		0x3A
#define VMMC2_DEV_GRP		0x2B
#define VMMC2_DEDICATED		0x2E

#define OMAP2_CONTROL_DEVCONF0	0x48002274
#define OMAP2_CONTROL_DEVCONF1	0x490022E8
#define OMAP3_CONTROL_DEVCONF1	0x480022D8

#define OMAP2_CONTROL_DEVCONF0_LBCLK	(1 << 24)
#define OMAP2_CONTROL_DEVCONF1_ACTOV	(1 << 31)

#define OMAP2_CONTROL_PBIAS_VMODE	(1 << 0)
#define OMAP2_CONTROL_PBIAS_VMODE1	(1 << 8)
#define OMAP2_CONTROL_PBIAS_PWRDNZ	(1 << 1)
#define OMAP2_CONTROL_PBIAS_SCTRL	(1 << 2)
#define OMAP2_CONTROL_PBIAS_PWRDNZ1	(1 << 9)
#define OMAP2_CONTROL_PBIAS_SCTRL1	(1 << 10)

static struct archos_gpio gpio_hsmmc_power;
static struct archos_gpio gpio_hsmmc_detect;
static int detect_irq;


void archos_mmc_set_power(int enable)
{
	omap_set_gpio_dataout( GPIO_PIN( gpio_hsmmc_power ), enable ? 1 : 0 );
}

EXPORT_SYMBOL(archos_mmc_set_power);

#ifdef CONFIG_OMAP_HS_MMC1
static int hsmmc_card_detect(int irq)
{
	int gpio;
	gpio = !gpio_get_value_cansleep( GPIO_PIN( gpio_hsmmc_detect ) );
	/* GPIO is set to 1 when there is no card, and to 0 when a card is present... */
	return gpio;
}
#endif

#ifdef CONFIG_OMAP_HS_MMC2
extern int mmc2_card_inserted;

static int hsmmc2_card_detect(int irq)
{
	return mmc2_card_inserted;
}
#endif

/*
 * MMC Slot Initialization.
 */
#ifdef CONFIG_OMAP_HS_MMC1
static int hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	omap_set_gpio_debounce(GPIO_PIN( gpio_hsmmc_detect ),1);
	return ret;
}

static void hsmmc_cleanup(struct device *dev)
{
}
#endif

#ifdef CONFIG_OMAP_HS_MMC2
static int hsmmc2_late_init(struct device *dev)
{
	int ret = 0;
	return ret;
}

static void hsmmc2_cleanup(struct device *dev)
{
}
#endif

#ifdef CONFIG_PM

#ifdef CONFIG_OMAP_HS_MMC1
static int hsmmc_suspend(struct device *dev, int slot)
{
	int ret = 0;
	disable_irq(GPIO_PIN( gpio_hsmmc_detect ));
	return ret;
}

static int hsmmc_resume(struct device *dev, int slot)
{
	int ret = 0;
	enable_irq(GPIO_PIN( gpio_hsmmc_detect ));
	return ret;
}
#endif

#ifdef CONFIG_OMAP_HS_MMC2
static int hsmmc2_suspend(struct device *dev, int slot)
{
	int ret = 0;
	return ret;
}

static int hsmmc2_resume(struct device *dev, int slot)
{
	int ret = 0;
	return ret;
}
#endif

#endif

#ifdef CONFIG_OMAP_HS_MMC1
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
		archos_mmc_set_power(1);
		msleep(100);

		if (cpu_is_omap24xx())
			devconf = omap_readl(OMAP2_CONTROL_DEVCONF1);
		else
			devconf = omap_readl(OMAP2_CONTROL_DEVCONF0);

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
				    OMAP2_CONTROL_DEVCONF0);

		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg |= OMAP2_CONTROL_PBIAS_SCTRL;
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		reg = omap_readl(OMAP2_CONTROL_PBIAS);
		reg &= ~OMAP2_CONTROL_PBIAS_PWRDNZ;
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		if (!machine_has_micro_sd() ) {
			reg = omap_readl(OMAP2_CONTROL_PBIAS);
			reg |= OMAP2_CONTROL_PBIAS_SCTRL1;
			omap_writel(reg, OMAP2_CONTROL_PBIAS);

			reg = omap_readl(OMAP2_CONTROL_PBIAS);
			reg &= ~OMAP2_CONTROL_PBIAS_PWRDNZ1;
			omap_writel(reg, OMAP2_CONTROL_PBIAS);
		}
		/* ToDo */

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

		/* ToDo */
		
		/* 100ms delay required for PBIAS configuration */
		msleep(100);
		reg = omap_readl(OMAP2_CONTROL_PBIAS);

		reg |= (OMAP2_CONTROL_PBIAS_VMODE |
			OMAP2_CONTROL_PBIAS_PWRDNZ |
			OMAP2_CONTROL_PBIAS_SCTRL);

		if (!machine_has_micro_sd() ) {
			reg |= (OMAP2_CONTROL_PBIAS_VMODE1 |
				OMAP2_CONTROL_PBIAS_PWRDNZ1 |
				OMAP2_CONTROL_PBIAS_SCTRL1);
		}
		omap_writel(reg, OMAP2_CONTROL_PBIAS);

		archos_mmc_set_power(0);
		msleep(100);
	}

	return 0;
}
#endif

#ifdef CONFIG_OMAP_HS_MMC2
static int hsmmc2_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	u32 vdd_sel = 0, devconf = 0;
	int ret = 0;

	if (power_on) {
		devconf = omap_readl(OMAP3_CONTROL_DEVCONF1);

		switch (1 << vdd) {
		case MMC_VDD_33_34:
		case MMC_VDD_32_33:
			break;
		case MMC_VDD_165_195:
			break;
		}

		omap_writel(devconf | 1 << 6, OMAP3_CONTROL_DEVCONF1);

		/* ToDo */
		
		return ret;

	} else {
		/* ToDo */

		return ret;
	}
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

#ifdef CONFIG_OMAP_HS_MMC1
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
		.wire4			= 1,
		.set_power		= hsmmc_set_power,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34 |
						MMC_VDD_165_195,
		.name			= "first slot",

		.card_detect_irq        = 0,
		.card_detect            = hsmmc_card_detect,
	},
};
#endif

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
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195,
		.name			= "first slot",

		.card_detect_irq        = 0,
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

int __init hsmmc_init(void)
{
	const struct archos_sd_config *hsmmc_cfg;
	
	/* sd */
	hsmmc_cfg = omap_get_config( ARCHOS_TAG_SD, struct archos_sd_config );
	if (hsmmc_cfg == NULL) {
		printk(KERN_DEBUG "archos_hsmmc_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= hsmmc_cfg->nrev ) {
		printk(KERN_DEBUG "archos_hsmmc_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, hsmmc_cfg->nrev);
		return -ENODEV;
	}
	
	printk(KERN_DEBUG "archos_hsmmc_init\n");

	gpio_hsmmc_power = hsmmc_cfg->rev[hardware_rev].sd_power;
	GPIO_INIT_OUTPUT( gpio_hsmmc_power );

#ifdef CONFIG_OMAP_HS_MMC1
	gpio_hsmmc_detect = hsmmc_cfg->rev[hardware_rev].sd_detect;
	detect_irq = gpio_to_irq( GPIO_PIN( gpio_hsmmc_detect ) );
	mmc1_data.slots[0].card_detect_irq = detect_irq;

	/* Need to be here, before omap2_init_mmc which will correctly set the IRQ stuff */
	GPIO_INIT_INPUT( gpio_hsmmc_detect );
#endif

#ifdef CONFIG_OMAP_HS_MMC1
	hsmmc_data[0] = &mmc1_data;
#endif
#ifdef CONFIG_OMAP_HS_MMC2
	hsmmc_data[1] = &mmc2_data;
#endif
#ifdef CONFIG_OMAP_HS_MMC3
	hsmmc_data[2] = &mmc3_data;
#endif
	omap2_init_mmc(hsmmc_data, OMAP34XX_NR_MMC);
	return 0;
}

#else

int __init hsmmc_init(void)
{
	return 0;
}

#endif
