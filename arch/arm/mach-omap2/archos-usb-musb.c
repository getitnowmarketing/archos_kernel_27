/*
 * linux/arch/arm/mach-omap2/usb-musb.c
 *
 * This file will contain the board specific details for the
 * MENTOR USB OTG controller on OMAP3430
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Vikram Pandita
 *
 * Generalization by:
 * Felipe Balbi <felipe.balbi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>

#include <linux/usb/musb.h>
#include <linux/usb/otg.h>

#include <mach/hardware.h>
#include <mach/pm.h>
#include <mach/mux.h>
#include <mach/usb.h>
#include <mach/gpio.h>
#include <mach/board-archos.h>

#ifdef CONFIG_USB_MUSB_SOC
static struct resource musb_resources[] = {
	[0] = {
		.start	= cpu_is_omap34xx()
			? OMAP34XX_HSUSB_OTG_BASE
			: OMAP243X_HS_BASE,
		.end	= cpu_is_omap34xx()
			? OMAP34XX_HSUSB_OTG_BASE + SZ_8K - 1
			: OMAP243X_HS_BASE + SZ_8K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {	/* general IRQ */
		.start	= INT_243X_HS_USB_MC,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {	/* DMA IRQ */
		.start	= INT_243X_HS_USB_DMA,
		.flags	= IORESOURCE_IRQ,
	},
};

static int clk_on;
static struct clk *phy_clk;
static struct archos_gpio gpio_phy_enable = UNUSED_GPIO;

static void phy_enable( void )
{
	clk_enable(phy_clk);
	mdelay(1);
	if (GPIO_EXISTS(gpio_phy_enable))
		omap_set_gpio_dataout(GPIO_PIN(gpio_phy_enable), 1);
} 

static void phy_disable( void )
{
	if (GPIO_EXISTS(gpio_phy_enable))
		omap_set_gpio_dataout(GPIO_PIN(gpio_phy_enable), 0);
	mdelay(1);
	clk_disable(phy_clk);
}

static int musb_set_clock(struct clk *clk, int state)
{
	if (state) {
		if (clk_on > 0)
			return -ENODEV;

		phy_enable();
		clk_enable(clk);
		clk_on = 1;
	} else {
		if (clk_on == 0)
			return -ENODEV;

		clk_disable(clk);
		phy_disable();
		clk_on = 0;
	}

	return 0;
}

static struct musb_hdrc_eps_bits musb_eps[] = {
	{	"ep1_tx", 10,	},
	{	"ep1_rx", 10,	},
	{	"ep2_tx", 9,	},
	{	"ep2_rx", 9,	},
	{	"ep3_tx", 3,	},
	{	"ep3_rx", 3,	},
	{	"ep4_tx", 3,	},
	{	"ep4_rx", 3,	},
	{	"ep5_tx", 3,	},
	{	"ep5_rx", 3,	},
	{	"ep6_tx", 3,	},
	{	"ep6_rx", 3,	},
	{	"ep7_tx", 3,	},
	{	"ep7_rx", 3,	},
	{	"ep8_tx", 2,	},
	{	"ep8_rx", 2,	},
	{	"ep9_tx", 2,	},
	{	"ep9_rx", 2,	},
	{	"ep10_tx", 2,	},
	{	"ep10_rx", 2,	},
	{	"ep11_tx", 2,	},
	{	"ep11_rx", 2,	},
	{	"ep12_tx", 2,	},
	{	"ep12_rx", 2,	},
	{	"ep13_tx", 2,	},
	{	"ep13_rx", 2,	},
	{	"ep14_tx", 2,	},
	{	"ep14_rx", 2,	},
	{	"ep15_tx", 2,	},
	{	"ep15_rx", 2,	},
};

static struct musb_hdrc_config musb_config = {
	.multipoint	= 1,
	.dyn_fifo	= 1,
	.soft_con	= 1,
	.dma		= 1,
	.num_eps	= 16,
	.dma_channels	= 7,
	.dma_req_chan	= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3),
	.ram_bits	= 12,
	.eps_bits	= musb_eps,
};

static struct musb_hdrc_platform_data musb_plat = {
#ifdef CONFIG_USB_MUSB_OTG
	.mode		= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD) || defined(CONFIG_USB_MUSB_DUAL_ROLE)
	.mode		= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode		= MUSB_PERIPHERAL,
#endif
	.clock		= cpu_is_omap34xx()
			? "hsotgusb_ick"
			: "usbhs_ick",
	.set_clock	= musb_set_clock,
	.config		= &musb_config,

	/* REVISIT charge pump on TWL4030 can supply up to
	 * 100 mA ... but this value is board-specific, like
	 * "mode", and should be passed to usb_musb_init().
	 */
	.power		= 250,			/* up to 500 mA */
};

static u64 musb_dmamask = DMA_32BIT_MASK;

static struct platform_device musb_device = {
	.name		= "musb_hdrc",
	.id		= -1,
	.dev = {
		.dma_mask		= &musb_dmamask,
		.coherent_dma_mask	= DMA_32BIT_MASK,
		.platform_data		= &musb_plat,
	},
	.num_resources	= ARRAY_SIZE(musb_resources),
	.resource	= musb_resources,
};
#endif

static struct archos_gpio gpio_usb_id = UNUSED_GPIO;

int archos_set_usb_id( int enable )
{
	if (GPIO_EXISTS(gpio_usb_id)) {
		if (enable) 
			omap_set_gpio_dataout( GPIO_PIN( gpio_usb_id ), 1);
		else
			omap_set_gpio_dataout( GPIO_PIN( gpio_usb_id ), 0);
	}

	return 0;
} 
EXPORT_SYMBOL(archos_set_usb_id);

static int xceiv_set_power(struct otg_transceiver *otg, unsigned int mA)
{
	printk("xceiv_set_power %dmA\n", mA);
	return 0;
}

#ifdef CONFIG_USB_MUSB_SOC
static struct otg_transceiver musb_xceiv = {
	.dev = &musb_device.dev,
	.label = "SMSC3322",
	.set_power = xceiv_set_power,
};
#endif

void __init usb_musb_init(void)
{
#ifdef CONFIG_USB_MUSB_SOC
	const struct archos_usb_config *usb_cfg;
	usb_cfg = omap_get_config( ARCHOS_TAG_USB, struct archos_usb_config );
	if (usb_cfg == NULL) {
		printk(KERN_DEBUG "archos_usb_init: no board configuration found\n");
		return;
	}
	if ( hardware_rev >= usb_cfg->nrev ) {
		printk(KERN_DEBUG "archos_usb_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, usb_cfg->nrev);
		return;
	}

	/* ground USB ID pin */
	gpio_usb_id = usb_cfg->rev[hardware_rev].usb_id;
	if (GPIO_EXISTS(gpio_usb_id)) {
		GPIO_INIT_OUTPUT( gpio_usb_id );
		omap_set_gpio_dataout( GPIO_PIN( gpio_usb_id ), 0 );
	}

	/* keep PHY in reset to save power... */	
	gpio_phy_enable = usb_cfg->rev[hardware_rev].enable_usb_musb;
	if (GPIO_EXISTS(gpio_phy_enable)) {
		GPIO_INIT_OUTPUT( gpio_phy_enable );
		omap_set_gpio_dataout( GPIO_PIN ( gpio_phy_enable ), 0);
	}

	/* sys_clkout1 is needed by the phy */
	phy_clk = clk_get(NULL, "sys_clkout1");
	if (IS_ERR(phy_clk)) {
		/* No need to proceed, PHY will not work without clock */
		pr_err("usb_musb_init: cannot get PHY external clock\n");
		return;
	}

	if (platform_device_register(&musb_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (MUSB) device\n");
		return;
	}
	
	otg_set_transceiver(&musb_xceiv);
#endif
}
