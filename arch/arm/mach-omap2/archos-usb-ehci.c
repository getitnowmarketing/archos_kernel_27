/*
 * linux/arch/arm/mach-omap2/board-omap3evm-usb.c
 *
 * Copyright (C) 2007 Texas Instruments
 * Author: Vikram Pandita
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/usb/musb.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/mach-types.h>

#include <mach/usb.h>
#include <mach/gpio.h>
#include <mach/archos-gpio.h>
#include <mach/mux.h>
#include <mach/board-archos.h>
#include <mach/usb-ehci.h>

#if     defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
static struct ehci_platform_data ehci_plat;
static struct resource ehci_resources[] = {
	[0] = {
		.start   = OMAP34XX_HSUSB_HOST_BASE + 0x800,
		.end     = OMAP34XX_HSUSB_HOST_BASE + 0x800 + SZ_1K - 1,
		.flags   = IORESOURCE_MEM,
	},
	[1] = {         /* general IRQ */
		.start   = INT_34XX_EHCI_IRQ,
		.flags   = IORESOURCE_IRQ,
	}
};

static u64 ehci_dmamask = ~(u32)0;
static struct platform_device ehci_device = {
	.name           = "ehci-omap",
	.id             = 0,
	.dev = {
		.dma_mask               = &ehci_dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.platform_data          = &ehci_plat,
	},
	.num_resources  = ARRAY_SIZE(ehci_resources),
	.resource       = ehci_resources,
};
#endif
/***************************************************/

static int ehci_phy_enable;

static struct archos_gpio gio_ehci_enable = UNUSED_GPIO;

int archos_enable_ehci( int enable )
{
	if ( GPIO_EXISTS( gio_ehci_enable ) ) {
		if ( enable )
			omap_set_gpio_dataout( GPIO_PIN( gio_ehci_enable ), 1);
		else
			omap_set_gpio_dataout( GPIO_PIN( gio_ehci_enable ), 0);
	}

	ehci_phy_enable = enable;
	
	return 0;
}
EXPORT_SYMBOL(archos_enable_ehci);

static int archos_ehci_probe(struct platform_device *dev)
{
	return 0;
}

static int archos_ehci_suspend(struct platform_device *dev, pm_message_t pm)
{
	printk("archos_ehci_suspend\n");
	omap_set_gpio_dataout( GPIO_PIN(gio_ehci_enable), 0);
	return 0;	
}

static int archos_ehci_resume(struct platform_device *dev)
{
	printk("archos_ehci_resume\n");
	omap_set_gpio_dataout( GPIO_PIN( gio_ehci_enable ), ehci_phy_enable);
	return 0;
}


static struct platform_driver archos_ehci_driver = {
	.driver.name = "archos_ehci",
	.probe = archos_ehci_probe,
#ifdef CONFIG_PM
	.suspend_late = archos_ehci_suspend,
	.resume_early = archos_ehci_resume,
#endif
};

static struct platform_device archos_ehci_device = {
	.name = "archos_ehci",
	.id = -1,
};

void __init usb_ehci_init(void)
{
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

	if ( GPIO_PIN (usb_cfg->rev[hardware_rev].enable_usb_ehci) != 0 ) {

		gio_ehci_enable = usb_cfg->rev[hardware_rev].enable_usb_ehci;

		GPIO_INIT_OUTPUT( gio_ehci_enable );

#if     defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
		ehci_plat.usb_enable_pin = GPIO_PIN( gio_ehci_enable );
		ehci_plat.usb_enable_delay = 10;
#endif
		/* reset the PHY */
		archos_enable_ehci( 0 );
//		udelay(2);
//		archos_enable_ehci( 1 );

		if ( machine_is_archos_a5h() || machine_is_archos_a5s() ||
		   machine_is_archos_a5hg() || machine_is_archos_a5sg() ||
		   machine_is_archos_a5hgw() || machine_is_archos_a5sgw() ||
		   machine_is_archos_a5sc() || machine_is_archos_a5st() ||
		   machine_is_archos_a5gcam()	) {
			/* pin mux for EHCI Port 1 */
			omap_cfg_reg(AE10_3430_USB1HS_PHY_CLK);
			omap_cfg_reg(AF10_3430_USB1HS_PHY_STP);
			omap_cfg_reg(AF9_3430_USB1HS_PHY_DIR);
			omap_cfg_reg(AG9_3430_USB1HS_PHY_NXT);
			omap_cfg_reg(AF11_3430_USB1HS_PHY_DATA0);
			omap_cfg_reg(AG12_3430_USB1HS_PHY_DATA1);
			omap_cfg_reg(AH12_3430_USB1HS_PHY_DATA2);
			omap_cfg_reg(AH14_3430_USB1HS_PHY_DATA3);
			omap_cfg_reg(AE11_3430_USB1HS_PHY_DATA4);
			omap_cfg_reg(AH9_3430_USB1HS_PHY_DATA5);
			omap_cfg_reg(AF13_3430_USB1HS_PHY_DATA6);
			omap_cfg_reg(AE13_3430_USB1HS_PHY_DATA7);
		} else {
			// gen6
			/* pin mux for EHCI Port 2 */
			omap_cfg_reg(AE7_3430_USB2HS_PHY_CLK);
			omap_cfg_reg(AF7_3430_USB2HS_PHY_STP);
			omap_cfg_reg(AG7_3430_USB2HS_PHY_DIR);
			omap_cfg_reg(AH7_3430_USB2HS_PHY_NXT);
			omap_cfg_reg(AG8_3430_USB2HS_PHY_DATA0);
			omap_cfg_reg(AH8_3430_USB2HS_PHY_DATA1);
			omap_cfg_reg(AB2_3430_USB2HS_PHY_DATA2);
			omap_cfg_reg(V3_3430_USB2HS_PHY_DATA3);
			omap_cfg_reg(Y2_3430_USB2HS_PHY_DATA4);
			omap_cfg_reg(Y3_3430_USB2HS_PHY_DATA5);
			omap_cfg_reg(Y4_3430_USB2HS_PHY_DATA6);
			omap_cfg_reg(AA3_3430_USB2HS_PHY_DATA7);
		}
	}

#if     defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
	if (platform_device_register(&ehci_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (EHCI) device\n");
		return;
	}
#endif

	if (platform_device_register(&archos_ehci_device) < 0)
		printk(KERN_ERR "Unable to register Archos EHCI device\n");
	
	platform_driver_register(&archos_ehci_driver);

}
