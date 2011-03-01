/*
 * linux/arch/arm/mach-omap2/usb-ehci.c
 *
 * This file will contain the board specific details for the
 * Synopsys EHCI host controller on OMAP3430
 *
 * Copyright (C) 2007 Texas Instruments
 * Author: Vikram Pandita <vikram.pandita@ti.com>
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
#include <asm/io.h>
#include <mach/mux.h>
#include <linux/usb/musb.h>

#include <mach/hardware.h>
#include <mach/pm.h>
#include <mach/usb.h>

/* EHCI platform specific data */

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
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
		.platform_data          = NULL,
	},
	.num_resources  = ARRAY_SIZE(ehci_resources),
	.resource       = ehci_resources,
};
#endif /* EHCI specific data */


/* OHCI platform specific data */
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct resource ohci_resources[] = {
	[0] = {
		.start   = OMAP34XX_HSUSB_HOST_BASE + 0x400,
		.end     = OMAP34XX_HSUSB_HOST_BASE + 0x400 + SZ_1K - 1,
		.flags   = IORESOURCE_MEM,
	},
	[1] = {         /* general IRQ */
		.start   = 76,
		.flags   = IORESOURCE_IRQ,
	}
};

/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32)0;

static void usb_release(struct device *dev)
{
	/* normally not freed */
}

static struct platform_device ohci_device = {
	.name           = "ohci-omap",
	.id             = 0,
	.dev = {
		.release		= usb_release,
		.dma_mask               = &ohci_dmamask,
		.coherent_dma_mask      = 0xffffffff,
		.platform_data          = NULL,
	},
	.num_resources  = ARRAY_SIZE(ohci_resources),
	.resource       = ohci_resources,
};
#endif /* OHCI specific data */


/* MUX settings for EHCI pins */
/*
 * setup_ehci_io_mux - initialize IO pad mux for USBHOST
 */
#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
static void setup_ehci_io_mux(void)
{

/* Configure pin-muxing on a per-port basis
 * We can configure a port for either PHY mode or for TLL mode
 *
 * PHY mode of operation :
 * 	usable with expansion board 750-2099-001(C)
 * 	where ISP1504 is connected to Port 1 and Port 2
 * 	pin-muxing is set for 12-pin ULPI PHY mode for these two ports
 *
 * TLL Mode of operation :
 * 	pin-muxing is set for TLL mode of operation
 * 	12-pin ULPI SDR TLL mode for Port1/2/3
 *
 * Note that we cannot have a combination of ULPI PHY mode for some port
 * and TLL mode for another port at the same time.
 */

#ifdef CONFIG_OMAP_EHCI_PHY_MODE_PORT1

	omap_cfg_reg(Y9_3430_USB1HS_PHY_STP);
	omap_cfg_reg(Y8_3430_USB1HS_PHY_CLK);
	omap_cfg_reg(AA14_3430_USB1HS_PHY_DIR);
	omap_cfg_reg(AA11_3430_USB1HS_PHY_NXT);
	omap_cfg_reg(W13_3430_USB1HS_PHY_DATA0);
	omap_cfg_reg(W12_3430_USB1HS_PHY_DATA1);
	omap_cfg_reg(W11_3430_USB1HS_PHY_DATA2);
	omap_cfg_reg(Y11_3430_USB1HS_PHY_DATA3);
	omap_cfg_reg(W9_3430_USB1HS_PHY_DATA4);
	omap_cfg_reg(Y12_3430_USB1HS_PHY_DATA5);
	omap_cfg_reg(W8_3430_USB1HS_PHY_DATA6);
	omap_cfg_reg(Y13_3430_USB1HS_PHY_DATA7);

#elif defined(CONFIG_OMAP_EHCI_TLL_MODE_PORT1)

	omap_cfg_reg(Y9_3430_USB1HS_TLL_STP);
	omap_cfg_reg(Y8_3430_USB1HS_TLL_CLK);
	omap_cfg_reg(AA14_3430_USB1HS_TLL_DIR);
	omap_cfg_reg(AA11_3430_USB1HS_TLL_NXT);
	omap_cfg_reg(W13_3430_USB1HS_TLL_DATA0);
	omap_cfg_reg(W12_3430_USB1HS_TLL_DATA1);
	omap_cfg_reg(W11_3430_USB1HS_TLL_DATA2);
	omap_cfg_reg(Y11_3430_USB1HS_TLL_DATA3);
	omap_cfg_reg(W9_3430_USB1HS_TLL_DATA4);
	omap_cfg_reg(Y12_3430_USB1HS_TLL_DATA5);
	omap_cfg_reg(W8_3430_USB1HS_TLL_DATA6);
	omap_cfg_reg(Y13_3430_USB1HS_TLL_DATA7);

#endif /* Port 1 */

#ifdef CONFIG_OMAP_EHCI_PHY_MODE_PORT2

	omap_cfg_reg(AA10_3430_USB2HS_PHY_STP);
	omap_cfg_reg(AA8_3430_USB2HS_PHY_CLK);
	omap_cfg_reg(AA9_3430_USB2HS_PHY_DIR);
	omap_cfg_reg(AB11_3430_USB2HS_PHY_NXT);
	omap_cfg_reg(AB10_3430_USB2HS_PHY_DATA0);
	omap_cfg_reg(AB9_3430_USB2HS_PHY_DATA1);
	omap_cfg_reg(W3_3430_USB2HS_PHY_DATA2);
	omap_cfg_reg(T4_3430_USB2HS_PHY_DATA3);
	omap_cfg_reg(T3_3430_USB2HS_PHY_DATA4);
	omap_cfg_reg(R3_3430_USB2HS_PHY_DATA5);
	omap_cfg_reg(R4_3430_USB2HS_PHY_DATA6);
	omap_cfg_reg(T2_3430_USB2HS_PHY_DATA7);

#elif defined(CONFIG_OMAP_EHCI_TLL_MODE_PORT2)

	omap_cfg_reg(AA10_3430_USB2HS_TLL_STP);
	omap_cfg_reg(AA8_3430_USB2HS_TLL_CLK);
	omap_cfg_reg(AA9_3430_USB2HS_TLL_DIR);
	omap_cfg_reg(AB11_3430_USB2HS_TLL_NXT);
	omap_cfg_reg(AB10_3430_USB2HS_TLL_DATA0);
	omap_cfg_reg(AB9_3430_USB2HS_TLL_DATA1);
	omap_cfg_reg(W3_3430_USB2HS_TLL_DATA2);
	omap_cfg_reg(T4_3430_USB2HS_TLL_DATA3);
	omap_cfg_reg(T3_3430_USB2HS_TLL_DATA4);
	omap_cfg_reg(R3_3430_USB2HS_TLL_DATA5);
	omap_cfg_reg(R4_3430_USB2HS_TLL_DATA6);
	omap_cfg_reg(T2_3430_USB2HS_TLL_DATA7);

#endif	/* Port2 */

#ifdef CONFIG_OMAP_EHCI_TLL_MODE_PORT3

	omap_cfg_reg(AB3_3430_USB3HS_TLL_STP);
	omap_cfg_reg(AA6_3430_USB3HS_TLL_CLK);
	omap_cfg_reg(AA3_3430_USB3HS_TLL_DIR);
	omap_cfg_reg(Y3_3430_USB3HS_TLL_NXT);
	omap_cfg_reg(AA5_3430_USB3HS_TLL_DATA0);
	omap_cfg_reg(Y4_3430_USB3HS_TLL_DATA1);
	omap_cfg_reg(Y5_3430_USB3HS_TLL_DATA2);
	omap_cfg_reg(W5_3430_USB3HS_TLL_DATA3);
	omap_cfg_reg(AB12_3430_USB3HS_TLL_DATA4);
	omap_cfg_reg(AB13_3430_USB3HS_TLL_DATA5);
	omap_cfg_reg(AA13_3430_USB3HS_TLL_DATA6);
	omap_cfg_reg(AA12_3430_USB3HS_TLL_DATA7);

#endif	/* Port3 */

	return;
}
#endif /* CONFIG_USB_EHCI_HCD */


/* MUX settings for OHCI pins */
/*
 * setup_ehci_io_mux - initialize IO pad mux for USBHOST
 */
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static void setup_ohci_io_mux(void)
{

/* Configure pin-muxing on a per-port basis
 * We can configure a port for either ULPI PHY mode (EHCI), or for
 * ULPI TLL mode (EHCI), or for serial PHY mode (OHCI)
 * TODO: Add pin-mux settings for PORT 2 serial PHY mode
 * 	 and for OHCI TLL mode
 */

/* PHY mode for board: 750-2083-001
 * On this board, ISP1301 is connected to Port 1
 * Another ISP1301 transceiver is connected to Port 3
 */

#if defined(CONFIG_OMAP_OHCI_PHY_MODE_3PIN_PORT1) || \
    defined(CONFIG_OMAP_OHCI_PHY_MODE_4PIN_PORT1)

	omap_cfg_reg(AF10_3430_USB1FS_PHY_MM1_RXDP);
	omap_cfg_reg(AG9_3430_USB1FS_PHY_MM1_RXDM);
	omap_cfg_reg(W13_3430_USB1FS_PHY_MM1_RXRCV);
	omap_cfg_reg(W12_3430_USB1FS_PHY_MM1_TXSE0);
	omap_cfg_reg(W11_3430_USB1FS_PHY_MM1_TXDAT);
	omap_cfg_reg(Y11_3430_USB1FS_PHY_MM1_TXEN_N);

#endif /* Port 1 */

/* PHY mode for board: 750-2099-001(C)
 * On this board, there is only one ISP1301 connected to Port3
 */

#if defined(CONFIG_OMAP_OHCI_PHY_MODE_3PIN_PORT3) || \
    defined(CONFIG_OMAP_OHCI_PHY_MODE_4PIN_PORT3)

	omap_cfg_reg(AH3_3430_USB3FS_PHY_MM3_RXDP);
	omap_cfg_reg(AE3_3430_USB3FS_PHY_MM3_RXDM);
	omap_cfg_reg(AD1_3430_USB3FS_PHY_MM3_RXRCV);
	omap_cfg_reg(AE1_3430_USB3FS_PHY_MM3_TXSE0);
	omap_cfg_reg(AD2_3430_USB3FS_PHY_MM3_TXDAT);
	omap_cfg_reg(AC1_3430_USB3FS_PHY_MM3_TXEN_N);

#endif	/* Port3: */

	return;
}
#endif /* CONFIG_USB_OHCI_HCD */

void __init usb_ehci_init(void)
{

#if     defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)

	/* Setup Pin IO MUX for EHCI */
	if (cpu_is_omap34xx())
		setup_ehci_io_mux();

	if (platform_device_register(&ehci_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (EHCI) device\n");
		return;
	}
#endif

}

void __init usb_ohci_init(void)
{

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)

	/* Setup Pin IO MUX for OHCI */
	if (cpu_is_omap34xx())
		setup_ohci_io_mux();

	if (platform_device_register(&ohci_device) < 0) {
		printk(KERN_ERR "Unable to register FS-USB (OHCI) device\n");
		return;
	}
#endif

}
