/*
 * Copyright (C) 2005-2007 by Texas Instruments
 * Some code has been taken from tusb6010.c
 * Copyrights for that are attributable to:
 * Copyright (C) 2006 Nokia Corporation
 * Jarkko Nikula <jarkko.nikula@nokia.com>
 * Tony Lindgren <tony@atomide.com>
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/mux.h>
#include <mach/archos-battery.h>

#include "musb_core.h"
#include "omap2430.h"

#include "smsc.h"

#ifdef CONFIG_ARCH_OMAP3430
#define	get_cpu_rev()	2
#endif

#define MUSB_TIMEOUT_A_WAIT_BCON	1100

static struct timer_list musb_idle_timer;

static void musb_do_idle(unsigned long _musb)
{
	struct musb	*musb = (void *)_musb;
	unsigned long	flags;
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	u8	power;
#endif
	u8	devctl;

	spin_lock_irqsave(&musb->lock, flags);

	/*
	 * If the asynch timer fires when the device is
	 * idle, we do nothing.
	 */
	if (musb->clk_suspend) {
		spin_unlock_irqrestore(&musb->lock, flags);
		return;
	}

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	switch (musb->xceiv.state) {
	case OTG_STATE_A_WAIT_BCON:
#if 0
		devctl &= ~MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE) {
			musb->xceiv.state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
		} else {
			musb->xceiv.state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		}
#if defined(CONFIG_OMAP34XX_OFFMODE)
		/* Keep MUSB suspended on Cable Detach */
		musb_platform_suspend(musb);
#endif
#endif
		break;
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	case OTG_STATE_A_SUSPEND:
		/* finish RESUME signaling? */
		if (musb->port1_status & MUSB_PORT_STAT_RESUME) {
			power = musb_readb(musb->mregs, MUSB_POWER);
			power &= ~MUSB_POWER_RESUME;
			DBG(1, "root port resume stopped, power %02x\n", power);
			musb_writeb(musb->mregs, MUSB_POWER, power);
			musb->is_active = 1;
			musb->port1_status &= ~(USB_PORT_STAT_SUSPEND
						| MUSB_PORT_STAT_RESUME);
			musb->port1_status |= USB_PORT_STAT_C_SUSPEND << 16;
			usb_hcd_poll_rh_status(musb_to_hcd(musb));
			/* NOTE: it might really be A_WAIT_BCON ... */
			musb->xceiv.state = OTG_STATE_A_HOST;
		}
		break;
#endif
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	case OTG_STATE_A_HOST:
		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl &  MUSB_DEVCTL_BDEVICE)
			musb->xceiv.state = OTG_STATE_B_IDLE;
		else
			musb->xceiv.state = OTG_STATE_A_WAIT_BCON;
#endif
	default:
		break;
	}

	spin_unlock_irqrestore(&musb->lock, flags);
}


void musb_platform_try_idle(struct musb *musb, unsigned long timeout)
{
	unsigned long		default_timeout = jiffies + msecs_to_jiffies(3);
	static unsigned long	last_timer;

	if (timeout == 0)
		timeout = default_timeout;

	/* Never idle if active, or when VBUS timeout is not set as host */
	if (musb->is_active || ((musb->a_wait_bcon == 0)
			&& (musb->xceiv.state == OTG_STATE_A_WAIT_BCON))) {
		DBG(4, "%s active, deleting timer\n", otg_state_string(musb));
		del_timer(&musb_idle_timer);
		last_timer = jiffies;
		return;
	}

	if (time_after(last_timer, timeout)) {
		if (!timer_pending(&musb_idle_timer))
			last_timer = timeout;
		else {
			DBG(4, "Longer idle timer already pending, ignoring\n");
			return;
		}
	}
	last_timer = timeout;

	DBG(4, "%s inactive, for idle timer for %lu ms\n",
		otg_state_string(musb),
		(unsigned long)jiffies_to_msecs(timeout - jiffies));
	mod_timer(&musb_idle_timer, timeout);
}

void musb_platform_force_idle(struct musb *musb)
{
	long l;

	/* in any role */
	l = omap_readl(OTG_FORCESTDBY);
	l &= ~ENABLEFORCE;		/* disable MSTANDBY */
	omap_writel(l, OTG_FORCESTDBY);

	l = omap_readl(OTG_SYSCONFIG);
	l &= ~(SMARTSTDBY | NOSTDBY);	/* enable force standby */
	omap_writel(l, OTG_SYSCONFIG);
	l &= ~AUTOIDLE;			/* disable auto idle */
	omap_writel(l, OTG_SYSCONFIG);

	l &= ~(NOIDLE | SMARTIDLE);	/* enable force idle */
	omap_writel(l, OTG_SYSCONFIG);

	l = omap_readl(OTG_FORCESTDBY);
	l |= ENABLEFORCE;		/* enable MSTANDBY */
	omap_writel(l, OTG_FORCESTDBY);

#if 0
	/* REVISIT: SYSCONFIG.AUTOIDLE should be always kept to 0 */
	l = omap_readl(OTG_SYSCONFIG);
	l |= AUTOIDLE;			/* enable auto idle */
	omap_writel(l, OTG_SYSCONFIG);
#endif
}

void musb_platform_enable(struct musb *musb)
{
}
void musb_platform_disable(struct musb *musb)
{
}
static void omap_vbus_power(struct musb *musb, int is_on, int sleeping)
{
}

static void omap_set_vbus(struct musb *musb, int is_on)
{
	u8		devctl;
	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (is_on) {
		musb->is_active = 1;
		musb->xceiv.default_a = 1;
		musb->xceiv.state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MUSB_DEVCTL_SESSION;

		MUSB_HST_MODE(musb);
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		musb->xceiv.default_a = 0;
		musb->xceiv.state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	DBG(1, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}
static int omap_set_power(struct otg_transceiver *x, unsigned mA)
{
	return 0;
}

static int smsc_read(struct musb *musb, int addr, u8 *reg)
{
 	unsigned long timeout;

	*reg = 0;
	
//	printk(KERN_ERR "SMSC read  addr %08X ", addr);

	musb_writeb(musb->mregs, MGC_O_HDRC_REGADDR, addr);
	musb_writeb(musb->mregs, MGC_O_HDRC_REGCONTROL, MGC_M_REGCONTROL_REGREQ | MGC_M_REGCONTROL_REGRDNWR);

	timeout = jiffies + 1*HZ;
	while(time_before(jiffies, timeout)) {
		u8 tmp = musb_readb(musb->mregs, MGC_O_HDRC_REGCONTROL);
		if (tmp & MGC_M_REGCONTROL_REGCMPLT)
			break;
		udelay(2);
//		printk(".");
	}

	if (jiffies >= timeout) {
		printk("%s: timeouted\n", __FUNCTION__);
		return -EIO;
	}
	
	*reg = musb_readb(musb->mregs, MGC_O_HDRC_REGDATA);

//	printk("%02X\n", *reg);

	musb_writeb(musb->mregs, MGC_O_HDRC_REGCONTROL, 0);

	return 0;
}

static int smsc_write(struct musb *musb, int addr, u8 value)
{
 	unsigned long timeout;
	
//	printk(KERN_ERR "SMSC write addr %08X ", addr);

	musb_writeb(musb->mregs, MGC_O_HDRC_REGADDR, addr);
	musb_writeb(musb->mregs, MGC_O_HDRC_REGDATA, value);
	musb_writeb(musb->mregs, MGC_O_HDRC_REGCONTROL, MGC_M_REGCONTROL_REGREQ);

	timeout = jiffies + 1*HZ;
	while(time_before(jiffies, timeout)) {
		u8 tmp = musb_readb(musb->mregs, MGC_O_HDRC_REGCONTROL);
		if (tmp & MGC_M_REGCONTROL_REGCMPLT)
			break;
		udelay(2);
//		printk(".");
	}

	if (jiffies >= timeout) {
		printk("%s: timeouted\n", __FUNCTION__);
		return -EIO;
	}
		
// 	printk("%02X\n", value);

	musb_writeb(musb->mregs, MGC_O_HDRC_REGCONTROL, 0);

	return 0;
}

void musb_platform_set_mode(struct musb *musb, u8 musb_mode)
{
	u8	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	devctl |= MUSB_DEVCTL_SESSION;
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	switch (musb_mode) {
	case MUSB_HOST:
		otg_set_host(&musb->xceiv, musb->xceiv.host);
		break;
	case MUSB_PERIPHERAL:
		otg_set_peripheral(&musb->xceiv, musb->xceiv.gadget);
		break;
	case MUSB_OTG:
		break;
	}
}

static int platform_test_charger( struct musb *musb )
{
	int ret = 0;
	u8 reg = 0;

	printk( KERN_DEBUG "Detecting host type...\n");
	ret = smsc_read(musb, SMSC_FUNCTIONCTRL, &reg);
	if (ret)
		return -EIO;
	reg |= SMSC_FUNCTIONCTRL_OPMODE_L;
	reg &= ~SMSC_FUNCTIONCTRL_OPMODE_H;	
	ret = smsc_write(musb, SMSC_FUNCTIONCTRL, reg);
	if (ret)
		return -EIO;
	ret = smsc_read(musb, SMSC_POWERIO, &reg);
	if (ret)
		return -EIO;
	reg |= SMSC_POWERIO_PULLUPDP;
	ret = smsc_write(musb, SMSC_POWERIO, reg);
	if (ret)
		return -EIO;

	msleep(100);

	ret = smsc_read(musb, SMSC_DEBUG, &reg);
	if (ret)
		return -EIO;

	if( (reg & SMSC_DEBUG_LINESTATE0) && (reg & SMSC_DEBUG_LINESTATE1) ){
		// D+ and D- shorted
		printk( KERN_DEBUG "Single-Ended 1 --> Charger\n");
#ifdef CONFIG_MACH_ARCHOS
		archos_usb_high_charge( 1 );
#endif
	} else {
		// Normal case
		printk( KERN_DEBUG "Single-Ended 0 --> Host or other\n");
	}

	// restore good register values.
	ret = smsc_read(musb, SMSC_POWERIO, &reg);
	if (ret)
		return -EIO;
	reg &= ~SMSC_POWERIO_PULLUPDP;
	reg &= ~SMSC_POWERIO_PULLUPDM;
	ret = smsc_write(musb, SMSC_POWERIO, reg);
	if (ret)
		return -EIO;

	ret = smsc_read(musb, SMSC_FUNCTIONCTRL, &reg);
	if (ret)
		return -EIO;
	reg &= ~SMSC_FUNCTIONCTRL_OPMODE_L;
	reg &= ~SMSC_FUNCTIONCTRL_OPMODE_H;	
	ret = smsc_write(musb, SMSC_FUNCTIONCTRL, reg);
	if (ret)
		return -EIO;

	return 0;
}

int __init musb_platform_init(struct musb *musb)
{
	struct otg_transceiver *x = otg_get_transceiver();
	u8 reg;
	int ret = 0;	
	u32 l;

#if defined(CONFIG_ARCH_OMAP2430)
	omap_cfg_reg(AE5_2430_USB0HS_STP);
#endif

	if (x)
		musb->xceiv = *x;

	/* clock (re)set in platform resume */
	musb_platform_resume(musb);

	l = omap_readl(OTG_SYSCONFIG);
	l &= ~ENABLEWAKEUP;	/* disable wakeup */
	l &= ~NOSTDBY;		/* remove possible nostdby */
	l |= SMARTSTDBY;	/* enable smart standby */
	l &= ~AUTOIDLE;		/* disable auto idle */
	l &= ~NOIDLE;		/* remove possible noidle */
	l |= SMARTIDLE;		/* enable smart idle */
#if 0
	/* REVISIT: "SYSCONFIG.AUTOIDLE should be always kept to 0" */
	l |= AUTOIDLE;		/* enable auto idle */
#endif
	omap_writel(l, OTG_SYSCONFIG);

	l = omap_readl(OTG_INTERFSEL);
	l |= ULPI_12PIN;
	omap_writel(l, OTG_INTERFSEL);

#ifdef CONFIG_MACH_ARCHOS
	/* welwarsky@archos.com
	 * wait some time to allow the controller to initialize
	 * the PHY before we mess with its internals
	 */
	msleep(10);

	// bonnin@archos.com
	// disable the vbus comparators, we do not want an error if there is a glitch on vbus
	ret = smsc_read(musb, SMSC_INTENFALLING, &reg);
	if (ret)
		return ret;
	reg &= ~SMSC_INTENFALLING_VBUSVALID;
	ret = smsc_write(musb, SMSC_INTENFALLING, reg);
	if (ret)
		return ret;

	if( is_host_enabled(musb) ){
		printk("Ground ID inside the phy.\n");
		ret = smsc_read(musb, SMSC_CARKITCTRL, &reg);
		if (ret)
			return ret;
		reg |= SMSC_CARKITCTRL_IDGND;
		ret = smsc_write(musb, SMSC_CARKITCTRL, reg);
		if (ret)
			return ret;
	} else {
		printk("Put ID high impedance.\n");
		ret = smsc_read(musb, SMSC_CARKITCTRL, &reg);
		if (ret)
			return ret;
		reg &= ~SMSC_CARKITCTRL_IDGND;
		ret = smsc_write(musb, SMSC_CARKITCTRL, reg);
		if (ret)
			return ret;

		ret = platform_test_charger(musb);
		if (ret)
			return ret;
	}
#endif

	pr_debug("HS USB OTG: revision 0x%x, sysconfig 0x%02x, "
			"sysstatus 0x%x, intrfsel 0x%x, simenable  0x%x\n",
			omap_readl(OTG_REVISION), omap_readl(OTG_SYSCONFIG),
			omap_readl(OTG_SYSSTATUS), omap_readl(OTG_INTERFSEL),
			omap_readl(OTG_SIMENABLE));

	omap_vbus_power(musb, musb->board_mode == MUSB_HOST, 1);

	if (is_host_enabled(musb))
		musb->board_set_vbus = omap_set_vbus;
	if (is_peripheral_enabled(musb))
		musb->xceiv.set_power = omap_set_power;
	musb->a_wait_bcon = MUSB_TIMEOUT_A_WAIT_BCON;

	setup_timer(&musb_idle_timer, musb_do_idle, (unsigned long) musb);

	return 0;
}

int musb_platform_suspend(struct musb *musb)
{
	if (!musb->clock)
		return 0;

	/* ensure clock not suspended */
	if (musb->clk_suspend) {
		if (musb->set_clock)
			musb->set_clock(musb->clock, 1);
		else
			clk_enable(musb->clock);
		musb->clk_suspend = 0;
	}

	musb_platform_force_idle(musb);

#if defined(CONFIG_OMAP34XX_OFFMODE)
		/* Do nothing */
#else
	if (musb->xceiv.set_suspend)
		musb->xceiv.set_suspend(otg_get_transceiver(), 1);
#endif

	musb->clk_suspend = 1;
	if (musb->set_clock)
		musb->set_clock(musb->clock, 0);
	else
		clk_disable(musb->clock);

	return 0;
}

int musb_platform_resume(struct musb *musb)
{
	u32 l;

	if (!musb->clock)
		return 0;

#if defined(CONFIG_OMAP34XX_OFFMODE)
	/* Do nothing */
#else
	if (musb->xceiv.set_suspend)
		musb->xceiv.set_suspend(otg_get_transceiver(), 0);
#endif

	if (musb->set_clock)
		musb->set_clock(musb->clock, 1);
	else
		clk_enable(musb->clock);
	musb->clk_suspend = 0;

	l = omap_readl(OTG_FORCESTDBY);
	l &= ~ENABLEFORCE;	/* disable MSTANDBY */
	omap_writel(l, OTG_FORCESTDBY);

	l = omap_readl(OTG_SYSCONFIG);
	l |= SMARTSTDBY;	/* enable smart standby */
	l &= ~AUTOIDLE;		/* disable auto idle */
	omap_writel(l, OTG_SYSCONFIG);
	l |= SMARTIDLE;
	omap_writel(l, OTG_SYSCONFIG);
#if 0
	/* REVISIT: SYSCONFIG.AUTOIDLE should be always kept to 0 */
	l |= AUTOIDLE;		/* enable auto idle */
	omap_writel(l, OTG_SYSCONFIG);
#endif

	return 0;
}

int musb_platform_exit(struct musb *musb)
{
	u32 l;
	omap_vbus_power(musb, 0 /*off*/, 1);

	/* ensure clock not suspended */
	if (musb->clk_suspend) {
		if (musb->set_clock)
			musb->set_clock(musb->clock, 1);
		else
			clk_enable(musb->clock);
		musb->clk_suspend = 0;
	}

	musb_platform_force_idle(musb);

	/* make sure to stop all timers */
	del_timer_sync( &musb_idle_timer );
	
	l = omap_readl(OTG_SYSCONFIG);
	l &= ~ENABLEWAKEUP; /* Disable Wakeup */
	omap_writel(l, OTG_SYSCONFIG);

	return 0;
}

#if defined(CONFIG_OMAP34XX_OFFMODE)
struct musb_common_regs{
	/* common registers */
	u8 	faddr;
	u8	power;
	u16 	intrtx;
	u16 	intrrx;
	u16 	intrtxe;
	u16 	intrrxe;
	u8 	intrusbe;
	u8 	intrusb;
	u8	devctl;
};

struct musb_index_regs{
	/* Fifo registers */
	u8	rxfifosz;
	u8	txfifosz;
	u16	txfifoadd;
	u16	rxfifoadd;
} musb_index_regs_t;

struct musb_context{
	/* MUSB init context */
	struct musb_common_regs	common_regs;
	struct musb_index_regs 	index_regs[MUSB_C_NUM_EPS];
};


/* Global: MUSB Context data pointer */
struct musb_context *context_ptr = NULL;
struct musb *musb_ptr = NULL;
bool do_cold_plugging = 0;

/* Context Save/Restore for OFF mode */
int musb_context_store_and_suspend(struct musb *musb, int overwrite)
{
#if 0
	u8 i;
	u32 l;
	DBG(3, "MUSB-Context-SAVE (Off mode support)\n");

	/* Save MUSB Context only once */
	if (!context_ptr || overwrite) {
		if (!context_ptr) {
			context_ptr = kzalloc(sizeof(struct musb_context),
								GFP_KERNEL);
			if (!context_ptr)
				return -ENOMEM;
		}

		/* Save musb ptr */
		musb_ptr = musb;

		/* Save Common registers */
		context_ptr->common_regs.faddr = musb_readb(musb_ptr->mregs,
								MUSB_FADDR);
		context_ptr->common_regs.power = musb_readb(musb_ptr->mregs,
								MUSB_POWER);
		context_ptr->common_regs.intrtx = musb_readw(musb_ptr->mregs,
								MUSB_INTRTX);
		context_ptr->common_regs.intrrx = musb_readw(musb_ptr->mregs,
								MUSB_INTRRX);
		context_ptr->common_regs.intrtxe = musb_readw(musb_ptr->mregs,
								MUSB_INTRTXE);
		context_ptr->common_regs.intrrxe = musb_readw(musb_ptr->mregs,
								MUSB_INTRRXE);
		context_ptr->common_regs.intrusbe = musb_readb(musb_ptr->mregs,
								MUSB_INTRUSBE);
		context_ptr->common_regs.intrusb = musb_readb(musb_ptr->mregs,
								MUSB_INTRUSB);
		context_ptr->common_regs.devctl = musb_readb(musb_ptr->mregs,
								MUSB_DEVCTL);

		DBG(4, "MUSB ContextStore: Common regs:\nfaddr(%x)\n"
			"power(%x)\ninttx(%x)\nintrx(%x)\ninttxe(%x)\n"
			"intrxe(%x)\nintusbe(%x)\nintusb(%x)\ndevctl(%x)\n",
				context_ptr->common_regs.faddr,
				context_ptr->common_regs.power,
				context_ptr->common_regs.intrtx,
				context_ptr->common_regs.intrrx,
				context_ptr->common_regs.intrtxe,
				context_ptr->common_regs.intrrxe,
				context_ptr->common_regs.intrusbe,
				context_ptr->common_regs.intrusb,
				context_ptr->common_regs.devctl);

		/* Save FIFO setup details */
		for (i = 0; i < MUSB_C_NUM_EPS; i++) {
			musb_writeb(musb_ptr->mregs, MUSB_INDEX, i);
			context_ptr->index_regs[i].rxfifosz = musb_readb(musb_ptr->mregs, MUSB_RXFIFOSZ);
			context_ptr->index_regs[i].txfifosz = musb_readb(musb_ptr->mregs, MUSB_TXFIFOSZ);
			context_ptr->index_regs[i].txfifoadd = musb_readw(musb_ptr->mregs, MUSB_TXFIFOADD);
			context_ptr->index_regs[i].rxfifoadd = musb_readw(musb_ptr->mregs, MUSB_RXFIFOADD);

			DBG(4, "EP(%d) rxfifosz(%x) txfifosz(%x) "
				"txfifoaddr(%x) rxfifoadd(%x)\n",
					i,
					context_ptr->index_regs[i].rxfifosz,
					context_ptr->index_regs[i].txfifosz,
					context_ptr->index_regs[i].txfifoadd,
					context_ptr->index_regs[i].rxfifoadd);
		}
		DBG(4, "MUSB Context: FIFO END\n");
	}

	/* Keep system suspended and wakeup through T2 pres INT */
	/* For cold plugging case: do not suspend controller */
	/* If device is connected, do not suspend */

	if (do_cold_plugging) {
		do_cold_plugging = 0; /* For once only */
	} else if (!twl4030_usb_device_connected()) {
		/* Reset the controller and keep in default state on power-up */
		l = omap_readl(OTG_SYSCONFIG);
		l |= SOFTRST;
		omap_writel(l, OTG_SYSCONFIG);

		/* Keep USB suspended till cable is attached */
		musb_platform_suspend(musb_ptr);
	}
#endif
	
	return 0;
}
EXPORT_SYMBOL(musb_context_store_and_suspend);

void musb_context_restore_and_wakeup(void)
{
#if 0
	u8 i;
	u32 l;

	if (!context_ptr) {
		/* T2 called us as a device was found connected */
		/* Its cold plugging, so remember the state when MUSB is up */
		do_cold_plugging = 1;
		return;
	}

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	if ((is_otg_enabled(musb_ptr) || is_peripheral_enabled(musb_ptr)) &&
			(musb_ptr->gadget_driver == NULL)) {
		/* The gadget driver is not yet loaded. Treat this as a case
		 * of cold_plugging
		 */
		do_cold_plugging = 1;
		return;
	}
#endif

	DBG(3, "MUSB Restore Context: Start\n");

	/* Ensure that I-CLK is on after OFF mode */
	musb_platform_resume(musb_ptr);

	/* Set System specific registers
	 * Init controller
	 */
	l = omap_readl(OTG_SYSCONFIG);
	l |= ENABLEWAKEUP;
	omap_writel(l, OTG_SYSCONFIG);

	l = omap_readl(OTG_INTERFSEL);
	l |= ULPI_12PIN;
	omap_writel(l, OTG_INTERFSEL);

	/* Restore MUSB specific registers */

	/* Restore: MUSB Common regs */
	musb_writeb(musb_ptr->mregs, MUSB_FADDR,
					context_ptr->common_regs.faddr);
	musb_writeb(musb_ptr->mregs, MUSB_POWER,
					context_ptr->common_regs.power);
	musb_writew(musb_ptr->mregs, MUSB_INTRTX,
					context_ptr->common_regs.intrtx);
	musb_writew(musb_ptr->mregs, MUSB_INTRRX,
					context_ptr->common_regs.intrrx);
	musb_writew(musb_ptr->mregs, MUSB_INTRTXE,
					context_ptr->common_regs.intrtxe);
	musb_writew(musb_ptr->mregs, MUSB_INTRRXE,
					context_ptr->common_regs.intrrxe);
	musb_writeb(musb_ptr->mregs, MUSB_INTRUSBE,
					context_ptr->common_regs.intrusbe);
	musb_writeb(musb_ptr->mregs, MUSB_INTRUSB,
					context_ptr->common_regs.intrusb);
	musb_writeb(musb_ptr->mregs, MUSB_DEVCTL,
					context_ptr->common_regs.devctl);

	DBG(4, "MUSB ContextStore: Common regs:\nfaddr(%x)\npower(%x)\n"
		"inttx(%x)\nintrx(%x)\ninttxe(%x)\nintrxe(%x)\nintusbe(%x)\n"
		"intusb(%x)\ndevctl(%x)\n",
				context_ptr->common_regs.faddr,
				context_ptr->common_regs.power,
				context_ptr->common_regs.intrtx,
				context_ptr->common_regs.intrrx,
				context_ptr->common_regs.intrtxe,
				context_ptr->common_regs.intrrxe,
				context_ptr->common_regs.intrusbe,
				context_ptr->common_regs.intrusb,
				context_ptr->common_regs.devctl);

	/* Restore: FIFO setup details */
	for (i = 0; i < MUSB_C_NUM_EPS; i++) {
		musb_writeb(musb_ptr->mregs, MUSB_INDEX, i);
		musb_writeb(musb_ptr->mregs, MUSB_RXFIFOSZ,
					context_ptr->index_regs[i].rxfifosz);
		musb_writeb(musb_ptr->mregs, MUSB_TXFIFOSZ,
					context_ptr->index_regs[i].txfifosz);
		musb_writew(musb_ptr->mregs, MUSB_TXFIFOADD,
					context_ptr->index_regs[i].txfifoadd);
		musb_writew(musb_ptr->mregs, MUSB_RXFIFOADD,
					context_ptr->index_regs[i].rxfifoadd);
		DBG(4, "EP(%d) rxfifosz(%x) txfifosz(%x) txfifoaddr(%x) "
			"rxfifoadd(%x)\n",
					i,
					context_ptr->index_regs[i].rxfifosz,
					context_ptr->index_regs[i].txfifosz,
					context_ptr->index_regs[i].txfifoadd,
					context_ptr->index_regs[i].rxfifoadd);
	}

	DBG(3, "MUSB Restore Context: Done\n");
	return;
#endif
	
}
EXPORT_SYMBOL(musb_context_restore_and_wakeup);
#endif /* CONFIG_OMAP34XX_OFFMODE */
