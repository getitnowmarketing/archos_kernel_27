/*
 * arch/arm/mach-omap2/serial.c
 *
 * OMAP2 serial support.
 *
 * Copyright (C) 2005-2008 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * Based off of arch/arm/mach-omap/omap1/serial.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/serial_reg.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/serial.h>
#ifdef CONFIG_SERIAL_8250
#include <linux/serial_8250.h>
#endif

#ifdef CONFIG_SERIAL_OMAP
#include <linux/platform_device.h>
#endif
#include <linux/dma-mapping.h>
#include <linux/init.h>

#ifdef CONFIG_TRACK_RESOURCES
#include <linux/device.h>
#endif

#include <asm/system.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <asm/dma.h>

#include <asm/io.h>
#include <asm/setup.h>
#include <mach/clock.h>
#include <mach/serial.h>
#ifdef CONFIG_SERIAL_8250
#include <mach/omap-hsuart.h>
#endif

#include <mach/gpio.h>
#include <mach/common.h>
#include <mach/board.h>

#ifdef CONFIG_SERIAL_OMAP
#include <mach/omap-serial.h>
#endif

#ifdef CONFIG_OMAP3_PM
#include "prcm-regs.h"
#include "ti-compat.h"
#endif

#define CONSOLE_NAME    "console="
#define FREE            0
#define USED            1
#define MAX_BUF_SIZE    12000

#ifdef CONFIG_HS_SERIAL_SUPPORT
/* structure for storing UART DMA info */
struct omap_hsuart {
	u8 uart_no;
	int rx_dma_channel;
	int tx_dma_channel;

	u8 dma_tx;      /* DMA receive line */
	u8 dma_rx;      /* DMA transmit line */

	dma_addr_t rx_buf_dma_phys;     /* Physical adress of RX DMA buffer */
	dma_addr_t tx_buf_dma_phys;     /* Physical adress of TX DMA buffer */

	void *rx_buf_dma_virt;  /* Virtual adress of RX DMA buffer */
	void *tx_buf_dma_virt;  /* Virtual adress of TX DMA buffer */

	u8 tx_buf_state;
	u8 rx_buf_state;

	struct uart_callback cb;
	u8 mode;

	spinlock_t uart_lock;
	int in_use;
};
static struct omap_hsuart ui[MAX_UARTS + 1];
#endif

static struct clk *uart_ick[OMAP_MAX_NR_PORTS];
static struct clk *uart_fck[OMAP_MAX_NR_PORTS];

#ifdef CONFIG_OMAP3_PM
struct omap_uart_regs {
	u16 dll;
	u16 dlh;
	u16 ier;
	u16 sysc;
	u16 scr;
	u16 wer;
	u16 efr;
};
static struct omap_uart_regs uart_ctx[OMAP_MAX_NR_PORTS];
#endif /* #ifdef CONFIG_OMAP3_PM */

#ifdef CONFIG_SERIAL_8250
static struct plat_serial8250_port serial_platform_data[] = {
	{
		.membase	= IO_ADDRESS(OMAP_UART1_BASE),
		.mapbase	= OMAP_UART1_BASE,
		.irq		= 72,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP24XX_BASE_BAUD * 16,
	}, {
		.membase	= IO_ADDRESS(OMAP_UART2_BASE),
		.mapbase	= OMAP_UART2_BASE,
		.irq		= 73,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP24XX_BASE_BAUD * 16,
	}, {
		.membase	= IO_ADDRESS(OMAP_UART3_BASE),
		.mapbase	= OMAP_UART3_BASE,
		.irq		= 74,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP24XX_BASE_BAUD * 16,
	},
#define QUART_CLK (1843200)
#ifdef CONFIG_MACH_OMAP_ZOOM2
	{
		.membase	= 0,
		.mapbase	= 0x10000000,
		.irq		= OMAP_GPIO_IRQ(102),
		.flags		= UPF_BOOT_AUTOCONF|UPF_IOREMAP|UPF_SHARE_IRQ,
		.iotype		= UPIO_MEM,
		.regshift	= 1,
		.uartclk	= QUART_CLK,
	},
#endif
	{
		.flags		= 0
	}
};
#endif

#ifdef CONFIG_SERIAL_OMAP
static struct resource omap2_uart1_resources[] = {
	{
		.start		= OMAP_UART1_BASE,
		.end		= OMAP_UART1_BASE + 0x3ff,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= 72,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct resource omap2_uart2_resources[] = {
	{
		.start		= OMAP_UART2_BASE,
		.end		= OMAP_UART2_BASE + 0x3ff,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= 73,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct resource omap2_uart3_resources[] = {
	{
		.start		= OMAP_UART3_BASE,
		.end		= OMAP_UART3_BASE + 0x3ff,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= 74,
		.flags		= IORESOURCE_IRQ,
	}
};

#ifdef CONFIG_MACH_OMAP_ZOOM2
static struct resource omap2_quaduart_resources[] = {
	{
		.start		= 0x10000000,
		.end		= 0x10000000 + (0x16 << 1),
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= OMAP_GPIO_IRQ(102),
		.flags		= IORESOURCE_IRQ,
	}
};
#endif

/* OMAP UART platform structure */
static struct platform_device uart1_device = {
	.name			= "omap-uart",
	.id			= 1,
	.num_resources		= ARRAY_SIZE(omap2_uart1_resources),
	.resource		= omap2_uart1_resources,
};
static struct platform_device uart2_device = {
	.name			= "omap-uart",
	.id			= 2,
	.num_resources		= ARRAY_SIZE(omap2_uart2_resources),
	.resource		= omap2_uart2_resources,
};
static struct platform_device uart3_device = {
	.name			= "omap-uart",
	.id			= 3,
	.num_resources		= ARRAY_SIZE(omap2_uart3_resources),
	.resource		= omap2_uart3_resources,
};

#ifdef CONFIG_MACH_OMAP_ZOOM2
static struct platform_device quaduart_device = {
	.name			= "omap-uart",
	.id			= 4,
	.num_resources		= ARRAY_SIZE(omap2_quaduart_resources),
	.resource		= omap2_quaduart_resources,
};
#endif

static struct platform_device *uart_devices[] = {
	&uart1_device,
	&uart2_device,
	&uart3_device,
#ifdef CONFIG_MACH_OMAP_ZOOM2
	&quaduart_device
#endif
};
#endif
#ifdef CONFIG_OMAP3_PM
struct uart_params {
	volatile u32	*wken_regaddr;
	u8		wken_bitpos;
	volatile u32	*sysc_regaddr;
	volatile u32	*wer_regaddr;
};

struct uart_params uart_wkup_params[OMAP_MAX_NR_PORTS];
#endif

static inline unsigned int serial_read_reg(int unum, int offset)
{
#ifdef CONFIG_SERIAL_8250
	struct plat_serial8250_port *p = serial_platform_data + unum;
	offset <<= p->regshift;
	return (unsigned int)__raw_readb(p->membase + offset);
#endif
#ifdef CONFIG_SERIAL_OMAP
	return (unsigned int)__raw_readb(UART_MODULE_BASE(unum) +
					 (offset << 2));
#endif
	return 0;
}

static inline void serial_write_reg(int unum, int offset, int value)
{
#ifdef CONFIG_SERIAL_8250
	struct plat_serial8250_port *p = serial_platform_data + unum;
	offset <<= p->regshift;
	__raw_writeb(value, (unsigned long)(p->membase + offset));
#endif
#ifdef CONFIG_SERIAL_OMAP
	__raw_writeb(value, (unsigned long)(UART_MODULE_BASE(unum)
						+ (offset << 2)));
#endif
}

#ifdef CONFIG_HS_SERIAL_SUPPORT
/*
 * omap_hsuart_isr
 * Identifes the source of interrupt(UART1, UART2, UART3)
 * and reads respective IIR register data.
 * Sends IIR data to the user driver.
 */
static irqreturn_t
omap_hsuart_isr(int irq, void *hs_uart)
{
	struct omap_hsuart *hs = hs_uart;
	u8 lcr_data, iir_data;

	lcr_data = serial_read_reg(hs->uart_no, UART_LCR);
	serial_write_reg(hs->uart_no, UART_LCR, LCR_MODE1);

	iir_data = serial_read_reg(hs->uart_no, UART_IIR);
	/* Restore lcr data */
	serial_write_reg(hs->uart_no, UART_LCR, lcr_data);

	hs->cb.int_callback(iir_data, hs->cb.dev);

	return IRQ_HANDLED;
}

static void uart_rx_dma_callback(int lch, u16 ch_status, void *data)
{
	u8 uart_no = 0;

	if (lch == ui[UART1].rx_dma_channel)
		uart_no = UART1;
	else if (lch == ui[UART2].rx_dma_channel)
		uart_no = UART2;
	else if (lch == ui[UART3].rx_dma_channel)
		uart_no = UART3;

	ui[uart_no].cb.uart_rx_dma_callback(lch, ch_status, data);
	ui[uart_no].rx_buf_state = FREE;
}

static void uart_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	u8 uart_no = 0;

	if (lch == ui[UART1].tx_dma_channel)
		uart_no = UART1;
	else if (lch == ui[UART2].tx_dma_channel)
		uart_no = UART2;
	else if (lch == ui[UART3].tx_dma_channel)
		uart_no = UART3;

	ui[uart_no].cb.uart_tx_dma_callback(lch, ch_status, data);
	ui[uart_no].tx_buf_state = FREE;
}

/*
 * omap_hsuart_get_parms
 * reads requested register data
 */
int omap_hsuart_get_parms(u8 uart_no, u8 *data, u8 reg, u8 lcr_mode)
{
	u8 lcr_data;

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	lcr_data = serial_read_reg(uart_no, UART_LCR);

	serial_write_reg(uart_no, UART_LCR, lcr_mode);
	*data = serial_read_reg(uart_no, reg);
	/* Restore LCR data */
	serial_write_reg(uart_no, UART_LCR, lcr_data);

	return 0;
}
EXPORT_SYMBOL(omap_hsuart_get_parms);

/*
 * omap_hsuart_set_parms
 * writes values into requested UART register
 */
int omap_hsuart_set_parms(u8 uart_no, struct uart_setparm *uart_set)
{
	u8 lcr_data;

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	spin_lock(&(ui[uart_no].uart_lock));

	lcr_data = serial_read_reg(uart_no, UART_LCR);

	serial_write_reg(uart_no, UART_LCR, uart_set->lcr);
	serial_write_reg(uart_no, uart_set->reg, uart_set->reg_data);

	/* Restore LCR data */
	serial_write_reg(uart_no, UART_LCR, lcr_data);

	spin_unlock(&(ui[uart_no].uart_lock));
	return 0;
}
EXPORT_SYMBOL(omap_hsuart_set_parms);

/*
 * omap_hsuart_get_speed
 * reads DLL and DLH register values and
 * calculates UART speed.
 */
int omap_hsuart_get_speed(u8 uart_no, int *speed)
{
	u8 reg = 0;
	u8 dll, dlh;
	u16 divisor = 0;

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	spin_lock(&(ui[uart_no].uart_lock));
	reg = LCR_MODE2;
	serial_write_reg(uart_no, UART_LCR, reg);
	dll = serial_read_reg(uart_no, UART_DLL);
	dlh = serial_read_reg(uart_no, UART_DLM);

	divisor = (dlh << 8) + dll;

	if (!divisor) {
		printk(KERN_WARNING "DLL and DLH read error\n");
		spin_unlock(&(ui[uart_no].uart_lock));
		return -EPERM;
	}

	*speed = (BASE_CLK) / 16 * divisor;
	spin_unlock(&(ui[uart_no].uart_lock));
	return 0;
}
EXPORT_SYMBOL(omap_hsuart_get_speed);

/*
 * omap_hsuart_set_speed
 * used to set the UART speed.
 */
int omap_hsuart_set_speed(u8 uart_no, int speed)
{
	u8 lcr_data, mdr1_data;
	int divisor = 0;

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	spin_lock(&(ui[uart_no].uart_lock));
	/* Disable UART before changing the clock speed - TRM - 18-52 */
	mdr1_data = serial_read_reg(uart_no, UART_OMAP_MDR1);
	serial_write_reg(uart_no, UART_OMAP_MDR1, ~MODE_SELECT_MASK);

	/* Enable access to DLL and DLH registers */
	lcr_data = serial_read_reg(uart_no, UART_LCR);
	serial_write_reg(uart_no, UART_LCR, LCR_MODE2);

	mdr1_data &= MODE_SELECT_MASK;

	/* Only UART3 supports IrDA mode */
	if ((uart_no == UART3) && (ui[uart_no].mode == IRDA_MODE)) {
		if (speed <= IR_SIR_SPEED) {
			divisor = BASE_CLK / (16 * speed);
			mdr1_data |= UART_SIR_MODE;
		} else if (speed <= IR_MIR_SPEED) {
			divisor = BASE_CLK / (41 * speed);
			mdr1_data |= UART_MIR_MODE;
		} else if (speed <= IR_FIR_SPEED)
			mdr1_data |= UART_FIR_MODE;
		else
			goto exit_path1;
	} else if (ui[uart_no].mode == UART_MODE) {
		if (speed <= UART_16X_SPEED) {
			divisor = BASE_CLK / (16 * speed);
			mdr1_data |= UART_16X_MODE;
		} else if (speed <= UART_13X_SPEED) {
			divisor = BASE_CLK / (13 * speed);
			mdr1_data |= UART_13X_MODE;
		} else
			goto exit_path1;
	} else if (ui[uart_no].mode == UART_AUTOBAUD_MODE)
		mdr1_data |= UART_16XAUTO_MODE;
	else if (ui[uart_no].mode == CIR_MODE)
		mdr1_data |= UART_CIR_MODE;
	else
		goto exit_path1;

	serial_write_reg(uart_no, UART_DLL, divisor & 0xFF);
	serial_write_reg(uart_no, UART_DLM, divisor >> 8);
	serial_write_reg(uart_no, UART_OMAP_MDR1, mdr1_data);

	printk(KERN_DEBUG "Changing UART speed to %d....\n", speed);

	/* restore LCR values */
	serial_write_reg(uart_no, UART_LCR, lcr_data);
	spin_unlock(&(ui[uart_no].uart_lock));
	return 0;

exit_path1:
	printk(KERN_ERR "Requested speed is not supported\n");
	/* Restore LCR and MDR1 regisgters to original value */
	serial_write_reg(uart_no, UART_OMAP_MDR1, mdr1_data);
	serial_write_reg(uart_no, UART_LCR, lcr_data);
	spin_unlock(&(ui[uart_no].uart_lock));
	return -EPERM;
}
EXPORT_SYMBOL(omap_hsuart_set_speed);

/*
 * omap_hsuart_config
 * configures the requested UART
 */
int omap_hsuart_config(u8 uart_no, struct uart_config *uartcfg)
{
	if (in_interrupt())
		BUG();

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	spin_lock(&(ui[uart_no].uart_lock));

	ui[uart_no].mode = uartcfg->mode;

	/* Put UART3 in reset mode */
	serial_write_reg(uart_no, UART_OMAP_MDR1, ~MODE_SELECT_MASK);

	/* Clear DLL and DLH */
	serial_write_reg(uart_no, UART_LCR, LCR_MODE2);
	serial_write_reg(uart_no, UART_DLL, 0);
	serial_write_reg(uart_no, UART_DLM, 0);

	serial_write_reg(uart_no, UART_OMAP_SCR, 0);

	serial_write_reg(uart_no, UART_LCR, LCR_MODE3);
	serial_write_reg(uart_no, UART_EFR, (uartcfg->efr));

	serial_write_reg(uart_no, UART_LCR, LCR_MODE2);
	/* enable TCR and TLR Registers */
	serial_write_reg(uart_no, UART_MCR, MCR_TCR_TLR);

	serial_write_reg(uart_no, UART_TI752_TLR, (uartcfg->tlr));

	serial_write_reg(uart_no, UART_LCR, (uartcfg->lcr));

	serial_write_reg(uart_no, UART_FCR, (uartcfg->fcr));

	/* disable access to TCR and TLR registers */
	serial_write_reg(uart_no, UART_MCR, 0);

	serial_write_reg(uart_no, UART_OMAP_SCR, (uartcfg->scr));

	serial_write_reg(uart_no, UART_OMAP_MDR1, (uartcfg->mdr1));

	serial_write_reg(uart_no, UART_OMAP_MDR2, (uartcfg->mdr2));

	serial_write_reg(uart_no, UART_OMAP_ACREG, (uartcfg->acreg));

	serial_write_reg(uart_no, UART_IER, (uartcfg->ier));

	if (ui[uart_no].mode == IRDA_MODE) {
		serial_write_reg(uart_no, UART_OMAP_RXFLL, (uartcfg->rxfll));
		serial_write_reg(uart_no, UART_OMAP_RXFLH, (uartcfg->rxflh));
	}
	serial_read_reg(uart_no, UART_OMAP_RESUME);
	spin_unlock(&(ui[uart_no].uart_lock));
	return 0;
}
EXPORT_SYMBOL(omap_hsuart_config);

/* omap_hsuart_stop stops/resets requested UART. */
int omap_hsuart_stop(u8 uart_no)
{
	if (in_interrupt())
		BUG();

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	spin_lock(&(ui[uart_no].uart_lock));
	/* Put UART3 in reset mode */
	serial_write_reg(uart_no, UART_OMAP_MDR1, ~MODE_SELECT_MASK);

	/* Clear DLL and DLH */
	serial_write_reg(uart_no, UART_LCR, LCR_MODE2);
	serial_write_reg(uart_no, UART_DLL, 0);
	serial_write_reg(uart_no, UART_DLM, 0);

	/* Disable requested UART interrupts */
	serial_write_reg(uart_no, UART_IER, 0);

	/* Stop DMA channels */
	omap_stop_dma(ui[uart_no].rx_dma_channel);
	omap_stop_dma(ui[uart_no].tx_dma_channel);

	/* Move buffer states to free */
	ui[uart_no].tx_buf_state = FREE;
	ui[uart_no].rx_buf_state = FREE;

	spin_unlock(&(ui[uart_no].uart_lock));
	return 0;
}
EXPORT_SYMBOL(omap_hsuart_stop);

/* omap_hsuart_rx Copies data from DMA buffer to user driver buffer. */
int omap_hsuart_rx(u8 uart_no, void *data, int *len)
{
	unsigned long flags;
	int ret = 0;

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	spin_lock_irqsave(&(ui[uart_no].uart_lock), flags);
	*len = omap_get_dma_dst_pos(ui[uart_no].rx_dma_channel);
	*len -= ui[uart_no].rx_buf_dma_phys;

	memcpy(data, ui[uart_no].rx_buf_dma_virt, *len);
	/* DMA data is copied to user driver buffer,
	 * now it is safe to move rx_buf_state to free.
	 */
	ui[uart_no].rx_buf_state = FREE;

	spin_unlock_irqrestore(&(ui[uart_no].uart_lock), flags);
	return ret;
}
EXPORT_SYMBOL(omap_hsuart_rx);

/* omap_hsuart_tx copies data from client driver buffer to DMA buffer. */
int omap_hsuart_tx(u8 uart_no, void *data, int size)
{
	unsigned long flags;
	int ret = 0;

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	if (unlikely(size < 0 || size > ui[uart_no].cb.tx_buf_size)) {
		printk(KERN_DEBUG "omap_hsuart: Error.Invalid buffer size\n");
		return -EPERM;
	}
	spin_lock_irqsave(&(ui[uart_no].uart_lock), flags);
	printk(KERN_DEBUG "omap_hsuart:"
				"omap_hsuart_tx:%s\n %d\n", (char *)data, size);
	memcpy(ui[uart_no].tx_buf_dma_virt, data, size);

	spin_unlock_irqrestore(&(ui[uart_no].uart_lock), flags);
	return ret;
}
EXPORT_SYMBOL(omap_hsuart_tx);

/*  omap_hsuart_start_tx - starts TX of data using DMA or PIO */
int omap_hsuart_start_tx(u8 uart_no, int size)
{
	struct plat_serial8250_port *p = serial_platform_data + uart_no;

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	if (ui[uart_no].tx_buf_state == USED) {
		printk(KERN_DEBUG "omap_hsuart: UART DMA buffer is not free\n");
		return -EBUSY;
	}

	ui[uart_no].tx_buf_state = USED;

	omap_set_dma_dest_params(ui[uart_no].tx_dma_channel, 0,
				 OMAP_DMA_AMODE_CONSTANT, p->mapbase, 0,
				 0);
	omap_set_dma_src_params(ui[uart_no].tx_dma_channel, 0,
				OMAP_DMA_AMODE_POST_INC,
				ui[uart_no].tx_buf_dma_phys, 0, 0);
	omap_set_dma_transfer_params(ui[uart_no].tx_dma_channel,
				     OMAP_DMA_DATA_TYPE_S8, size, 1,
				     OMAP_DMA_SYNC_ELEMENT,
				     ui[uart_no].dma_tx, 0);
	omap_start_dma(ui[uart_no].tx_dma_channel);

	return 0;
}
EXPORT_SYMBOL(omap_hsuart_start_tx);

/* Starts receiving data from DMA rx channel */
int omap_hsuart_start_rx(u8 uart_no, int size)
{
	struct plat_serial8250_port *p = serial_platform_data + uart_no;

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	if (ui[uart_no].rx_buf_state == USED) {
		printk(KERN_DEBUG "omap_hsuart: UART DMA buffer is not free\n");
		return -EBUSY;
	}

	ui[uart_no].rx_buf_state = USED;

	omap_set_dma_src_params(ui[uart_no].rx_dma_channel, 0,
				OMAP_DMA_AMODE_CONSTANT, p->mapbase, 0,
				0);
	omap_set_dma_dest_params(ui[uart_no].rx_dma_channel, 0,
					OMAP_DMA_AMODE_POST_INC,
					ui[uart_no].rx_buf_dma_phys, 0,
					0);

	omap_set_dma_transfer_params(ui[uart_no].rx_dma_channel,
					OMAP_DMA_DATA_TYPE_S8, size, 1,
					OMAP_DMA_SYNC_ELEMENT,
					ui[uart_no].dma_rx, 0);
	omap_start_dma(ui[uart_no].rx_dma_channel);

	return 0;
}
EXPORT_SYMBOL(omap_hsuart_start_rx);

/* omap_hsuart_stop_rx : stops rx dma */
int omap_hsuart_stop_rx(u8 uart_no)
{
	if (uart_no > MAX_UARTS)
		return -ENODEV;

	omap_stop_dma(ui[uart_no].rx_dma_channel);

	return 0;
}
EXPORT_SYMBOL(omap_hsuart_stop_rx);

/* omap_hsuart_stop_tx : stops tx dma */
int omap_hsuart_stop_tx(u8 uart_no)
{
	if (uart_no > MAX_UARTS)
		return -ENODEV;

	omap_stop_dma(ui[uart_no].tx_dma_channel);

	return 0;
}
EXPORT_SYMBOL(omap_hsuart_stop_tx);

/*  omap_hsuart_interrupt used to enable/disable UART interrupts */
int omap_hsuart_interrupt(u8 uart_no, int enable)
{
	struct plat_serial8250_port *p = serial_platform_data + uart_no;
	if (uart_no > MAX_UARTS)
		return -ENODEV;

	if (enable)
		enable_irq(p->irq);
	else
		disable_irq(p->irq);

	return 0;
}
EXPORT_SYMBOL(omap_hsuart_interrupt);

/*  omap_hsuart_request Allocates requested UART.*/
int omap_hsuart_request(u8 uart_no, struct uart_callback *uart_cback)
{
	int err;
	struct plat_serial8250_port *p = serial_platform_data + uart_no;
	struct uart_callback *cb = uart_cback;
	struct omap_hsuart *hs = &ui[uart_no];

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	spin_lock(&(hs->uart_lock));
	if (ui[uart_no].in_use) {
		printk(KERN_DEBUG
		       "Err Requested UART is not available\n");
		spin_unlock(&(ui[uart_no].uart_lock));
		return -EACCES;
	}
	spin_unlock(&(hs->uart_lock));

	serial8250_unregister_port(uart_no);

	switch (uart_no) {
	case UART1:
		hs->dma_tx = OMAP24XX_DMA_UART1_TX;
		hs->dma_rx = OMAP24XX_DMA_UART1_RX;
		break;
	case UART2:
		hs->dma_tx = OMAP24XX_DMA_UART2_TX;
		hs->dma_rx = OMAP24XX_DMA_UART2_RX;
		break;
	case UART3:
		hs->dma_tx = OMAP24XX_DMA_UART3_TX;
		hs->dma_rx = OMAP24XX_DMA_UART3_RX;
		break;
	default:
		return -EPERM;
	}


	if ((hs->cb.txrx_req_flag == TXRX)
	    || (hs->cb.txrx_req_flag == RX_ONLY)) {
		if (unlikely(cb->rx_buf_size < 1 ||
			cb->rx_buf_size > MAX_BUF_SIZE)) {
			printk(KERN_DEBUG
				 "omap_hsuart: Err Invalid RX buffer size\n");
			return -EACCES;
		}
	}

	if ((hs->cb.txrx_req_flag == TXRX)
	    || (hs->cb.txrx_req_flag == TX_ONLY)) {
		if (unlikely(cb->tx_buf_size < 1 ||
			cb->tx_buf_size > MAX_BUF_SIZE)) {
			printk(KERN_DEBUG
				 "omap_hsuart: Err Invalid TX buffer size\n");
			return -EACCES;
		}
	}

	hs->uart_no = uart_no;
	hs->cb.dev = cb->dev;
	hs->cb.int_callback = cb->int_callback;
	hs->cb.dev_name = cb->dev_name;

	if (request_irq(p->irq, (void *)omap_hsuart_isr, 0,
		hs->cb.dev_name, hs)) {
		printk(KERN_ERR "omap_hsuart: Error: IRQ allocation failed\n");
		err = -EPERM;
		goto exit_path0;
	}

	spin_lock(&(hs->uart_lock));
	hs->cb.txrx_req_flag = cb->txrx_req_flag;

	if ((hs->cb.txrx_req_flag == TXRX)
	    || (hs->cb.txrx_req_flag == RX_ONLY)) {
		/* Request DMA Channels for requested UART */
		err = omap_request_dma(hs->dma_rx,
			 "UART Rx DMA", (void *)uart_rx_dma_callback,
			 hs->cb.dev, &(hs->rx_dma_channel));
		if (err) {
			printk(KERN_ERR
				 "omap_hsuart: Failed to get DMA Channels\n");
			goto exit_path1;
		}

		hs->cb.rx_buf_size = cb->rx_buf_size;
		hs->cb.uart_rx_dma_callback =
		    cb->uart_rx_dma_callback;
		hs->rx_buf_dma_virt =
		    dma_alloc_coherent(NULL, hs->cb.rx_buf_size,
				       (dma_addr_t *) &
					(hs->rx_buf_dma_phys), 0);
		if (!hs->rx_buf_dma_virt) {
			printk(KERN_ERR
			       "Failed to allocate DMA Rx Buffer...!!!!\n");
			goto exit_path2;
		}
	}

	if ((hs->cb.txrx_req_flag == TXRX)
	    || (hs->cb.txrx_req_flag == TX_ONLY)) {
		err = omap_request_dma(hs->dma_tx,
			 "UART Tx DMA", (void *)uart_tx_dma_callback,
			 hs->cb.dev, &(hs->tx_dma_channel));
		if (err) {
			printk(KERN_ERR
				 "omap_hsuart: Failed to get DMA Channels\n");
			goto exit_path3;
		}
		hs->cb.tx_buf_size = cb->tx_buf_size;
		hs->cb.uart_tx_dma_callback =
		    cb->uart_tx_dma_callback;
		hs->tx_buf_dma_virt =
		    dma_alloc_coherent(NULL, hs->cb.tx_buf_size,
					(dma_addr_t *) &
					(hs->tx_buf_dma_phys), 0);
		if (!hs->tx_buf_dma_virt) {
			printk(KERN_ERR
			       "omap_hsuart: Failed to allocate DMA Tx Buf\n");
			goto exit_path4;
		}
	}

	hs->tx_buf_state = FREE;
	hs->rx_buf_state = FREE;

	hs->in_use = 1;
	spin_unlock(&(hs->uart_lock));

	return 0;

exit_path4:
	omap_free_dma(hs->rx_dma_channel);
	if ((hs->cb.txrx_req_flag == TXRX)
	    || (hs->cb.txrx_req_flag == TX_ONLY)) {
		hs->cb.tx_buf_size = 0;
		hs->cb.uart_tx_dma_callback = NULL;
	}

exit_path3:
	if ((hs->cb.txrx_req_flag == TXRX)
	    || (hs->cb.txrx_req_flag == RX_ONLY)) {
		dma_free_coherent(hs->cb.dev,
				  hs->cb.rx_buf_size,
				  hs->rx_buf_dma_virt,
				  hs->rx_buf_dma_phys);
		hs->cb.rx_buf_size = 0;
		hs->cb.uart_rx_dma_callback = NULL;

	}
exit_path2:
	if (hs->cb.txrx_req_flag == TXRX
	    || (hs->cb.txrx_req_flag == RX_ONLY))
		omap_free_dma(hs->rx_dma_channel);
exit_path1:
	free_irq(p->irq, hs->cb.dev);
exit_path0:
	hs->cb.dev = NULL;
	hs->cb.int_callback = NULL;
	spin_unlock(&(hs->uart_lock));

	return err;
}
EXPORT_SYMBOL(omap_hsuart_request);

/* Release the requested uart */
int omap_hsuart_release(u8 uart_no)
{
	struct omap_hsuart *hs = &ui[uart_no];
	struct plat_serial8250_port *p = serial_platform_data + uart_no;

	if (uart_no > MAX_UARTS)
		return -ENODEV;

	spin_lock(&(ui[uart_no].uart_lock));
	if (!hs->in_use) {
		printk(KERN_DEBUG
			 "omap_hsuart: Requested UART already in free state\n");
		spin_unlock(&(hs->uart_lock));
		return -EACCES;
	}
	/* MOVE requested UART to free state.
	 * Free DMA channels.
	 * Free DMA buffers.
	 */
	free_irq(p->irq, hs);

	omap_free_dma(hs->rx_dma_channel);
	omap_free_dma(hs->tx_dma_channel);

	dma_free_coherent(hs->cb.dev,
			hs->cb.rx_buf_size,
			hs->rx_buf_dma_virt,
			hs->rx_buf_dma_phys);
	dma_free_coherent(ui[uart_no].cb.dev,
			hs->cb.tx_buf_size,
			hs->tx_buf_dma_virt,
			hs->tx_buf_dma_phys);
	hs->cb.uart_tx_dma_callback = NULL;
	hs->cb.uart_rx_dma_callback = NULL;

	hs->cb.int_callback = NULL;
	hs->cb.dev_name = NULL;
	hs->cb.dev = NULL;
	hs->in_use = 0;

	hs->rx_dma_channel = 0;
	hs->tx_dma_channel = 0;
	hs->rx_buf_dma_phys = 0;
	hs->tx_buf_dma_phys = 0;
	hs->rx_buf_dma_virt = NULL;
	hs->tx_buf_dma_virt = NULL;
	hs->rx_buf_state = 0;
	hs->tx_buf_state = 0;

	spin_unlock(&(hs->uart_lock));
	return 0;
}
EXPORT_SYMBOL(omap_hsuart_release);

/* console_detect Detect: Console UART using command line parameter. */
int console_detect(char *str)
{
	char *next, *start = NULL;
	int i;

	i = strlen(CONSOLE_NAME);
	next = saved_command_line;
	while ((next = strchr(next, 'c')) != NULL) {
		if (!strncmp(next, CONSOLE_NAME, i)) {
			start = next;
			break;
		} else
			next++;

	}
	if (!start)
		return -EPERM;
	i = 0;
	start = strchr(start, '=') + 1;
	while (*start != ',') {
		str[i++] = *start++;
		if (i > 6) {
			printk(KERN_DEBUG
				 "omap_hsuart: Invalid Console Name\n");
			return -EPERM;
		}
	}
	str[i] = '\0';

	return 0;
}
#endif

/*
 * Internal UARTs need to be initialized for the 8250 autoconfig to work
 * properly. Note that the TX watermark initialization may not be needed
 * once the 8250.c watermark handling code is merged.
 */
static inline void __init omap_serial_reset(int unum)
{
	serial_write_reg(unum, UART_OMAP_MDR1, 0x07);
	serial_write_reg(unum, UART_OMAP_SCR, 0x08);
	serial_write_reg(unum, UART_OMAP_MDR1, 0x00);
	serial_write_reg(unum, UART_OMAP_SYSC, (0x02 << 3)
			 | (1 << 2) | (1 << 0));
}

#ifdef CONFIG_TRACK_RESOURCES
/* device name needed for resource tracking layer */
struct device_driver serial_drv = {
	.name =  "uart",
};

struct device serial_dev = {
	.driver = &serial_drv,
};
#endif

void omap_serial_enable_clocks(int enable)
{
	int i;
	for (i = 0; i < OMAP_MAX_NR_PORTS; i++) {
		if (uart_ick[i] && uart_fck[i]) {
			if (enable) {
				clk_enable(uart_ick[i]);
				clk_enable(uart_fck[i]);
			} else {
				clk_disable(uart_ick[i]);
				clk_disable(uart_fck[i]);
			}
		}
	}
}
#ifdef CONFIG_OMAP3_PM
void omap_serial_wken(void)
{
	int i;
	u32 uart_sysc, pm_wken;

	uart_wkup_params[0].wken_regaddr = &PM_WKEN1_CORE;
	uart_wkup_params[0].wken_bitpos = 13;
	uart_wkup_params[0].sysc_regaddr = &PRCM_UART1_SYSCONFIG;
	uart_wkup_params[0].wer_regaddr = &PRCM_UART1_WER;

	uart_wkup_params[1].wken_regaddr = &PM_WKEN1_CORE;
	uart_wkup_params[1].wken_bitpos = 14;
	uart_wkup_params[1].sysc_regaddr = &PRCM_UART2_SYSCONFIG;
	uart_wkup_params[1].wer_regaddr = &PRCM_UART2_WER;

	uart_wkup_params[2].wken_regaddr = &PM_WKEN_PER;
	uart_wkup_params[2].wken_bitpos = 11;
	uart_wkup_params[2].sysc_regaddr = &PRCM_UART3_SYSCONFIG;
	uart_wkup_params[2].wer_regaddr = &PRCM_UART3_WER;

	for (i = 0; i < OMAP_MAX_NR_PORTS; i++) {
		/* Enable module wakeup capability */
		pm_wken = *uart_wkup_params[i].wken_regaddr;
		pm_wken |= 1<<(uart_wkup_params[i].wken_bitpos);
		*uart_wkup_params[i].wken_regaddr = pm_wken;

		/* Enable smart idle in UART SYSC */
		uart_sysc = *uart_wkup_params[i].sysc_regaddr;
		uart_sysc |= ((2<<3)|(1<<2)|(1));
		*uart_wkup_params[i].sysc_regaddr = uart_sysc;

		/* Enabling all wakeup bits in WER_REG */
		*uart_wkup_params[i].wer_regaddr = 0x3F;
	}
}

void omap_uart_save_ctx(int unum)
{
	u16 lcr = 0;
	lcr = serial_read_reg(unum, UART_LCR);
	serial_write_reg(unum, UART_LCR, 0xBF);
	uart_ctx[unum].dll = serial_read_reg(unum, UART_DLL);
	uart_ctx[unum].dlh = serial_read_reg(unum, UART_DLM);
	uart_ctx[unum].efr = serial_read_reg(unum, UART_EFR);
	serial_write_reg(unum, UART_LCR, lcr);
	uart_ctx[unum].ier = serial_read_reg(unum, UART_IER);
	uart_ctx[unum].sysc = serial_read_reg(unum, UART_OMAP_SYSC);
	uart_ctx[unum].scr = serial_read_reg(unum, UART_OMAP_SCR);
	uart_ctx[unum].wer = serial_read_reg(unum, UART_OMAP_WER);
}
EXPORT_SYMBOL(omap_uart_save_ctx);

void omap_uart_restore_ctx(int unum)
{
	serial_write_reg(unum, UART_OMAP_MDR1, 0x7);
	serial_write_reg(unum, UART_LCR, 0xBF); /* Config B mode */
	serial_write_reg(unum, UART_EFR, UART_EFR_ECB);
	serial_write_reg(unum, UART_LCR, 0x0); /* Operational mode */
	serial_write_reg(unum, UART_IER, 0x0);
#ifdef CONFIG_SERIAL_OMAP
	serial_write_reg(unum, UART_FCR, fcr[unum]);
#else
	serial_write_reg(unum, UART_FCR, 0x61);
#endif
	serial_write_reg(unum, UART_LCR, 0xBF); /* Config B mode */
	serial_write_reg(unum, UART_DLL, uart_ctx[unum].dll);
	serial_write_reg(unum, UART_DLM, uart_ctx[unum].dlh);
	serial_write_reg(unum, UART_LCR, 0x0); /* Operational mode */
	serial_write_reg(unum, UART_IER, uart_ctx[unum].ier);
	serial_write_reg(unum, UART_LCR, 0xBF); /* Config B mode */
	serial_write_reg(unum, UART_EFR, uart_ctx[unum].efr);
	serial_write_reg(unum, UART_LCR, UART_LCR_WLEN8);
	serial_write_reg(unum, UART_OMAP_SCR, uart_ctx[unum].scr);
	serial_write_reg(unum, UART_OMAP_WER, uart_ctx[unum].wer);
	serial_write_reg(unum, UART_OMAP_SYSC,
			 uart_ctx[unum].sysc | (0x2 << 3));
	serial_write_reg(unum, UART_OMAP_MDR1, 0x00); /* UART 16x mode */
}
EXPORT_SYMBOL(omap_uart_restore_ctx);
#endif

void __init omap_serial_init(void)
{
	int i, arr_size;
#ifdef CONFIG_SERIAL_OMAP
	int r;
#endif
#ifdef CONFIG_HS_SERIAL_SUPPORT
	char str[7];
#endif
	const struct omap_uart_config *info;
	char name[16];
#ifdef CONFIG_TRACK_RESOURCES
	struct device *dev = &serial_dev;
#else
	struct device *dev = NULL;
#endif

	/*
	 * Make sure the serial ports are muxed on at this point.
	 * You have to mux them off in device drivers later on
	 * if not needed.
	 */

	info = omap_get_config(OMAP_TAG_UART, struct omap_uart_config);

	if (info == NULL)
		return;

#ifdef CONFIG_SERIAL_8250
	arr_size = OMAP_MAX_NR_PORTS;
#elif CONFIG_SERIAL_OMAP
	arr_size = ARRAY_SIZE(uart_devices);
#else
	arr_size = 0;
#endif
	for (i = 0; i < arr_size; i++) {
#ifdef CONFIG_SERIAL_8250
		struct plat_serial8250_port *p = serial_platform_data + i;
#endif

#ifndef CONFIG_MACH_OMAP_ZOOM2
		if (!(info->enabled_uarts & (1 << i))) {
#ifdef CONFIG_SERIAL_8250
			p->membase = NULL;
			p->mapbase = 0;
#endif
			continue;
		}
#endif

		/* Quart port is not omap uart based */
		if (i < OMAP_MAX_NR_PORTS) {
			sprintf(name, "uart%d_ick", i+1);
			printk(KERN_INFO "\n Enable clock [%s]", name);
			uart_ick[i] = clk_get(NULL, name);
			if (IS_ERR(uart_ick[i])) {
				printk(KERN_ERR "Could not\
						get uart%d_ick\n", i+1);
				uart_ick[i] = NULL;
			} else
				clk_enable(uart_ick[i]);

			sprintf(name, "uart%d_fck", i+1);
			printk(KERN_INFO "\n Enable clock [%s]", name);
			uart_fck[i] = clk_get(NULL, name);
			if (IS_ERR(uart_fck[i])) {
				printk(KERN_ERR "Could not get\
						uart%d_fck\n", i+1);
				uart_fck[i] = NULL;
			} else
				clk_enable(uart_fck[i]);
		}

#ifdef CONFIG_SERIAL_OMAP
		r = platform_device_register(uart_devices[i]);
		if (r < 0)
			printk(KERN_ERR "Failed to register UART%d\n", i + 1);
#endif

		if (!(info->enabled_uarts & (1 << i))) {
#ifdef CONFIG_SERIAL_8250
			p->membase = NULL;
			p->mapbase = 0;
#endif
			continue;
		}

		omap_serial_reset(i);
	}
#ifdef CONFIG_HS_SERIAL_SUPPORT
	/* initialize tx/rx channel */
	for (i = UART1; i <= UART3; i++) {
		ui[i].rx_dma_channel = -1;
		ui[i].tx_dma_channel = -1;
	}

	/* Reserve the console uart */
	if (console_detect(str))
		printk(KERN_DEBUG "omap_hsuart: Invalid console paramter"
			"UART Library Init Failed\n");

#ifdef CONFIG_MACH_OMAP_ZOOM2
	/* debug console is on quart chip outside omap for zoom2 */
	if (!strcmp(str, "ttyS3") ) {
		/* Correct decleration of UART for zoom2 board */
		return;
	} else {
		printk("\nWarning: Zoom2 has Quart: on ttyS3"
			"\n\tYou specified console as [%s]"
			"\n\tCorrect as follows:"
			"\n\tbootargs console=ttyS3"
			"\n\tFile system: etc/inittab"
			"\n\t\tttyS3::askfirst:-/bin/sh"
			"\n\n", str);
	}
#endif

	/* There is no Quart: If we are here
	 * Index 0,1,2 are for UART1, 2 and 3
	 * Index 3 is for Quart PortA.
	 *
	 * Disable Quart port as ttyS3 is not specified
	 * */
	serial_platform_data[3].flags = 0;

	if (!strcmp(str, "ttyS0"))
		ui[UART1].in_use = 1;
	else if (!strcmp(str, "ttyS1"))
		ui[UART2].in_use = 1;
	else if (!strcmp(str, "ttyS2"))
		ui[UART3].in_use = 1;
	else if (!strcmp(str, "ttyS3"))
		ui[UART4].in_use = 1;
	else
		printk(KERN_DEBUG
			"Unable to recongnize Console UART\n");
#endif
#ifdef CONFIG_OMAP3_PM
	omap_serial_wken();
#endif
}

#ifdef CONFIG_SERIAL_8250
static struct platform_device serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= serial_platform_data,
	},
};

static int __init omap_init(void)
{
	return platform_device_register(&serial_device);
}
arch_initcall(omap_init);
#endif
