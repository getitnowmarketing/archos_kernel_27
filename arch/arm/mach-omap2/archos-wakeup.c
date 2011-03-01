/*
 * archos-wakeup.c
 *
 *  Created on: Jan 7, 2009
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *      
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <mach/atmega.h>
#include <mach/resource.h>
#include <mach/io.h>

#ifdef CONFIG_OMAP3_PM
#define CONTROL_SYSC_SMARTIDLE  	(0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE   	(0x1)
#define PRCM_INTERRUPT_MASK     	(1 << 11)
#define UART1_INTERRUPT_MASK    	(1 << 8)
#define UART2_INTERRUPT_MASK    	(1 << 9)
#define UART3_INTERRUPT_MASK    	(1 << 10)

#include "prcm-regs.h"
#include "ti-compat.h"
#include <mach/prcm_34xx.h>

static struct constraint_handle *vdd1_opp_co;
static struct constraint_id reset_constr = {
	.type = RES_FREQ_CO,
	.data = (void*)"vdd1_opp",
};

int console_detect(char *str);
unsigned int uart_interrupt_mask_value;
u32 *console_padconf_reg;
int console_detected;

static unsigned long save_console_padconf;

static u32 *uart_detect(void) {
	char str[7];

	if (console_detected)
		return console_padconf_reg;

	console_padconf_reg = NULL;
	console_detected = 0;

	if (console_detect(str))
		printk(KERN_INFO"Invalid console paramter\n");

	if (!strcmp(str, "ttyS0")) {
		console_padconf_reg = (u32 *)(&CONTROL_PADCONF_UART1_CTS);
		uart_interrupt_mask_value = UART1_INTERRUPT_MASK;
		}
	else if (!strcmp(str, "ttyS1")) {
		console_padconf_reg = (u32 *)(&CONTROL_PADCONF_UART2_TX);
		uart_interrupt_mask_value = UART2_INTERRUPT_MASK;
		}
	else if (!strcmp(str, "ttyS2")) {
		console_padconf_reg = (u32 *)(&CONTROL_PADCONF_UART3_RTS_SD);
		uart_interrupt_mask_value = UART3_INTERRUPT_MASK;
		}
	else if (!strcmp(str, "ttyS3")) {
		console_padconf_reg = (u32 *)(&CONTROL_PADCONF_UART3_RTS_SD);
		uart_interrupt_mask_value = UART3_INTERRUPT_MASK;
		}
	else
		printk(KERN_INFO
		"Unable to recongnize Console UART!\n");

	if (console_padconf_reg)
		console_detected = 1;

	return console_padconf_reg;
}


void complete_board_wakeup(void)
{
}

void  init_wakeupconfig(void)
{
	u32 *ptr;
	char str[7];
	if (console_detect(str))
		printk(KERN_INFO"Invalid console paramter\n");
	/* For quad uart we are currently not doing io pad wake up config */
	if (!strcmp(str, "ttyS3"))
		return;
	ptr = uart_detect();

}

int is_console_wakeup(void)
{
	u32 *ptr;
	char str[7];
	if (console_detect(str))
		printk(KERN_INFO"Invalid console paramter\n");
	/* For quad uart we are currently not doing io pad wake up config */
	if (!strcmp(str, "ttyS3"))
		return 0;

	ptr = uart_detect();	
	
	if ((*ptr >> IO_PAD_HIGH_SHIFT) & IO_PAD_WAKEUPEVENT) {
		*ptr = save_console_padconf;
		return 1;
	}
	
	*ptr = save_console_padconf;	
	return 0;
}

void setup_board_wakeup_source(u32 wakeup_source)
{
	if ((wakeup_source & PRCM_WAKEUP_T2_KEYPAD) ||
		(wakeup_source & PRCM_WAKEUP_ATMEGA) || 
		(wakeup_source & PRCM_WAKEUP_TOUCHSCREEN)) {
		PRCM_GPIO1_SYSCONFIG = 0x15;
		PM_WKEN_WKUP |= 0x8;
		PM_MPUGRPSEL_WKUP |= 0x8;
		/* Unmask GPIO interrupt*/
		INTC_MIR_0 = ~((1<<29));
		
		/* clear wakeup enable and irqenable */
		GPIO1_IRQENABLE1 = 0;
		GPIO1_WAKEUPENABLE = 0;
	}
	if (wakeup_source & PRCM_WAKEUP_T2_KEYPAD) {
		CONTROL_PADCONF_SYS_NIRQ &= 0xFFFFFFF8;
		CONTROL_PADCONF_SYS_NIRQ |= 0x4;
		GPIO1_SETIRQENABLE1 = 0x1;
		GPIO1_SETWKUENA = 0x1;
		GPIO1_RISINGDETECT |= 0x1;
	}
	if (wakeup_source & PRCM_WAKEUP_ATMEGA) {
		GPIO1_SETIRQENABLE1 = 0x2;
		GPIO1_SETWKUENA = 0x2;
		GPIO1_RISINGDETECT |= 0x2;
	}
	
	printk("GPIO1_IRQENABLE1:   %08x\n", GPIO1_IRQENABLE1);
	printk("GPIO1_WAKEUPENABLE: %08x\n", GPIO1_WAKEUPENABLE);
	
	/* Unmasking the PRCM interrupts */
	INTC_MIR_0 &= ~PRCM_INTERRUPT_MASK;

	if (console_padconf_reg) {
		save_console_padconf = *console_padconf_reg;
		if (wakeup_source & PRCM_WAKEUP_UART) {
			/* Unmasking the UART interrupts */
			INTC_MIR_2 = ~uart_interrupt_mask_value;
	
			*console_padconf_reg |= (u32)((IO_PAD_WAKEUPENABLE | IO_PAD_OFFPULLUDENABLE |
					IO_PAD_OFFOUTENABLE | IO_PAD_OFFENABLE |
					IO_PAD_INPUTENABLE | IO_PAD_PULLUDENABLE)
						<<  IO_PAD_HIGH_SHIFT);
	
		}
	}
}

void uart_padconf_control(void)
{
#if 0
	u32 *ptr;
	ptr = (u32 *)uart_detect();
	*ptr |= (u32)((IO_PAD_WAKEUPENABLE)
		<< IO_PAD_HIGH_SHIFT);
#endif
}

#endif

void archos_power_off(void)
{
	struct atmega_exchange_table reset_table = { 0, ATMEGA_I2C_CTRL_CMD_SHUTDOWN, 0,0,0 };
	struct atmega_exchange_table alarm_table = { 0, ATMEGA_I2C_CTRL_CMD_RESET_ALARM, 0, 0, 0 };

#ifdef CONFIG_OMAP3_PM
	/* make sure VDD1 is in OPP3 before reset */
	vdd1_opp_co = constraint_get("reset", &reset_constr);
	constraint_set(vdd1_opp_co, 3);
#endif
	if (atmega_write_table( &alarm_table ) < 0)
		printk(KERN_ERR "archos_power_off: cannot clear alarm!\n");
	atmega_write_table( &reset_table );
}

void archos_reset_board(void)
{
	struct atmega_exchange_table reset_table = { 0, ATMEGA_I2C_CTRL_CMD_RESET_DAVINCI, 0,0,0 };
	struct atmega_exchange_table alarm_table = { 0, ATMEGA_I2C_CTRL_CMD_RESET_ALARM, 0, 0, 0 };

#ifdef CONFIG_OMAP3_PM
	/* make sure VDD1 is in OPP3 before reset */
	vdd1_opp_co = constraint_get("reset", &reset_constr);
	constraint_set(vdd1_opp_co, 3);
#endif
	
	if (atmega_write_table( &alarm_table ) < 0)
		printk(KERN_ERR "archos_reset_board: cannot clear alarm!\n");
	atmega_write_table( &reset_table );
}
