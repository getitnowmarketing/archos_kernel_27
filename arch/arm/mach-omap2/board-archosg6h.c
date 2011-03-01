/*
 * board-archosg6h.c
 *
 *  Created on: Jan 6, 2009
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/keypad.h>
#include <mach/gpmc.h>
#include <mach/hsmmc.h>
#include <mach/usb-musb.h>
#include <mach/usb-ehci.h>
#include <mach/mux.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <mach/control.h>
#include <mach/mux.h>
#include <mach/board-archos.h>
#include <mach/display.h>
#ifdef CONFIG_OMAP3_PM
#include "prcm-regs.h"
#include "ti-compat.h"
#include <mach/prcm_34xx.h>
#endif

#ifdef CONFIG_OMAP3_PM
#define CONTROL_SYSC_SMARTIDLE  	(0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE   	(0x1)
#endif

#ifdef CONFIG_OMAP3_PM
static void scm_clk_init(void)
{
	struct clk *p_omap_ctrl_clk = NULL;

	p_omap_ctrl_clk = clk_get(NULL, "omapctrl_ick");
	if (p_omap_ctrl_clk != NULL) {
		if (clk_enable(p_omap_ctrl_clk) != 0) {
			printk(KERN_ERR "failed to enable scm clks\n");
			clk_put(p_omap_ctrl_clk);
		}
	}
	/* Sysconfig set to SMARTIDLE and AUTOIDLE */
	CONTROL_SYSCONFIG = (CONTROL_SYSC_SMARTIDLE | CONTROL_SYSC_AUTOIDLE);
}
#endif

extern int __init archos_audio_gpio_init(void);

static void __init board_init_irq(void)
{
	omap2_init_common_hw(NULL);
	omap_init_irq();
#ifdef CONFIG_OMAP3_PM
	/* System Control module clock initialization */
	scm_clk_init();
#endif
	omap_gpio_init();
}

static struct archos_usb_config usb_config __initdata = {
	.nrev = 5, /* cover v0-v4 boards */
	.rev[0] = {
		.usb_id = { .nb = 56, .mux_cfg = R8_3430_GPIO56    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 58,  .mux_cfg = N8_3430_GPIO58   },
	},
	.rev[1] = {
		.usb_id = { .nb = 56, .mux_cfg = R8_3430_GPIO56    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 58,  .mux_cfg = N8_3430_GPIO58   },
	},
	.rev[2] = {
		.usb_id = { .nb = 156, .mux_cfg = Y21_3430_GPIO156    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 58,  .mux_cfg = N8_3430_GPIO58   },
	},
	.rev[3] = {
		.usb_id = { .nb = 156, .mux_cfg = Y21_3430_GPIO156    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 58,  .mux_cfg = N8_3430_GPIO58   },
	},
	.rev[4] = {
		.usb_id = { .nb = 156, .mux_cfg = Y21_3430_GPIO156    },
		.enable_usb_musb = { .nb = 20,  .mux_cfg = AF13_3430_GPIO20   },
		.enable_usb_ehci = { .nb = 58,  .mux_cfg = N8_3430_GPIO58   },
	},
};

static struct archos_display_config display_config __initdata = {
	.nrev = 5,  /* cover v0-v4 boards */
	.rev[0] = {
		.cpldreset =	{ 140, AF6_3430_GPIO140 },
		.disp_select = 	{ 14, AF11_3430_GPIO14 },
		.lcd_pwon = 	{ 143, AE5_3430_GPIO143 },
		.lcd_rst = 	{ 141, AE6_3430_GPIO141 },
		.bkl_pwon = 	{ 151, Y8_3430_GPIO151 },
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 152, AE1_3430_GPIO152 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 21, AH14_3430_GPIO21 },
		.bkl_pwm = 	{ .timer = 10, .mux_cfg = AB25_3430_GPT10 },
		.spi = {
			.spi_clk = { .nb = 137, .mux_cfg = AH3_3430_GPIO137 },
			.spi_data = { .nb = 138, .mux_cfg = AF3_3430_GPIO138 },
			.spi_cs = { .nb = 139, .mux_cfg = AE3_3430_GPIO139 },
		},
	},
	.rev[1] = {
		.cpldreset =	{ 140, AF6_3430_GPIO140 },
		.disp_select = 	{ 14, AF11_3430_GPIO14 },
		.lcd_pwon = 	{ 143, AE5_3430_GPIO143 },
		.lcd_rst = 	{ 141, AE6_3430_GPIO141 },
		.bkl_pwon = 	{ 151, Y8_3430_GPIO151 },
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 152, AE1_3430_GPIO152 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 21, AH14_3430_GPIO21 },
		.bkl_pwm = 	{ .timer = 10, .mux_cfg = AB25_3430_GPT10 },
		.spi = {
			.spi_clk = { .nb = 137, .mux_cfg = AH3_3430_GPIO137 },
			.spi_data = { .nb = 138, .mux_cfg = AF3_3430_GPIO138 },
			.spi_cs = { .nb = 139, .mux_cfg = AE3_3430_GPIO139 },
		},
	},
	.rev[2] = {
		.cpldreset =	{ 140, AF6_3430_GPIO140 },
		.disp_select = 	{ 14, AF11_3430_GPIO14 },
		.lcd_pwon = 	{ 143, AE5_3430_GPIO143 },
		.lcd_rst = 	{ 141, AE6_3430_GPIO141 },
		.bkl_pwon = 	{ 53, V8_3430_GPIO53 },
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 152, AE1_3430_GPIO152 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 21, AH14_3430_GPIO21 },
		.bkl_pwm = 	{ .timer = 10, .mux_cfg = AB25_3430_GPT10 },
		.spi = {
			.spi_clk = { .nb = 137, .mux_cfg = AH3_3430_GPIO137 },
			.spi_data = { .nb = 138, .mux_cfg = AF3_3430_GPIO138 },
			.spi_cs = { .nb = 139, .mux_cfg = AE3_3430_GPIO139 },
		},
	},
	.rev[3] = {
		.cpldreset =	{ 140, AF6_3430_GPIO140 },
		.disp_select = 	{ 14, AF11_3430_GPIO14 },
		.lcd_pwon = 	{ 143, AE5_3430_GPIO143 },
		.lcd_rst = 	{ 141, AE6_3430_GPIO141 },
		.bkl_pwon = 	{ 53, V8_3430_GPIO53 },
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 152, AE1_3430_GPIO152 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 21, AH14_3430_GPIO21 },
		.bkl_pwm = 	{ .timer = 10, .mux_cfg = AB25_3430_GPT10 },
		.spi = {
			.spi_clk = { .nb = 137, .mux_cfg = AH3_3430_GPIO137 },
			.spi_data = { .nb = 138, .mux_cfg = AF3_3430_GPIO138 },
			.spi_cs = { .nb = 139, .mux_cfg = AE3_3430_GPIO139 },
		},
	},
	.rev[4] = {
		.cpldreset =	{ 140, AF6_3430_GPIO140 },
		.disp_select = 	{ 14, AF11_3430_GPIO14 },
		.lcd_pwon = 	{ 143, AE5_3430_GPIO143 },
		.lcd_rst = 	{ 141, AE6_3430_GPIO141 },
		.bkl_pwon = 	{ 53, V8_3430_GPIO53 },
		.lcd_pci = 	{ 0, 0 },
		.hdmi_dac = 	{ 152, AE1_3430_GPIO152 },
		.hdmi_it = 	{ 15, AG12_3430_GPIO15 },
		.cpld_aux = 	{ 21, AH14_3430_GPIO21 },
		.bkl_pwm = 	{ .timer = 10, .mux_cfg = AB25_3430_GPT10 },
		.spi = {
			.spi_clk = { .nb = 137, .mux_cfg = AH3_3430_GPIO137 },
			.spi_data = { .nb = 138, .mux_cfg = AF3_3430_GPIO138 },
			.spi_cs = { .nb = 139, .mux_cfg = AE3_3430_GPIO139 },
		},
	},
};

static int __init archos_lcd_panel_init(struct omap_dss_platform_data *disp_data)
{
	switch (hardware_rev) {
	case 4:
	case 6:
	case 7:
		return panel_tpo_wvga_48_init(disp_data);
		
	default:
		return panel_samsung_wvga_48_init(disp_data);
	}
}

static struct omap_uart_config uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct archos_tsp_config tsp_config __initdata = {
	.nrev = 5, /* cover v0 to v4 boards */
	.rev[0] = {
		.irq_gpio = { .nb = 142, .mux_cfg = AF5_3430_GPIO142 },
		.pwr_gpio = { .nb = 148, .mux_cfg = AA8_3430_GPIO148 },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
	.rev[1] = {
		.irq_gpio = { .nb = 142, .mux_cfg = AF5_3430_GPIO142 },
		.pwr_gpio = { .nb = 148, .mux_cfg = AA8_3430_GPIO148 },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
	.rev[2] = {
		.irq_gpio = { .nb = 142, .mux_cfg = AF5_3430_GPIO142 },
		.pwr_gpio = { .nb = 56,  .mux_cfg = R8_3430_GPIO56   },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
	.rev[3] = {
		.irq_gpio = { .nb = 142, .mux_cfg = AF5_3430_GPIO142 },
		.pwr_gpio = { .nb = 56,  .mux_cfg = R8_3430_GPIO56   },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
	.rev[4] = {
		.irq_gpio = { .nb = 142, .mux_cfg = AF5_3430_GPIO142 },
		.pwr_gpio = { .nb = 56,  .mux_cfg = R8_3430_GPIO56   },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
};

static struct archos_keys_config keys_config __initdata = {
	.nrev = 5,  /* cover v0-v4 boards */
	.rev[0] = {
		.power = { .nb = 155, .mux_cfg = AC1_3430_GPIO155 },
		.vol_up = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_down = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
		.cam_first = UNUSED_GPIO,
		.cam_full = UNUSED_GPIO,
	},
	.rev[1] = {
		.power = { .nb = 155, .mux_cfg = AC1_3430_GPIO155 },
		.vol_up = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_down = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
		.cam_first = UNUSED_GPIO,
		.cam_full = UNUSED_GPIO,
	},
	.rev[2] = {
		.power = { .nb = 155, .mux_cfg = AC1_3430_GPIO155 },
		.vol_up = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_down = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
		.cam_first = UNUSED_GPIO,
		.cam_full = UNUSED_GPIO,
	},
	.rev[3] = {
		.power = { .nb = -1, .mux_cfg = -1 },
		.vol_up = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_down = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
		.cam_first = UNUSED_GPIO,
		.cam_full = UNUSED_GPIO,
	},
	.rev[4] = {
		.power = { .nb = -1, .mux_cfg = -1 },
		.vol_up = { .nb = 12, .mux_cfg = AF10_3430_GPIO12 },
		.vol_down = { .nb = 13, .mux_cfg = AE10_3430_GPIO13},
		.cam_first = UNUSED_GPIO,
		.cam_full = UNUSED_GPIO,
	},
};

static struct archos_audio_config audio_config __initdata = {
	.nrev = 8,
	.rev[0] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 22, .mux_cfg = AF9_3430_GPIO22 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[1] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 22, .mux_cfg = AF9_3430_GPIO22 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[2] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 22, .mux_cfg = AF9_3430_GPIO22 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[3] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 22, .mux_cfg = AF9_3430_GPIO22 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[4] = {
		.spdif = { .nb = 23, .mux_cfg = AG9_3430_GPIO23 },
		.hp_on = { .nb = 22, .mux_cfg = AF9_3430_GPIO22 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[5] = {
		.spdif = { .nb = 162, .mux_cfg = W21_3430_GPIO162 },
		.hp_on = { .nb = 157, .mux_cfg = AA21_3430_GPIO157 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[6] = {
		.spdif = { .nb = 162, .mux_cfg = W21_3430_GPIO162 },
		.hp_on = { .nb = 157, .mux_cfg = AA21_3430_GPIO157 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
	.rev[7] = {
		.spdif = { .nb = 162, .mux_cfg = W21_3430_GPIO162 },
		.hp_on = { .nb = 157, .mux_cfg = AA21_3430_GPIO157 },
		.headphone_plugged = { .nb = 17, .mux_cfg = AE13_3430_GPIO17},
	},
};

static struct archos_charge_config charge_config __initdata = {
	.nrev = 5,
	.rev[0] = { .nb = -1, .mux_cfg = -1 },
	.rev[1] = { .nb = -1, .mux_cfg = -1 },
	.rev[2] = { .nb = -1, .mux_cfg = -1 },
	.rev[3] = { .nb = 155, .mux_cfg = AC1_3430_GPIO155 },
	.rev[4] = { .nb = 155, .mux_cfg = AC1_3430_GPIO155 },
};

static struct archos_vbus_config vbus0_config __initdata = {
	.nrev = 5,  /* cover v0-v4 boards */
	.rev[0] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	.rev[1] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	.rev[2] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	.rev[3] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	.rev[4] = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
};

static struct archos_wifi_bt_config wifi_bt_dev_conf __initdata = {
	.nrev = 5,
	.rev[0] = { 	
		.wifi_power 	= { .nb = 186, .mux_cfg = AE22_3430_GPIO186 },
		.wifi_irq 	= { .nb = 0,   .mux_cfg = 0 },
		.bt_power 	= { .nb = 16,  .mux_cfg = 0 },
	},
	.rev[1] = { 	
		.wifi_power 	= { .nb = 186, .mux_cfg = AE22_3430_GPIO186 },
		.wifi_irq 	= { .nb = 0,   .mux_cfg = 0 },
		.bt_power 	= { .nb = 16,  .mux_cfg = 0 },
	},
	.rev[2] = { 	
		.wifi_power 	= { .nb = 54,  .mux_cfg = U8_3430_GPIO54 },
		.wifi_irq 	= { .nb = 0,   .mux_cfg = 0 },
		.bt_power 	= { .nb = 16,  .mux_cfg = 0 },
	},
	.rev[3] = { 	
		.wifi_power 	= { .nb = 54,  .mux_cfg = U8_3430_GPIO54 },
		.wifi_irq 	= { .nb = 0,   .mux_cfg = 0 },
		.bt_power 	= { .nb = 16,  .mux_cfg = 0 },
	},
	.rev[4] = { 	
		.wifi_power 	= { .nb = 54,  .mux_cfg = U8_3430_GPIO54 },
		.wifi_irq 	= { .nb = 0,   .mux_cfg = 0 },
		.bt_power 	= { .nb = 16,  .mux_cfg = 0 },
	},
};

static struct archos_sata_config sata_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.hdd_power = { .nb = 18, .mux_cfg = AE11_3430_GPIO18 },
		.sata_power = { .nb = 57, .mux_cfg = P8_3430_GPIO57 },
		.sata_ready = { .nb = 52, .mux_cfg = H3_3430_GPIO52_G6},
	},
	.rev[1] = {
		.hdd_power = { .nb = 18, .mux_cfg = AE11_3430_GPIO18 },
		.sata_power = { .nb = 57, .mux_cfg = P8_3430_GPIO57 },
		.sata_ready = { .nb = 52, .mux_cfg = H3_3430_GPIO52_G6},
	},
	.rev[2] = {
		.hdd_power = { .nb = 18, .mux_cfg = AE11_3430_GPIO18 },
		.sata_power = { .nb = 57, .mux_cfg = P8_3430_GPIO57 },
		.sata_ready = { .nb = 52, .mux_cfg = H3_3430_GPIO52_G6},
	},
	.rev[3] = {
		.hdd_power = { .nb = 18, .mux_cfg = AE11_3430_GPIO18 },
		.sata_power = { .nb = 57, .mux_cfg = P8_3430_GPIO57 },
		.sata_ready = { .nb = 52, .mux_cfg = H3_3430_GPIO52_G6},
	},
	.rev[4] = {
		.hdd_power = { .nb = 18, .mux_cfg = AE11_3430_GPIO18 },
		.sata_power = { .nb = 57, .mux_cfg = P8_3430_GPIO57 },
		.sata_ready = { .nb = 52, .mux_cfg = H3_3430_GPIO52_G6},
	},
};

static struct archos_irblaster_config irblaster_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.irblaster_pwm = { .timer = -1, .mux_cfg = 0 },
		.irblaster_pwm_disable = { .nb = -1, .mux_cfg = 0 },
		.irblaster_ctrl_timer = { .timer = -1, .mux_cfg = 0 },
	},
	.rev[1] = {
		.irblaster_pwm = { .timer = -1, .mux_cfg = 0 },
		.irblaster_pwm_disable = { .nb = -1, .mux_cfg = 0 },
		.irblaster_ctrl_timer = { .timer = -1, .mux_cfg = 0 },
	},
	.rev[2] = {
		.irblaster_pwm = { .timer = -1, .mux_cfg = 0 },
		.irblaster_pwm_disable = { .nb = -1, .mux_cfg = 0 },
		.irblaster_ctrl_timer = { .timer = -1, .mux_cfg = 0 },
	},
	.rev[3] = {
		.irblaster_pwm = { .timer = -1, .mux_cfg = 0 },
		.irblaster_pwm_disable = { .nb = -1, .mux_cfg = 0 },
		.irblaster_ctrl_timer = { .timer = -1, .mux_cfg = 0 },
	},
	.rev[4] = {
		.irblaster_pwm = { .timer = -1, .mux_cfg = 0 },
		.irblaster_pwm_disable = { .nb = -1, .mux_cfg = 0 },
		.irblaster_ctrl_timer = { .timer = -1, .mux_cfg = 0 },
	},
};

static struct archos_irremote_config irremote_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.irremote_timer = { .timer = -1, .mux_cfg = 0 },
		.irremote_timer_disable = { .nb = -1, .mux_cfg = 0 },
	},
	.rev[1] = {
		.irremote_timer = { .timer = -1, .mux_cfg = 0 },
		.irremote_timer_disable = { .nb = -1, .mux_cfg = 0 },
	},
	.rev[2] = {
		.irremote_timer = { .timer = -1, .mux_cfg = 0 },
		.irremote_timer_disable = { .nb = -1, .mux_cfg = 0 },
	},
	.rev[3] = {
		.irremote_timer = { .timer = -1, .mux_cfg = 0 },
		.irremote_timer_disable = { .nb = -1, .mux_cfg = 0 },
	},
	.rev[4] = {
		.irremote_timer = { .timer = -1, .mux_cfg = 0 },
		.irremote_timer_disable = { .nb = -1, .mux_cfg = 0 },
	},
};

static struct archos_uart3_config uart3_config __initdata = {
	.nrev = 5,
	.rev[0] = {
		.uart3_rx_mux = H20_3430_UART3_RX_IRRX,
		.uart3_tx_mux = H21_3430_UART3_TX_IRTX,
		.gpio_uart3_rx = { .nb = 165, .mux_cfg = H20_3430_GPIO165 },
		.gpio_uart3_tx = { .nb = 166, .mux_cfg = H21_3430_GPIO166 },
	},
	.rev[1] = {
		.uart3_rx_mux = H20_3430_UART3_RX_IRRX,
		.uart3_tx_mux = H21_3430_UART3_TX_IRTX,
		.gpio_uart3_rx = { .nb = 165, .mux_cfg = H20_3430_GPIO165 },
		.gpio_uart3_tx = { .nb = 166, .mux_cfg = H21_3430_GPIO166 },
	},
	.rev[2] = {
		.uart3_rx_mux = H20_3430_UART3_RX_IRRX,
		.uart3_tx_mux = H21_3430_UART3_TX_IRTX,
		.gpio_uart3_rx = { .nb = 165, .mux_cfg = H20_3430_GPIO165 },
		.gpio_uart3_tx = { .nb = 166, .mux_cfg = H21_3430_GPIO166 },
	},
	.rev[3] = {
		.uart3_rx_mux = H20_3430_UART3_RX_IRRX,
		.uart3_tx_mux = H21_3430_UART3_TX_IRTX,
		.gpio_uart3_rx = { .nb = 165, .mux_cfg = H20_3430_GPIO165 },
		.gpio_uart3_tx = { .nb = 166, .mux_cfg = H21_3430_GPIO166 },
	},
	.rev[4] = {
		.uart3_rx_mux = H20_3430_UART3_RX_IRRX,
		.uart3_tx_mux = H21_3430_UART3_TX_IRTX,
		.gpio_uart3_rx = { .nb = 165, .mux_cfg = H20_3430_GPIO165 },
		.gpio_uart3_tx = { .nb = 166, .mux_cfg = H21_3430_GPIO166 },
	},
};

static struct archos_atmega_config atmega_config __initdata = {
	.name = "atmegag6-io",
	.irq = { .nb = 170, .mux_cfg = J25_3430_GPIO170, },
};

static struct omap_dss_platform_data board_dss_data = {
	.num_displays = 0,
	.displays = {
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
	},
};
static struct platform_device board_dss_device = {
	.name		= "omap-dss",
	.id		= -1,
	.dev            = {
		.platform_data = &board_dss_data,
	},
};

static struct platform_device omap_tvp_isp_device = {
	.name		= "omap-tvp-isp",
	.id		= -1,
	.dev            = {
		.platform_data = NULL,
	},
};

static struct omap_board_config_kernel board_config[] __initdata = {
	{ OMAP_TAG_UART,	&uart_config },
	{ ARCHOS_TAG_USB,	&usb_config },
	{ ARCHOS_TAG_KEYS,	&keys_config},
	{ ARCHOS_TAG_DISPLAY,	&display_config },
	{ ARCHOS_TAG_TSP,	&tsp_config },
	{ ARCHOS_TAG_CHARGE,	&charge_config},
	{ ARCHOS_TAG_AUDIO,     &audio_config},
	{ ARCHOS_TAG_VBUS0,	&vbus0_config},
	{ ARCHOS_TAG_WIFI_BT,	&wifi_bt_dev_conf},
	{ ARCHOS_TAG_SATA,	&sata_config},
	{ ARCHOS_TAG_IRBLASTER,	&irblaster_config},
	{ ARCHOS_TAG_IRREMOTE,	&irremote_config},
	{ ARCHOS_TAG_UART3,	&uart3_config},
	{ ARCHOS_TAG_ATMEGA,	&atmega_config},
};

static struct i2c_board_info __initdata board_i2c_bus1_info[] = {
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
	},
};

static struct i2c_board_info __initdata board_i2c_bus2_info[] = {
	{
		I2C_BOARD_INFO("atmega", 0x2d),
		.flags = I2C_CLIENT_WAKE,
	},
	{
		I2C_BOARD_INFO("wm87xx", 0x1a),
		.flags = I2C_CLIENT_WAKE,
	}
};

static int __init omap_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, board_i2c_bus1_info,
			ARRAY_SIZE(board_i2c_bus1_info));
	omap_register_i2c_bus(2, 100, board_i2c_bus2_info, 
			ARRAY_SIZE(board_i2c_bus2_info));
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}

static void __init board_init(void)
{
	omap_i2c_init();
	
	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

#ifdef CONFIG_OMAP3_PM
	prcm_init();
#endif

	if (archos_lcd_panel_init(&board_dss_data) < 0)
		pr_err("archos_lcd_panel_init failed!\n");
	if (archos_tvout_venc_init(&board_dss_data) < 0)
		pr_err("archos_tvout_venc_init failed\n");
	if (archos_tvout_hdmi_init(&board_dss_data) < 0)
		pr_err("archos_tvout_hdmi_init failed\n");
	if (archos_tvout_extdac_init(&board_dss_data) < 0)
		pr_err("archos_tvout_extdac_init failed\n");
	
	platform_device_register(&board_dss_device);

	platform_device_register(&omap_tvp_isp_device);

	archosg6_init();
	archos_atmega_init();
	ads7846_dev_init();
	omap_serial_init();
	usb_musb_init();
	hsmmc_init();
	usb_ehci_init();
	archos_usb2sata_init();
	archos_audio_gpio_init();
	archos_keys_init();
	archos_wifi_bt_init();

	pm_power_off = archos_power_off;
	printk("board_init done\n");
	msleep(200);
}

static void __init board_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(ARCHOS_G6H, "Archos G6H board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
#ifdef CONFIG_ARCHOS_FIXUP
	.fixup		= fixup_archos,
#endif
	.map_io		= board_map_io,
	.init_irq	= board_init_irq,
	.init_machine	= board_init,
	.timer		= &omap_timer,
MACHINE_END
