/*
 * board-archos-a5st.c
 *
 *  Created on: Feb 11, 2009
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
#include <mach/gpmc.h>
#ifdef CONFIG_OMAP3_PM
#include "prcm-regs.h"
#include "ti-compat.h"
#include <mach/prcm_34xx.h>
#endif

#include <linux/mma7456l.h>

#ifdef CONFIG_OMAP3_PM
#define CONTROL_SYSC_SMARTIDLE  	(0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE   	(0x1)
#endif

static struct mma7456l_pdata board_mma7456l_pdata;

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

/** 
* init_buffer_pbias
* init for highpart of MMC1 bus used as GPIO
* set VDD_SIM as 1.8 and validate buffers
*/
#define OMAP2_CONTROL_PBIAS_VMODE1	(1 << 8)
#define OMAP2_CONTROL_PBIAS_PWRDNZ1	(1 << 9)
#define OMAP2_CONTROL_PBIAS 0x48002520
static void init_buffer_pbias(void) 
{
	u32 reg;
	reg = omap_readl(OMAP2_CONTROL_PBIAS);

	// set gpio interface in 1.8V
	reg &= ~OMAP2_CONTROL_PBIAS_VMODE1;
	// set VDDS as stable
	reg |= OMAP2_CONTROL_PBIAS_PWRDNZ1;
	omap_writel(reg, OMAP2_CONTROL_PBIAS);

}

static struct archos_usb_config usb_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.usb_id = UNUSED_GPIO,
		.enable_usb_musb = { .nb = 161,  .mux_cfg = K26_3430_GPIO161 },	//enable0
		.enable_usb_ehci = { .nb = 41, .mux_cfg = M3_3430_GPIO41 },	//enable1
	},
	.rev[1] = {
		.usb_id = UNUSED_GPIO,
		.enable_usb_musb = { .nb = 161,  .mux_cfg = K26_3430_GPIO161 },	//enable0
		.enable_usb_ehci = { .nb = 41, .mux_cfg = M3_3430_GPIO41 },	//enable1
	},
};

static struct archos_display_config display_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.cpldreset = 	{ 136,	AE4_3430_GPIO136 },
		.disp_select = 	{ 53,	V8_3430_GPIO53 },
		.lcd_pwon = 	{ 157,	AA21_3430_GPIO157 },
		.lcd_rst = 	{ 137,	AH3_3430_GPIO137 },
		.bkl_pwon = 	UNUSED_GPIO,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_dac = 	{ 126,	P27_3430_GPIO126 },
		.hdmi_it = 	{ 100,	AH17_3430_GPIO100 },
		.cpld_aux = 	{ 163,	H18_3430_GPIO163 },
		.bkl_pwm = 	{ .timer = 10, .mux_cfg = R8_3430_GPT10 },
		.vcom_pwm = 	{ .timer = 11, .mux_cfg = P8_3430_GPT11 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
	.rev[1] = {
		.cpldreset = 	{ 136,	AE4_3430_GPIO136 },
		.disp_select = 	{ 53,	V8_3430_GPIO53 },
		.lcd_pwon = 	{ 157,	AA21_3430_GPIO157 },
		.lcd_rst = 	{ 137,	AH3_3430_GPIO137 },
		.bkl_pwon = 	UNUSED_GPIO,
		.lcd_pci = 	UNUSED_GPIO,
		.hdmi_dac = 	{ 126,	P27_3430_GPIO126 },
		.hdmi_it = 	{ 100,	AH17_3430_GPIO100 },
		.cpld_aux = 	{ 163,	H18_3430_GPIO163 },
		.bkl_pwm = 	{ .timer = 10, .mux_cfg = R8_3430_GPT10 },
		.vcom_pwm = 	{ .timer = 11, .mux_cfg = P8_3430_GPT11 },
		.spi = {
			.spi_clk  = UNUSED_GPIO,
			.spi_data = UNUSED_GPIO,
			.spi_cs   = UNUSED_GPIO,
		},
	},
};

static int __init archos_lcd_panel_init(struct omap_dss_platform_data *disp_data)
{
	switch (hardware_rev) {
	default:
		return panel_cpt_wvga_48_init(disp_data);
	}
}

static struct omap_uart_config uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct archos_keys_config keys_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.power = UNUSED_GPIO,
		.vol_down = { .nb = 113, .mux_cfg = AH19_3430_GPIO113 },
		.vol_up = { .nb = 112, .mux_cfg = AG19_3430_GPIO112},
		.cam_first = UNUSED_GPIO,
		.cam_full = UNUSED_GPIO,
	},
	.rev[1] = {
		.power = UNUSED_GPIO,
		.vol_down = { .nb = 113, .mux_cfg = AH19_3430_GPIO113 },
		.vol_up = { .nb = 112, .mux_cfg = AG19_3430_GPIO112},
		.cam_first = UNUSED_GPIO,
		.cam_full = UNUSED_GPIO,
	},
};

static struct archos_tsp_config tsp_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.irq_gpio = { .nb = 11, .mux_cfg = AA11_3430_GPIO11 },
		.pwr_gpio = { .nb = 54, .mux_cfg = U8_3430_GPIO54 },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
	.rev[1] = {
		.irq_gpio = { .nb = 11, .mux_cfg = AA11_3430_GPIO11 },
		.pwr_gpio = { .nb = 54, .mux_cfg = U8_3430_GPIO54 },
		.x_plate_ohms = 745,
		.pressure_max = 700,
	},
};

static struct archos_audio_config audio_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.spdif = { .nb = 176, .mux_cfg = AB1_3430_GPIO176 },
		.hp_on = { .nb = 170, .mux_cfg = J25_34XX_GPIO170 },
		.headphone_plugged = { .nb = 99, .mux_cfg = AG17_3430_GPIO99},
	},
	.rev[1] = {
		.spdif = { .nb = 176, .mux_cfg = AB1_3430_GPIO176 },
		.hp_on = { .nb = 170, .mux_cfg = J25_34XX_GPIO170 },
		.headphone_plugged = { .nb = 99, .mux_cfg = AG17_3430_GPIO99},
	},
};

static struct archos_charge_config charge_config __initdata = {
	.nrev = 2,
	.rev[0] = { .nb = 159, .mux_cfg = U21_3430_GPIO159 },
	.rev[1] = { .nb = 159, .mux_cfg = U21_3430_GPIO159 },
};

static struct archos_vbus_config vbus0_config __initdata = {
	.nrev = 2,
	.rev[0] = { .nb = 61, .mux_cfg = U3_3430_GPIO61 },
	.rev[1] = { .nb = 61, .mux_cfg = U3_3430_GPIO61 },
};

static struct archos_wifi_bt_config wifi_bt_dev_conf __initdata = {
	.nrev = 2,
	.rev[0] = {
		.wifi_power 	= { .nb = 111, .mux_cfg = B26_3430_GPIO111 },
		.wifi_irq 	= { .nb = 31,  .mux_cfg = AA10_3430_GPIO31 },
		.bt_power 	= { .nb = 162, .mux_cfg = W21_3430_GPIO162 },
	},
	.rev[1] = {
		.wifi_power 	= { .nb = 111, .mux_cfg = B26_3430_GPIO111 },
		.wifi_irq 	= { .nb = 31,  .mux_cfg = AA10_3430_GPIO31 },
		.bt_power 	= { .nb = 162, .mux_cfg = W21_3430_GPIO162 },
	},
};

static struct archos_usbhdd_config usbhdd_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.hdd_power = { .nb = 129, .mux_cfg = R25_3430_GPIO129 },
	},
	.rev[1] = {
		.hdd_power = { .nb = 129, .mux_cfg = R25_3430_GPIO129 },
	},
};

static struct archos_sd_config sd_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.sd_power = { .nb = 158, .mux_cfg = V21_3430_GPIO158 },
		.sd_detect = { .nb = 65, .mux_cfg = J8_3430_GPIO65 },
	},
	.rev[1] = {
		.sd_power = { .nb = 158, .mux_cfg = V21_3430_GPIO158 },
		.sd_detect = { .nb = 65, .mux_cfg = J8_3430_GPIO65 },
	},
};

static struct archos_gps_config gps_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.gps_enable = { .nb = 102, .mux_cfg = C24_3430_GPIO102 },
		.gps_int = { .nb = 175, .mux_cfg = AC3_3430_GPIO175 },
		.gps_reset = { .nb = 160, .mux_cfg = T21_3430_GPIO160 },
	},
	.rev[1] = {
		.gps_enable = { .nb = 102, .mux_cfg = C24_3430_GPIO102 },
		.gps_int = { .nb = 175, .mux_cfg = AC3_3430_GPIO175 },
		.gps_reset = { .nb = 160, .mux_cfg = T21_3430_GPIO160 },
	},
};

static struct archos_irblaster_config irblaster_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.irblaster_pwm = { .timer = 8, .mux_cfg = N8_3430_GPT8 },
		.irblaster_pwm_disable = { .nb = 58, .mux_cfg = N8_3430_GPIO58 },
		.irblaster_ctrl_timer = { .timer = 3, .mux_cfg = 0 },
	},
	.rev[1] = {
		.irblaster_pwm = { .timer = 8, .mux_cfg = N8_3430_GPT8 },
		.irblaster_pwm_disable = { .nb = 58, .mux_cfg = N8_3430_GPIO58 },
		.irblaster_ctrl_timer = { .timer = 3, .mux_cfg = 0 },
	},
};

static struct archos_irremote_config irremote_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.irremote_timer = { .timer = 9, .mux_cfg = T8_3430_GPT9 },
		.irremote_timer_disable = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	},
	.rev[1] = {
		.irremote_timer = { .timer = 9, .mux_cfg = T8_3430_GPT9 },
		.irremote_timer_disable = { .nb = 55, .mux_cfg = T8_3430_GPIO55 },
	},
};

static struct archos_uart3_config uart3_config __initdata = {
	.nrev = 2,
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
};

static struct archos_accel_config accel_config __initdata = {
	.nrev = 2,
	.rev[0] = {
		.accel_int1 = { .nb = 115, .mux_cfg = AH18_3430_GPIO115},
		.accel_int2 = { .nb = 114, .mux_cfg = AG18_3430_GPIO114},
	},
	.rev[1] = {
		.accel_int1 = { .nb = 115, .mux_cfg = AH18_3430_GPIO115},
		.accel_int2 = { .nb = 114, .mux_cfg = AG18_3430_GPIO114},
	},
};

static struct archos_atmega_config atmega_config __initdata = {
	.name = "atmegag7-io",
	.irq = { .nb = 1, .mux_cfg = AF25_3430_GPIO1, },
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
	{ ARCHOS_TAG_USBHDD,    &usbhdd_config},
	{ ARCHOS_TAG_SD,    	&sd_config},
	{ ARCHOS_TAG_GPS,	&gps_config},
	{ ARCHOS_TAG_IRBLASTER,	&irblaster_config},
	{ ARCHOS_TAG_IRREMOTE,	&irremote_config},
	{ ARCHOS_TAG_UART3,	&uart3_config},
	{ ARCHOS_TAG_ACCEL,	&accel_config},
	{ ARCHOS_TAG_ATMEGA,	&atmega_config},
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
		I2C_BOARD_INFO("mma7456l", 0x1d),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &board_mma7456l_pdata,
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
	omap_board_config = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

	archos_accel_init(&board_mma7456l_pdata);
	omap_i2c_init();

#ifdef CONFIG_OMAP3_PM
	prcm_init();
#endif

	init_buffer_pbias();

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

	archosg7_init();
	archos_flash_init();
	archos_atmega_init();
	ads7846_dev_init();
	omap_serial_init();
	usb_musb_init();
	hsmmc_init();
	usb_ehci_init();
	archos_usbhdd_init();
	archos_audio_gpio_init();
	archos_keys_init();
	archos_wifi_bt_init();

	omap_cfg_reg(AH26_3430_GPIO2);
	
	pm_power_off = archos_power_off;
}

static void __init board_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(ARCHOS_A5ST, "Archos A5ST board")
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
