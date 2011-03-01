/*
 * archos-lcd-samsung.c
 *
 *  Created on: Jan 13, 2009
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>

#include <asm/mach-types.h>

#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/omapfb.h>
#include <mach/display.h>
#include <mach/archos-gpio.h>
#include <mach/board-archos.h>
#include <mach/spi-gpio.h>
#include <mach/dmtimer.h>

static struct archos_disp_conf display_gpio;
static struct spigpio_info lcd_spi_conf;
static int bkl_level;
static int saved_bkl_level;
static struct omap_dm_timer *bkl_pwm;
static int panel_state;

static int panel_set_backlight_level(
		struct omap_display *disp, int level);

/* set state of spi pins */
static void spi_gpio_init_state( int state )
{
	pr_debug("spi_gpio_init_state  %d \n", state);
	if (state) {
		omap_set_gpio_dataout(lcd_spi_conf.pin_clk, 1);
		omap_set_gpio_dataout(lcd_spi_conf.pin_mosi, 0);
		omap_set_gpio_dataout(lcd_spi_conf.pin_cs, 1);
	} else {
		// set all pins to 0
		omap_set_gpio_dataout(lcd_spi_conf.pin_clk, 0);
		omap_set_gpio_dataout(lcd_spi_conf.pin_mosi, 0);
		omap_set_gpio_dataout(lcd_spi_conf.pin_cs, 0);
	}
}

static int panel_init(struct omap_display_data *ddata)
{
	pr_debug("panel_init [%s]\n", ddata->panel_name);

	GPIO_INIT_OUTPUT(display_gpio.lcd_pwon);
	GPIO_INIT_OUTPUT(display_gpio.lcd_rst);
	GPIO_INIT_OUTPUT(display_gpio.lcd_pci);
	GPIO_INIT_OUTPUT(display_gpio.disp_select);
	GPIO_INIT_OUTPUT(display_gpio.cpldreset);

#if !defined(CONFIG_FB_OMAP_BOOTLOADER_INIT)
	if (GPIO_EXISTS(display_gpio.lcd_pwon))
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.lcd_pwon), 0);
	if (GPIO_EXISTS(display_gpio.lcd_rst))
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.lcd_rst), 0);
	if (GPIO_EXISTS(display_gpio.lcd_pci))
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.lcd_pci), 0);

	/* reset and enable the CPLD */
	if (GPIO_EXISTS(display_gpio.cpldreset))
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.cpldreset), 0);
	mdelay(2);
	if (GPIO_EXISTS(display_gpio.cpldreset))
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.cpldreset), 1);
#endif
	return 0;
}

static int panel_enable(struct omap_display *disp)
{
	pr_info("panel_enable [%s]\n", disp->panel->name);

	if (GPIO_EXISTS(display_gpio.disp_select))
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.disp_select), 1);

	if (GPIO_EXISTS(display_gpio.lcd_pwon)) {
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.lcd_pwon), 1 );
		msleep(50);
	}
	if (GPIO_EXISTS(display_gpio.lcd_rst)) {
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.lcd_rst), 1 );
		msleep(100);
	}
	if (GPIO_EXISTS(display_gpio.lcd_pci))
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.lcd_pci), 1 );

	spi_gpio_init_state(1);
	panel_state = 1;
	panel_set_backlight_level(disp, saved_bkl_level);
	return 0;
}

static void panel_disable(struct omap_display *disp)
{
	pr_debug("panel_disable [%s]\n", disp->panel->name);

	spi_gpio_init_state(0);

	if (GPIO_EXISTS(display_gpio.lcd_rst))
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.lcd_rst), 0 );
	if (GPIO_EXISTS(display_gpio.lcd_pci))
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.lcd_pci), 0 );
	if (GPIO_EXISTS(display_gpio.lcd_pwon))
		omap_set_gpio_dataout( GPIO_PIN(display_gpio.lcd_pwon), 0 );
	
	saved_bkl_level = bkl_level;
	panel_set_backlight_level(disp, 0);
	panel_state = 0;
}


static void pwm_set_speed(struct omap_dm_timer *gpt,
		int frequency, int duty_cycle)
{
	u32 val;
	u32 period;
	struct clk *timer_fclk;

	/* and you will have an overflow in 1 sec         */
	/* so,                              */
	/* freq_timer     -> 1s             */
	/* carrier_period -> 1/carrier_freq */
	/* => carrier_period = freq_timer/carrier_freq */

	timer_fclk = omap_dm_timer_get_fclk(gpt);
	period = clk_get_rate(timer_fclk) / frequency;

	val = 0xFFFFFFFF+1-period;
	omap_dm_timer_set_load(gpt, 1, val);

	val = 0xFFFFFFFF+1-(period*duty_cycle/256);
	omap_dm_timer_set_match(gpt, 1, val);

	/* assume overflow first: no toogle if first trig is match */
	omap_dm_timer_write_counter(gpt, 0xFFFFFFFE);
}

static int panel_set_backlight_level(
		struct omap_display *disp, int level)
{
	int hw_level;

	pr_debug("panel_set_backlight_level [%s] %i\n",
			disp != NULL ? disp->panel->name: "samsung_wvga_48", level);

	/* skip if panel is not on */
	if (panel_state == 0) {
		saved_bkl_level = level;
		return 0;
	}
	
	/* clamp the level */
	if (level < 0)
		level = 0;
	if (level > 255)
		level = 255;

	/* nothing to do if levels are equal */
	if (bkl_level == level)
		return 0;

	/* stop backlight? */
	if (level == 0) {
		if (GPIO_EXISTS(display_gpio.bkl_pwon))
			omap_set_gpio_dataout(GPIO_PIN(display_gpio.bkl_pwon), 0);
		omap_dm_timer_stop(bkl_pwm);
		omap_dm_timer_disable(bkl_pwm);
		bkl_level = 0;
		return 0;
	}

	/* start backlight? */
	if (bkl_level == 0) {
		omap_dm_timer_enable(bkl_pwm);
		omap_dm_timer_set_pwm(bkl_pwm, 0, 1, 2);
		omap_dm_timer_start(bkl_pwm);
		if (GPIO_EXISTS(display_gpio.bkl_pwon))
			omap_set_gpio_dataout(GPIO_PIN(display_gpio.bkl_pwon), 1);
	}

	/* set new level, g7 machines have inverted level */
	hw_level = level;
	if (!machine_is_archos_g6h() && !machine_is_archos_g6s()
			/* && !machine_is_archos_g6l() && !machine_is_archos_g6plus() */)
		hw_level = 255 - level;
	pwm_set_speed(bkl_pwm, 10000, hw_level);
	bkl_level = level;
	return 0;
}

static struct omap_display_data samsung_wvga_48_panel = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.panel_name = "samsung_wvga_48",
	.u.dpi.data_lines=24,
	.panel_enable = panel_enable,
	.panel_disable = panel_disable,
	.set_backlight = panel_set_backlight_level,
};

static struct spi_board_info lcd_spi_board_info[] = {
	[0] = {
		.modalias	= "hx5091",
		.bus_num	= 5,
		.chip_select	= 0,
		.max_speed_hz   = 1500000,
		.controller_data = &lcd_spi_conf,
	}
};

static struct platform_device panel_spi_device = {
	.name		  = "omap3-spi-gpio",
	.id		  = 5,
	.dev.platform_data = &lcd_spi_conf,
};

int __init panel_samsung_wvga_48_init(struct omap_dss_platform_data *disp_data)
{
	const struct archos_display_config *disp_cfg;
	int ret = -ENODEV;

	disp_cfg = omap_get_config( ARCHOS_TAG_DISPLAY, struct archos_display_config );
	if (disp_cfg == NULL)
		return ret;

	if ( hardware_rev >= disp_cfg->nrev ) {
		printk(KERN_DEBUG "archos_display_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, disp_cfg->nrev);
		return ret;
	}

	display_gpio = disp_cfg->rev[hardware_rev];

	/* Initialize SPI bus lines */
	GPIO_INIT_OUTPUT(display_gpio.spi.spi_clk);
	GPIO_INIT_OUTPUT(display_gpio.spi.spi_cs);
	GPIO_INIT_OUTPUT(display_gpio.spi.spi_data);
	lcd_spi_conf.pin_clk = GPIO_PIN(display_gpio.spi.spi_clk);
	lcd_spi_conf.pin_mosi = GPIO_PIN(display_gpio.spi.spi_data);
	lcd_spi_conf.pin_cs = GPIO_PIN(display_gpio.spi.spi_cs);

	spi_gpio_init_state(0);
	spi_register_board_info(lcd_spi_board_info, 1);

	/*
	 * Backlight configuration,
	 * TODO: retrieve GPT id and mux through omap_get_config()
	 */
	GPIO_INIT_OUTPUT(display_gpio.bkl_pwon);
	bkl_pwm = omap_dm_timer_request_specific(display_gpio.bkl_pwm.timer);
	if (bkl_pwm) {
		omap_dm_timer_set_source(bkl_pwm, OMAP_TIMER_SRC_SYS_CLK);
		omap_cfg_reg(display_gpio.bkl_pwm.mux_cfg);
	} else
		pr_err("panel_samsung_wvga_48_init: no backlight PWM\n");

	ret = platform_device_register(&panel_spi_device);
	if (ret < 0) {
		pr_err("panel_samsung_wvga_48_init: cannot register panel spi\n");
		omap_dm_timer_free(bkl_pwm);
		return ret;
	}

	panel_init(&samsung_wvga_48_panel);
#if defined(CONFIG_FB_OMAP_BOOTLOADER_INIT)
	panel_set_backlight_level(NULL, 255);
#endif
	disp_data->displays[disp_data->num_displays] = &samsung_wvga_48_panel;
	disp_data->num_displays++;

	return 0;
}
