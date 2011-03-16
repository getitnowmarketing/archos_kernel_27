/*
 * archos-tvout-hdmi.c
 *
 *  Created on: Feb 10, 2009
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.   
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <mach/display.h>
#include <mach/mux.h>
#include <mach/board-archos.h>
#include <mach/gpio.h>
#include <mach/archos-gpio.h>

static struct archos_hdmi_conf hdmi_platform_data;

static int panel_enable(struct omap_display *disp)
{
	pr_info("panel_enable [%s]\n", disp->panel->name);
	
	omap_set_gpio_dataout(GPIO_PIN(hdmi_platform_data.hdmi_dac), 1);
	omap_set_gpio_dataout(GPIO_PIN(hdmi_platform_data.disp_sel), 0);
	return 0;
}

static void panel_disable(struct omap_display *disp)
{
	pr_info("panel_disable [%s]\n", disp->panel->name);
	omap_set_gpio_dataout(GPIO_PIN(hdmi_platform_data.hdmi_dac), 0);
	omap_set_gpio_dataout(GPIO_PIN(hdmi_platform_data.disp_sel), 1);
}

static struct omap_display_data hdmi_panel = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "hdmi",
	.panel_name = "panel-hdmi",
	.u.dpi.data_lines=24,
	.panel_enable = panel_enable,
	.panel_disable = panel_disable,
};

static const struct i2c_board_info __initdata ad9x89_i2c_board_info[]  = {
	{
		I2C_BOARD_INFO("ad9x89_core", 0x39),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &hdmi_platform_data,
	},
	{	
		I2C_BOARD_INFO("ad9x89_edid", 0x3F),
		.flags = I2C_CLIENT_WAKE,
	},
	{
		I2C_BOARD_INFO("ad9x89_gmp", 0x38),
		.flags = I2C_CLIENT_WAKE,
	},
};

int __init archos_tvout_hdmi_init(struct omap_dss_platform_data *disp_data)
{
	const struct archos_display_config *disp_cfg;
	int ret = -ENODEV;
	
	disp_cfg = omap_get_config(ARCHOS_TAG_DISPLAY, struct archos_display_config);
	if (disp_cfg == NULL)
		return ret;

	if (hardware_rev >= disp_cfg->nrev) {
		printk(KERN_DEBUG "archos_tvout_hdmi_init: hardware_rev (%i) >= nrev (%i)",
				hardware_rev, disp_cfg->nrev);
		return ret;
	}
	
	hdmi_platform_data.hdmi_irq = disp_cfg->rev[hardware_rev].hdmi_it;
	hdmi_platform_data.hdmi_dac = disp_cfg->rev[hardware_rev].hdmi_dac;
	hdmi_platform_data.disp_sel = disp_cfg->rev[hardware_rev].disp_select;
	
	GPIO_INIT_OUTPUT(hdmi_platform_data.hdmi_dac);
	GPIO_INIT_INPUT(hdmi_platform_data.hdmi_irq);
	
	ret = i2c_register_board_info(3, ad9x89_i2c_board_info, 
			ARRAY_SIZE(ad9x89_i2c_board_info));
	if (ret < 0)
		pr_err("archos_tvout_hdmi_init: unable to register ad9x89 board info\n");

	disp_data->displays[disp_data->num_displays] = &hdmi_panel;
	disp_data->num_displays++;

	return 0;
}
