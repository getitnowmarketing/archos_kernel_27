/*
 * archos-tvout-extdac.c
 *
 *  Created on: Mar 5, 2009
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

static struct archos_extdac_conf extdac_platform_data;

static int panel_enable(struct omap_display *disp)
{
	pr_debug("panel_enable [%s]\n", disp->panel->name);
	
	omap_set_gpio_dataout(GPIO_PIN(extdac_platform_data.hdmi_dac), 0);
	omap_set_gpio_dataout(GPIO_PIN(extdac_platform_data.disp_sel), 0);
	return 0;
}

static void panel_disable(struct omap_display *disp)
{
	pr_debug("panel_disable [%s]\n", disp->panel->name);
	omap_set_gpio_dataout(GPIO_PIN(extdac_platform_data.hdmi_dac), 0);
	omap_set_gpio_dataout(GPIO_PIN(extdac_platform_data.disp_sel), 1);
}

static struct omap_display_data extdac_panel = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "extdac",
	.panel_name = "adv734x",
	.u.dpi.data_lines = 24,
	.panel_enable = panel_enable,
	.panel_disable = panel_disable,
	.priv = &extdac_platform_data,
};

static const struct i2c_board_info __initdata adv734x_i2c_board_info[]  = 
{
	{
		I2C_BOARD_INFO("adv734x", 0x2a),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &extdac_platform_data,
	},
};

int __init archos_tvout_extdac_init(struct omap_dss_platform_data *disp_data)
{
	const struct archos_display_config *disp_cfg;
	int ret = -ENODEV;
	
	disp_cfg = omap_get_config(ARCHOS_TAG_DISPLAY, struct archos_display_config);
	if (disp_cfg == NULL)
		return ret;

	if (hardware_rev >= disp_cfg->nrev) {
		printk(KERN_DEBUG "archos_tvout_extdac_init: hardware_rev (%i) >= nrev (%i)",
				hardware_rev, disp_cfg->nrev);
		return ret;
	}
	
	extdac_platform_data.cpld_aux = disp_cfg->rev[hardware_rev].cpld_aux;
	extdac_platform_data.hdmi_dac = disp_cfg->rev[hardware_rev].hdmi_dac;
	extdac_platform_data.disp_sel = disp_cfg->rev[hardware_rev].disp_select;
	
	GPIO_INIT_OUTPUT(extdac_platform_data.hdmi_dac);
	GPIO_INIT_OUTPUT(extdac_platform_data.disp_sel);
	GPIO_INIT_OUTPUT(extdac_platform_data.cpld_aux);
	
	ret = i2c_register_board_info(3, adv734x_i2c_board_info, 
			ARRAY_SIZE(adv734x_i2c_board_info));
	if (ret < 0)
		pr_err("archos_tvout_extdac_init: unable to register adv743x board info\n");

	disp_data->displays[disp_data->num_displays] = &extdac_panel;
	disp_data->num_displays++;

	return 0;
}
