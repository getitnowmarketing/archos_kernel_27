/*
 * linux/arch/arm/mach-omap2/board-apollon-keys.c
 *
 * Copyright (C) 2007 Samsung Electronics
 * Author: Kyungmin Park <kyungmin.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <mach/archos-gpio.h>
#include <mach/board-archos.h>

#include <mach/gpio.h>
#include <mach/mux.h>

static struct gpio_keys_button gpio_keys_buttons_list[] = {
	[0] = {
		.code		= KEY_VOLUMEUP,
		.gpio		= 0,
		.desc		= "vol up sw",
		.type		= EV_KEY,
		.active_low	= 1,
		.wakeup		= 0,
	},
	[1] = {
		.code		= KEY_VOLUMEDOWN,
		.gpio		= 0,
		.desc		= "vol down sw",
		.type		= EV_KEY,
		.active_low	= 1,
		.wakeup		= 0,
	},
	[2] = {
		.code		= KEY_POWER,
		.gpio		= 0,
		.desc		= "power sw",
		.type		= EV_KEY,
		.active_low	= 0,
		.wakeup		= 0,
	},
	[3] = {
		.code		= KEY_PROG1,
		.gpio		= 0,
		.desc		= "cam first sw",
		.type		= EV_KEY,
		.active_low	= 1,
		.wakeup		= 0,
	},
	[4] = {
		.code		= KEY_CAMERA,
		.gpio		= 0,
		.desc		= "cam full sw",
		.type		= EV_KEY,
		.active_low	= 1,
		.wakeup		= 0,
	},
};

static struct gpio_keys_button gpio_keys_buttons[5];

static struct gpio_keys_platform_data gpio_keys = {
	.buttons		= gpio_keys_buttons,
	.nbuttons		= 0,
	.rep			= 1,
};

static struct platform_device gpio_keys_device = {
	.name			= "gpio-keys",
	.id			= -1,
	.dev			= {
		.platform_data	= &gpio_keys,
	},
};

static int core_dump_codes[] = {KEY_VOLUMEUP, KEY_VOLUMEDOWN };

static struct archos_core_platform_data core_dump_data = {
	.gpio_keys		= &gpio_keys,
	.codes			= core_dump_codes,
	.ncodes			= 2,
};

static struct platform_device core_dump_device = {
	.name			= "core-dump",
	.id			= -1,
	.dev			= {
		.platform_data	= &core_dump_data,
	},
};

static int __init _keys_config(void)
{
	const struct archos_keys_config *keys_cfg;
	
	keys_cfg = omap_get_config( ARCHOS_TAG_KEYS, struct archos_keys_config );
	if (keys_cfg == NULL) {
		printk(KERN_DEBUG "archosh_keys_init: no board configuration found\n");
		return -ENODEV;
	}

	if ( hardware_rev >= keys_cfg->nrev ) {
		printk(KERN_DEBUG "archos_keys_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, keys_cfg->nrev);
		return 0;
	}

	/* Vol Up SW */
	omap_cfg_reg(GPIO_MUX(keys_cfg->rev[hardware_rev].vol_up));
	memcpy(&gpio_keys_buttons[gpio_keys.nbuttons], &gpio_keys_buttons_list[0], sizeof(struct gpio_keys_button));
	gpio_keys_buttons[gpio_keys.nbuttons].gpio = GPIO_PIN( keys_cfg->rev[hardware_rev].vol_up );
	gpio_keys.nbuttons++;
	
	/* Vol Down SW */
	omap_cfg_reg(GPIO_MUX(keys_cfg->rev[hardware_rev].vol_down));
	memcpy(&gpio_keys_buttons[gpio_keys.nbuttons], &gpio_keys_buttons_list[1], sizeof(struct gpio_keys_button));
	gpio_keys_buttons[gpio_keys.nbuttons].gpio = GPIO_PIN( keys_cfg->rev[hardware_rev].vol_down );
	gpio_keys.nbuttons++;

	if (GPIO_EXISTS( keys_cfg->rev[hardware_rev].power )) {
		omap_cfg_reg(GPIO_MUX(keys_cfg->rev[hardware_rev].power));
		memcpy(&gpio_keys_buttons[gpio_keys.nbuttons], &gpio_keys_buttons_list[2], sizeof(struct gpio_keys_button));
		gpio_keys_buttons[gpio_keys.nbuttons].gpio = GPIO_PIN( keys_cfg->rev[hardware_rev].power );
		gpio_keys.nbuttons++;
	}

	if (GPIO_EXISTS( keys_cfg->rev[hardware_rev].cam_first )) {
		omap_cfg_reg(GPIO_MUX(keys_cfg->rev[hardware_rev].cam_first));
		memcpy(&gpio_keys_buttons[gpio_keys.nbuttons], &gpio_keys_buttons_list[3], sizeof(struct gpio_keys_button));
		gpio_keys_buttons[gpio_keys.nbuttons].gpio = GPIO_PIN( keys_cfg->rev[hardware_rev].cam_first );
		gpio_keys.nbuttons++;
	}

	if (GPIO_EXISTS( keys_cfg->rev[hardware_rev].cam_full )) {
		omap_cfg_reg(GPIO_MUX(keys_cfg->rev[hardware_rev].cam_full));
		memcpy(&gpio_keys_buttons[gpio_keys.nbuttons], &gpio_keys_buttons_list[4], sizeof(struct gpio_keys_button));
		gpio_keys_buttons[gpio_keys.nbuttons].gpio = GPIO_PIN( keys_cfg->rev[hardware_rev].cam_full );
		gpio_keys.nbuttons++;
	}

	return 0;
}

int __init archos_keys_init(void)
{
	int ret;
	if ((ret = _keys_config()) < 0)
		return ret;

	ret = platform_device_register(&gpio_keys_device);
	if (ret < 0)
		return ret;

	return platform_device_register(&core_dump_device);
}
