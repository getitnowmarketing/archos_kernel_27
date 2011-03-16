/*
 * archos-tvout-venc.c
 *
 *  Created on: Feb 9, 2009
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>

#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/omapfb.h>
#include <mach/display.h>
#include <mach/archos-gpio.h>
#include <mach/board-archos.h>

#ifdef CONFIG_FB_OMAP2
static int panel_enable_tv(struct omap_display *display)
{
	return 0;
}

static void panel_disable_tv(struct omap_display *display)
{
}

static struct omap_display_data board_display_venc = {
	.type = OMAP_DISPLAY_TYPE_VENC,
	.name = "venc",
	.panel_name = "omap3_video_encoder",
	.u.venc.type = OMAP_DSS_VENC_TYPE_COMPOSITE,
	.u.venc.norm = OMAP_DSS_VENC_NORM_PAL,
	.panel_enable = panel_enable_tv,
	.panel_disable = panel_disable_tv,
};

static struct resource omap_vout_resource[3 - CONFIG_FB_OMAP2_NUM_FBS] = {
};

static struct platform_device omap_vout_device = {
	.name			= "omap_vout",
	.num_resources	= ARRAY_SIZE(omap_vout_resource),
	.resource 		= &omap_vout_resource[0],
	.id		= -1,
};

int __init archos_tvout_venc_init(struct omap_dss_platform_data *disp_data)
{
	int ret;

	ret = platform_device_register(&omap_vout_device);
	if (ret)
		return ret;

	disp_data->displays[disp_data->num_displays] = &board_display_venc;
	disp_data->num_displays++;
	return 0;
}
#else
int __init archos_tvout_venc_init(struct omap_dss_platform_data *disp_data)
{
	return 0;
}
#endif
