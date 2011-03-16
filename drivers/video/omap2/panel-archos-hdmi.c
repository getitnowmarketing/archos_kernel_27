/*
 * panel-hdmi.c
 *
 *  Created on: Feb 12, 2009
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 *     
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/display.h>

#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/omapfb.h>
#include <mach/display.h>

static int hdmi_panel_init(struct omap_display *display)
{
	return 0;
}

static int hdmi_panel_enable(struct omap_display *display)
{
	int r = 0;

	if (display->hw_config.panel_enable)
		r = display->hw_config.panel_enable(display);

	return r;
}

static void hdmi_panel_disable(struct omap_display *display)
{
	if (display->hw_config.panel_disable)
		display->hw_config.panel_disable(display);
}

static int hdmi_panel_suspend(struct omap_display *display)
{
	hdmi_panel_disable(display);
	return 0;
}

static int hdmi_panel_resume(struct omap_display *display)
{
	return hdmi_panel_enable(display);
}

static struct omap_panel hdmi_panel = {
	.owner		= THIS_MODULE,
	.name		= "panel-hdmi",
	.init		= hdmi_panel_init,
	.enable		= hdmi_panel_enable,
	.disable	= hdmi_panel_disable,
	.suspend	= hdmi_panel_suspend,
	.resume		= hdmi_panel_resume,

	.timings = {
#if defined(CONFIG_RES_576P)
		/* timing for 576p */
		.x_res          = 720,
		.y_res		= 576,
		.pixel_clock 	= 27027,
		.hfp		= 12,
		.hsw		= 32,
		.hbp		= 100,
		.vfp		= 4,
		.vsw		= 5,
		.vbp		= 40,
#elif defined(CONFIG_RES_480P)
		/* timing for 480p */
		.x_res		= 720,
		.y_res		= 480,
		.pixel_clock 	= 27027,
		.hfp		= 92,
		.hsw		= 30,
		.hbp		= 16,
		.vfp		= 8,
		.vsw		= 6,
		.vbp		= 31,
#elif defined(CONFIG_RES_720P)
		/* timing for 720P */
		.x_res		= 1280,
		.y_res		= 720,
		.pixel_clock	= 74250,
		.hfp		= 110,
		.hsw		= 8,
		.hbp		= 252,
		.vfp		= 21,
		.vsw		= 5,
		.vbp		= 4,
#endif
	},

	.bpp		= 24,
#ifndef CONFIG_RES_720P
	.config		= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
			OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF,
#else
	.config		= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF,
#endif
};


static int __init hdmi_panel_drv_init(void)
{
	omap_dss_register_panel(&hdmi_panel);
	return 0;
}

static void __exit hdmi_panel_drv_exit(void)
{
	omap_dss_unregister_panel(&hdmi_panel);
}

module_init(hdmi_panel_drv_init);
module_exit(hdmi_panel_drv_exit);
MODULE_LICENSE("GPL");
