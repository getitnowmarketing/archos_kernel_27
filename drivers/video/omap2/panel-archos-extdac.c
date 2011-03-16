/*
 * panel-archos-extdac.c
 *
 *  Created on: Mar 5, 2009
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
#include <mach/archos-gpio.h>
#include <mach/board-archos.h>

#define EVEN_FIELD 	0
#define ODD_FIELD 	1
static int field;	/* 0: EVEN, 1: ODD */
static void* isr_handle;

static void extdac_frame_done_handler(void* arg, u32 mask)
{
	struct omap_display *display = arg;
	struct omap_overlay_manager *mgr = display->manager;
	struct archos_extdac_conf *cfg = display->hw_config.priv;
	struct omap_video_timings t;
	struct omap_overlay *ovl;
	int i;
	
	/* manager not attached yet */
	if (mgr == NULL)
		return;

	for (i = 0; i < mgr->num_overlays; i++) {
		ovl = &mgr->overlays[i];
		if (ovl->info.enabled) {
			u32 ovl_paddr = ovl->info.paddr;
			if (field == EVEN_FIELD)
				ovl_paddr += ovl->info.tv_field1_offset;
			omap_dispc_set_plane_ba0(ovl->id, ovl_paddr);
			omap_dispc_set_plane_ba1(ovl->id, ovl_paddr);
			/* TODO: configure overlay accu */
		}
	}
	
	/* setup vertical timing for new field */
	display->get_timings(display, &t);
	/* odd field has reduced back porch */
	if (field == ODD_FIELD)
		t.vbp -= 1;
	omap_dispc_set_lcd_timings(t.hsw, t.hfp, t.hbp, t.vsw, t.vfp, t.vbp);
	/* establish new values */
	omap_dispc_go(mgr->id);
	
	/* signal field info to CPLD */
	omap_set_gpio_dataout(GPIO_PIN(cfg->cpld_aux), field == EVEN_FIELD);
	
	/* toggle field for next interrupt */
	field ^= 1;
}

static int extdac_panel_init(struct omap_display *display)
{
	return 0;
}

static int extdac_panel_enable(struct omap_display *display)
{	
	struct archos_extdac_conf *cfg = display->hw_config.priv;
	int r = 0;
	
	field = 0;
	omap_set_gpio_dataout(GPIO_PIN(cfg->cpld_aux), field == EVEN_FIELD);

	omap_dispc_set_linenbr_irq(25);
	
	isr_handle = omap_dispc_register_isr(extdac_frame_done_handler, 
			display, DISPC_IRQ_PROG_LINE_NUM);
	if (isr_handle == NULL)
		return -EBUSY;
	
	if (display->hw_config.panel_enable)
		r = display->hw_config.panel_enable(display);

	return r;
}

static void extdac_panel_disable(struct omap_display *display)
{
	omap_dispc_unregister_isr(isr_handle);
	
	if (display->hw_config.panel_disable)
		display->hw_config.panel_disable(display);
}

static int extdac_panel_suspend(struct omap_display *display)
{
	extdac_panel_disable(display);
	return 0;
}

static int extdac_panel_resume(struct omap_display *display)
{
	return extdac_panel_enable(display);
}

static struct omap_panel extdac_panel = {
	.owner		= THIS_MODULE,
	.name		= "adv734x",
	.init		= extdac_panel_init,
	.enable		= extdac_panel_enable,
	.disable	= extdac_panel_disable,
	.suspend	= extdac_panel_suspend,
	.resume		= extdac_panel_resume,

	.timings = {
		/* timing for 576i (PAL) */
		.x_res          = 720,
		.y_res		= 576,
		.pixel_clock 	= 13500,
		.hfp		= 12,
		.hbp		= 100,
		.hsw		= 32,
		.vfp		= 1,
		.vbp		= 20,
		.vsw		= 4,
		.interlaced	= 1,
	},

	.bpp		= 24,
	.config		= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
			OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF,
};


static int __init extdac_panel_drv_init(void)
{
	omap_dss_register_panel(&extdac_panel);
	return 0;
}

static void __exit extdac_panel_drv_exit(void)
{
	omap_dss_unregister_panel(&extdac_panel);
}

module_init(extdac_panel_drv_init);
module_exit(extdac_panel_drv_exit);
MODULE_LICENSE("GPL");
