/*
 * lcd_cpt-wvga48.c
 *
 *  Created on: Jan 13, 2009
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
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/display.h>
#include <linux/leds.h>

#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/omapfb.h>
#include <mach/display.h>


/*
 * Panel Instance
 */
struct panel {
	struct omap_display *disp;
	struct led_classdev lcd_backlight;
};
#define to_panel(obj) container_of(obj, struct panel, lcd_backlight)

#define MAX_BACKLIGHT	210	// pwm value for around 300mcd
#define DEFAULT_VCOM	128	// pwm value for 50% VCOM

#define CLAMP_BACKLIGHT(x) ( (x * MAX_BACKLIGHT) / 255)

static int lcd_backlight = MAX_BACKLIGHT;
static int led_classdev;

#ifdef CONFIG_DISPLAY_SUPPORT
/* 
 * Display device driver
 */
static struct display_device *display_dev;
static int lcd_vcom = DEFAULT_VCOM;

static int panel_get_contrast(struct display_device *disp)
{
	return 0;
}

static int panel_set_contrast(struct display_device *disp, unsigned int level)
{
	return 0;
}

static int panel_get_brightness(struct display_device *disp)
{
	return 0;
}

static int panel_set_brightness(struct display_device *disp, unsigned int level)
{
	return 0;
}

static int panel_get_vcom(struct display_device *disp)
{
	return lcd_vcom;
}

static int panel_set_vcom(struct display_device *ddev, unsigned int level)
{
	struct omap_display *disp = ddev->priv_data;
	
	lcd_vcom = level;	
	if (disp->hw_config.set_vcom)
		return disp->hw_config.set_vcom(disp, level);

	return 0;
}

static int panel_set_backlight(struct display_device *ddev, unsigned int level)
{
	struct omap_display *disp = ddev->priv_data;

	if (disp->hw_config.set_backlight)
		return disp->hw_config.set_backlight(disp, CLAMP_BACKLIGHT(level));
	return 0;
}

static int panel_get_backlight(struct display_device *ddev)
{
	return lcd_backlight;
}

static int panel_display_probe(struct display_device *disp, void *data)
{
	return 0;
}

static int panel_display_remove(struct display_device *disp)
{
	return 0;
}

static struct display_driver panel_display_driver = {
	.set_contrast = panel_set_contrast,
	.get_contrast = panel_get_contrast,
	.set_brightness = panel_set_brightness,
	.get_brightness = panel_get_brightness,
	.set_vcom = panel_set_vcom,
	.get_vcom = panel_get_vcom,
	.set_backlight = panel_set_backlight,
	.get_backlight = panel_get_backlight,
	.probe = panel_display_probe,
	.remove = panel_display_remove,
	.max_contrast = 0x3f,
	.max_brightness = 0x3f,
	.max_vcom = 0x3f,
	.max_backlight = 255,
};

/* 
 * END Display device driver
 */
#endif /* CONFIG_DISPLAY_SUPPORT */

#ifdef CONFIG_LEDS_CLASS
static void lcd_backlight_set(struct led_classdev *led_cdev, 
		enum led_brightness val)
{
	struct panel *obj = to_panel(led_cdev);
	struct omap_display *disp = obj->disp;

	lcd_backlight = CLAMP_BACKLIGHT((int)val);
	
	if (disp->hw_config.set_backlight)
		disp->hw_config.set_backlight(disp, lcd_backlight);
	led_cdev->brightness = lcd_backlight;
}

static enum led_brightness lcd_backlight_get(struct led_classdev *lcd_dev)
{
	return (enum led_brightness)lcd_backlight;
}

static struct panel panel_instance = {
	.lcd_backlight = {
		.name = "lcd-backlight",
		.brightness_set = lcd_backlight_set,
		.brightness_get = lcd_backlight_get,
	},
};
#endif

static int panel_init(struct omap_display *disp)
{	
#ifdef CONFIG_DISPLAY_SUPPORT
	if (!display_dev) {
		display_dev = display_device_register(&panel_display_driver, disp->dev, disp);
		if (IS_ERR(display_dev))
			dev_dbg(disp->dev, "cannot register panel as display driver\n");
	}
#endif
#ifdef CONFIG_LEDS_CLASS
	if (!led_classdev) {
		panel_instance.disp = disp;
		led_classdev_register(disp->dev, &panel_instance.lcd_backlight);
		led_classdev = 1;
	}
#endif
	return 0;
}

static void panel_cleanup(struct omap_display *disp)
{
#ifdef CONFIG_DISPLAY_SUPPORT
	if (display_dev) {
		display_device_unregister(display_dev);
		display_dev = NULL;
	}
#endif
#ifdef CONFIG_LEDS_CLASS
	if (led_classdev) {
		led_classdev_unregister(&panel_instance.lcd_backlight);
		led_classdev = 0;
	}
#endif
}

static int panel_enable(struct omap_display *disp)
{
	int ret = 0;

	if (disp->hw_config.panel_enable)
		ret = disp->hw_config.panel_enable(disp);
	if (ret < 0)
		return ret;

	if (disp->hw_config.set_vcom)
		disp->hw_config.set_vcom(disp, lcd_vcom);
	if (disp->hw_config.set_backlight)
		ret = disp->hw_config.set_backlight(disp, lcd_backlight);

	return ret;
}

static void panel_disable(struct omap_display *disp)
{
	if (disp->hw_config.set_backlight)
		disp->hw_config.set_backlight(disp, 0);
	if (disp->hw_config.set_vcom)
		disp->hw_config.set_vcom(disp, 0);

	if (disp->hw_config.panel_disable)
		disp->hw_config.panel_disable(disp);
}

static int panel_suspend(struct omap_display *disp)
{
	panel_disable(disp);
	return 0;
}

static int panel_resume(struct omap_display *disp)
{
	panel_enable(disp);
	return 0;
}

#define SAMSUNG_LCD_PIXCLOCK	27027	/* 27 MHz */

static struct omap_panel cpt_panel = {
	.owner		= THIS_MODULE,
	.name		= "cpt_wvga_48",
	.init		= panel_init,
	.cleanup	= panel_cleanup,
	.enable		= panel_enable,
	.disable	= panel_disable,
	.suspend	= panel_suspend,
	.resume		= panel_resume,

	.timings = {
		.x_res		= 800,
		.y_res		= 480,

		.pixel_clock	= SAMSUNG_LCD_PIXCLOCK,	/* 27.027Mhz */
		.hsw		= 2,		/* horizontal sync pulse width */
		.hfp		= 22,		/* horizontal front porch */
		.hbp		= 212,		/* horizontal back porch */
		.vsw		= 2,		/* vertical sync pulse width */
		.vfp		= 7,		/* vertical front porch */
		.vbp		= 32,		/* vertical back porch */
	},

	.config		= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF,

	.bpp = 24,
};

static int __init panel_drv_init(void)
{
	omap_dss_register_panel(&cpt_panel);
	return 0;
}

static void __exit panel_drv_exit(void)
{
	omap_dss_unregister_panel(&cpt_panel);
}

module_init(panel_drv_init);
module_exit(panel_drv_exit);
