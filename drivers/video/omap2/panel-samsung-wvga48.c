/*
 * lcd_samsung-wvga48.c
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

#include "hx5091.h"

/*
 * Panel Instance
 */
struct panel {
	struct omap_display *disp;
	struct led_classdev lcd_backlight;
};
#define to_panel(obj) container_of(obj, struct panel, lcd_backlight)

/*
 * Panel SPI driver
 */
struct hx5091_spi_t {
	struct spi_device	*spi;
	struct spi_driver	driver;
	struct spi_transfer	t[2];
	struct spi_message	msg;
	unsigned short 	dummy;
};

static struct hx5091_spi_t *hx5091_spi;

struct hx5091_regfile {
	int reg;
	int val;
};
#define SMS_REG_TO_INIT 56

// new table 20/11/2008
// ref Samsung "Archos LMS480KC01 Vertical line Issue_081119.pdf"
static const struct hx5091_regfile 
	SMSWVGA48_init [SMS_REG_TO_INIT] = {
	{0x00, 0x63},	// stdby by software, normal mode
	{0x01, 0x55},	// default
	{0x02, 0x10},	// v posit archos value
	{0x03, 0x80},	// h posit archos value
	{0x04, 0x00},	// default
	{0x05, 0x11},	// default
	{0x06, 0x00},
	{0x07, 0x00},	// default
	{0x08, 0x40},	// default
	{0x09, 0x01},
	{0x0a, 0x20},
	{0x0b, 0x29},
	{0x0c, 0x10},
	{0x0d, 0x30},
	{0x0e, 0x20},	// default
	{0x0f, 0x20},	// default
	{0x10, 0xca},
	{0x11, 0xca},
	{0x12, 0x0b},	// vcom default
	{0x13, 0x20},	// gain default
	{0x14, 0x20},	// gain default
	{0x15, 0x20},	// gain default
	{0x16, 0x80},	// offset default
	{0x17, 0x80},	// offset default
	{0x18, 0x80},	// offset default
	{0x20, 0x00},
	{0x21, 0x03},
	{0x22, 0xef},
	{0x23, 0x2d},
	{0x24, 0x6b},
	{0x25, 0xa1},
	{0x26, 0xb9},
	{0x27, 0x6d},
	{0x28, 0xeb},
	{0x29, 0x51},
	{0x2a, 0x97},
	{0x2b, 0xff},
	{0x2c, 0x40},
	{0x2d, 0x95},
	{0x2e, 0xfe},
	{0x50, 0x00},
	{0x51, 0x03},
	{0x52, 0xef},
	{0x53, 0x2d},
	{0x54, 0x6b},
	{0x55, 0xa1},
	{0x56, 0xd8},
	{0x57, 0x6d},
	{0x58, 0xeb},
	{0x59, 0x51},
	{0x5a, 0x97},
	{0x5b, 0xff},
	{0x5c, 0x40},
	{0x5d, 0x95},
	{0x5e, 0xfe},
	{0x2f, 0x21},
};

static int hx5091_spi_reg_write( unsigned int addr, unsigned int value) 
{
	unsigned char data[2];
	
	//pr_debug("spi_reg_write: addr=%d, val=%02X\n", addr, value);

	/* Now we prepare the command for transferring */
	data[0] = addr & 0x7f;
	data[1] = value;

	spi_message_init(&hx5091_spi->msg);

	memset(&hx5091_spi->t, 0, sizeof(hx5091_spi->t));
	hx5091_spi->t[0].tx_buf = data;
	hx5091_spi->t[0].rx_buf = NULL;
	hx5091_spi->t[0].len = 2;
	spi_message_add_tail(&hx5091_spi->t[0], &hx5091_spi->msg);
	spi_sync( hx5091_spi->spi, &hx5091_spi->msg);

	return 0;
}

static int __devinit hx5091_probe( struct spi_device *spi )
{
	int r;

	pr_debug("hx5091_probe bus %x\n", spi->master->bus_num);

	hx5091_spi = kzalloc( sizeof(*hx5091_spi), GFP_KERNEL );
	if (hx5091_spi == NULL) {
		dev_err(&spi->dev, "out of mem\n");
		return -ENOMEM;
	}

	spi_set_drvdata( spi, hx5091_spi);
	hx5091_spi->spi = spi;

	hx5091_spi->spi->mode = SPI_MODE_3;
	hx5091_spi->spi->bits_per_word = 8;
	if ((r = spi_setup( hx5091_spi->spi )) < 0) {
		dev_err(&spi->dev, "SPI setup failed\n");
		goto err;
	}

	dev_info(&spi->dev, "initialized\n");

	return 0;
err:
	kfree( hx5091_spi );
	return r;
}

static int hx5091_remove(struct spi_device *spi)
{
	kfree( hx5091_spi );
	return 0;
}


static struct spi_driver hx5091_driver = {
	.driver = {
		.name	= "hx5091",
		.owner	= THIS_MODULE,
	},
	.probe =	hx5091_probe,
	.remove =	__devexit_p(hx5091_remove),
};

/*
 * END Panel SPI driver
 */

#define MAX_BACKLIGHT	180	// pwm value for around 300cd

#define CLAMP_BACKLIGHT(x) ( (x * MAX_BACKLIGHT) / 255)

static int lcd_backlight = MAX_BACKLIGHT;
static int led_classdev;

#ifdef CONFIG_DISPLAY_SUPPORT
/* 
 * Display device driver
 */
static struct display_device *display_dev;
static int lcd_contrast;
static int lcd_brightness;
static int lcd_vcom;

static int panel_get_contrast(struct display_device *disp)
{
	return lcd_contrast;
}

static int panel_set_contrast(struct display_device *disp, unsigned int level)
{
	int ret;

	if ( level < 15 )
		level = 15;
	if ( level > 0x3f )
		level = 0x3f;

	lcd_contrast = level;
	
	ret = hx5091_spi_reg_write( HX5091_RGAIN, level);
	ret = hx5091_spi_reg_write( HX5091_GGAIN, level);
	ret = hx5091_spi_reg_write( HX5091_BGAIN, level);
	
	return ret;
}

static int panel_get_brightness(struct display_device *disp)
{
	return lcd_brightness;
}

static int panel_set_brightness(struct display_device *disp, unsigned int level)
{
	int ret;

	level *=4;
	if ( level < 50 )
		level = 50;
	if ( level > 0xff )
		level = 0xff;

	ret = hx5091_spi_reg_write( HX5091_ROFFSET, level);
	ret = hx5091_spi_reg_write( HX5091_GOFFSET, level);
	ret = hx5091_spi_reg_write( HX5091_BOFFSET, level);

	return ret;
}

static int panel_get_vcom(struct display_device *disp)
{
	return lcd_vcom;
}

static int panel_set_vcom(struct display_device *disp, unsigned int level)
{
	int ret;

	level /=2;
	if ( level < 0 )
		level = 0;
	if ( level > 0x1f )
		level = 0x1f;

	ret = hx5091_spi_reg_write( HX5091_VCOM, level);
	
	return ret;
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
	int i;

	if (disp->hw_config.panel_enable)
		ret = disp->hw_config.panel_enable(disp);
	if (ret < 0)
		return ret;
	
	/* Init LCD controller with default configuration */
	for ( i = 0; i < SMS_REG_TO_INIT; i++ ) {
		ret = hx5091_spi_reg_write( SMSWVGA48_init[i].reg, 
				SMSWVGA48_init[i].val);
		if (ret < 0)
			break;
	}

	if (disp->hw_config.set_backlight)
		ret = disp->hw_config.set_backlight(disp, lcd_backlight);
	
	return ret;
}

static void panel_disable(struct omap_display *disp)
{
	if (disp->hw_config.set_backlight)
		disp->hw_config.set_backlight(disp, 0);
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

static struct omap_panel samsung_panel = {
	.owner		= THIS_MODULE,
	.name		= "samsung_wvga_48",
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
		.hsw		= 3,		/* horizontal sync pulse width */
		.hfp		= 23,		/* horizontal front porch */
		.hbp		= 213,		/* horizontal back porch */
		.vsw		= 3,		/* vertical sync pulse width */
		.vfp		= 7,		/* vertical front porch */
		.vbp		= 32,		/* vertical back porch */
	},

	.config		= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF,

	.bpp = 24,
};

static int __init panel_drv_init(void)
{
        int err;

        err = spi_register_driver( &hx5091_driver );
        if (err < 0)
                pr_err("Failed to register HX5091 client.\n");

	omap_dss_register_panel(&samsung_panel);
	return 0;
}

static void __exit panel_drv_exit(void)
{
	omap_dss_unregister_panel(&samsung_panel);
        spi_unregister_driver(&hx5091_driver);
}

module_init(panel_drv_init);
module_exit(panel_drv_exit);
