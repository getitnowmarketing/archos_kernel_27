/*
 * lcd_tpo.c
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

#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/omapfb.h>
#include <mach/display.h>

#include "tpo_lmj048.h"

/*
 * Panel SPI driver
 */
static struct tpo_spi_t {
	struct spi_device	*spi;
	struct spi_driver	driver;
	struct spi_transfer	t[2];
	struct spi_message	msg;
	unsigned short 	dummy;
} *tpo_48_spi;

struct tpo_48_regfile {
	int reg;
	int val;
};
// configuration table from TPO
// report 26 /05 /2008
#define TPO_REG_TO_INIT	27
static struct tpo_48_regfile TPOWVGA48_init[TPO_REG_TO_INIT] = {
	{0x03, 0x86},	// stdby by software, normal mode
	{0x04, 0x0f},	// default 0x0f 
	{0x05, 0x33},	// default
	{0x09, 0xff},	// default 
	{0x3a, 0x99},	// vcom
	{0x3c, 0xe0},	// default
	{0x3d, 0xf4},	// default
	{0x3e, 0x21},
	{0x3f, 0x87},	// default
	{0x15, 0x55},	// default
	{0x16, 0xaf},
	{0x17, 0xfc},
	{0x18, 0x77},
	{0x19, 0xd3},
	{0x1a, 0xdf},
	{0x1b, 0xf0},	// default
	{0x1c, 0x10},	// default
	{0x1d, 0x45},
	{0x1e, 0x7b},
	{0x1f, 0xea},	//  default
	{0x20, 0x8d},	// gain default
	{0x21, 0xf0},	// gain default
	{0x22, 0x26},	// gain default
	{0x23, 0x53},	// offset default
	{0x24, 0x7c},	// offset default
	{0x25, 0xbb},	// offset default
	{0x26, 0xff},	// offset default

};

static int tpo_spi_reg_write( unsigned int addr, unsigned int value) 
{
	unsigned char data[2];
	
	pr_debug("spi_reg_write: addr=%d, val=%02X\n", addr, value);

	/* Now we prepare the command for transferring */
	data[0] = (addr << 1 ) & 0xfe;
	data[1] = value;

	spi_message_init(&tpo_48_spi->msg);

	memset(&tpo_48_spi->t, 0, sizeof(tpo_48_spi->t));
	tpo_48_spi->t[0].tx_buf = data;
	tpo_48_spi->t[0].rx_buf = NULL;
	tpo_48_spi->t[0].len = 2;
	spi_message_add_tail(&tpo_48_spi->t[0], &tpo_48_spi->msg);
	spi_sync( tpo_48_spi->spi, &tpo_48_spi->msg);

	return 0;
}

static int __devinit tpo_48_probe( struct spi_device *spi )
{
	int r;

	pr_debug("tpo_48_probe bus %x\n", spi->master->bus_num);

	tpo_48_spi = kzalloc( sizeof(*tpo_48_spi), GFP_KERNEL );
	if (tpo_48_spi == NULL) {
		dev_err(&spi->dev, "out of mem\n");
		return -ENOMEM;
	}

	spi_set_drvdata( spi, tpo_48_spi);
	tpo_48_spi->spi = spi;

	tpo_48_spi->spi->mode = SPI_MODE_3;
	tpo_48_spi->spi->bits_per_word = 8;
	if ((r = spi_setup( tpo_48_spi->spi )) < 0) {
		dev_err(&spi->dev, "SPI setup failed\n");
		goto err;
	}

	dev_info(&spi->dev, "initialized\n");

	return 0;
err:
	kfree( tpo_48_spi );
	return r;
}

static int tpo_48_remove(struct spi_device *spi)
{
	kfree( tpo_48_spi );
	return 0;
}


static struct spi_driver tpo_48_driver = {
	.driver = {
		.name	= "tpo_48",
		.owner	= THIS_MODULE,
	},
	.probe =	tpo_48_probe,
	.remove =	__devexit_p(tpo_48_remove),
};

/*
 * END Panel SPI driver
 */

static int lcd_backlight = 200;

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

	if ( level < 10 )
		level = 10;
	if ( level > 0x3f )
		level = 0x3f;

	lcd_contrast = level;
	
	ret = tpo_spi_reg_write( TPO_RGAIN, level);
	ret = tpo_spi_reg_write( TPO_GGAIN, level);
	ret = tpo_spi_reg_write( TPO_BGAIN, level);

	return ret;
}

static int panel_get_brightness(struct display_device *disp)
{
	return lcd_brightness;
}

static int panel_set_brightness(struct display_device *disp, unsigned int level)
{
	int ret;

	if ( level < 5 )
		level = 5;
	if ( level > 0x3f )
		level = 0x3f;

	ret = tpo_spi_reg_write( TPO_ROFFSET, level);
	ret = tpo_spi_reg_write( TPO_GOFFSET, level);
	ret = tpo_spi_reg_write( TPO_BOFFSET, level);

	return ret;
}

static int panel_get_vcom(struct display_device *disp)
{
	return lcd_vcom;
}

static int panel_set_vcom(struct display_device *disp, unsigned int level)
{
	int ret;

	if ( level < 0 )
		level = 0;
	if ( level > 0x3f )
		level = 0x3f;

	level |= 0x80;	// vcom capability default

	ret = tpo_spi_reg_write( TPO_VCOM, level);
	
	return ret;
}

static int panel_set_backlight(struct display_device *ddev, unsigned int level)
{
	struct omap_display *disp = ddev->priv_data;
	
	if (disp->hw_config.set_backlight)
		return disp->hw_config.set_backlight(disp, level);
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

static int panel_init(struct omap_display *disp)
{
#ifdef CONFIG_DISPLAY_SUPPORT
	if (!display_dev) {
		display_dev = display_device_register(&panel_display_driver, disp->dev, disp);
		if (IS_ERR(display_dev))
			dev_dbg(disp->dev, "cannot register panel as display driver\n");
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
	for ( i = 0; i < TPO_REG_TO_INIT; i++ ) {
		ret = tpo_spi_reg_write( TPOWVGA48_init[i].reg, 
				TPOWVGA48_init[i].val);
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

#define TPO_LCD_PIXCLOCK	27027	/* 27 MHz */

static struct omap_panel tpo_panel = {
	.owner		= THIS_MODULE,
	.name		= "tpo_wvga_48",
	.init		= panel_init,
	.cleanup	= panel_cleanup,
	.enable		= panel_enable,
	.disable	= panel_disable,
//	.suspend	= panel_suspend,
//	.resume		= panel_resume,

	.timings = {
		.x_res		= 800,
		.y_res		= 480,

		.pixel_clock	= TPO_LCD_PIXCLOCK,	/* 27.027Mhz */
		.hsw		= 3,		/* horizontal sync pulse width */
		.hfp		= 43,		/* horizontal front porch */
		.hbp		= 213,		/* horizontal back porch */
		.vsw		= 3,		/* vertical sync pulse width */
		.vfp		= 11,		/* vertical front porch */
		.vbp		= 18,		/* vertical back porch */
	},

	.config		= OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF,

	.bpp = 24,
};

static int __init panel_drv_init(void)
{
        int err;

        err = spi_register_driver( &tpo_48_driver );
        if (err < 0) {
                pr_err("Failed to register tpo i2c client.\n");
		return -1;
	}

	omap_dss_register_panel(&tpo_panel);
	return 0;
}

static void __exit panel_drv_exit(void)
{
	omap_dss_unregister_panel(&tpo_panel);
	spi_unregister_driver(&tpo_48_driver);
}

module_init(panel_drv_init);
module_exit(panel_drv_exit);
