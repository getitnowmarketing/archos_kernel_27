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
#include <asm/mach-types.h>

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
	data[0] = addr & 0x7f;
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

#ifdef CONFIG_DISPLAY_SUPPORT
/* 
 * Display device driver
 */
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

	if ( level < 5 )
		level = 5;
	if ( level > 0x3f )
		level = 0x3f;

	lcd_contrast = level;
	
	ret = tpo_spi_reg_write( TPO_ROFFSET, level);
	ret = tpo_spi_reg_write( TPO_GOFFSET, level);
	ret = tpo_spi_reg_write( TPO_BOFFSET, level);

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
	.probe = panel_display_probe,
	.remove = panel_display_remove,
	.max_contrast = 0x3f,
	.max_brightness = 0x3f,
	.max_vcom = 0x3f,
};

/* 
 * END Display device driver
 */
#endif /* CONFIG_DISPLAY_SUPPORT */

static int (*platform_panel_init)(struct lcd_panel *panel, 
		struct omapfb_device *fbdev);
static int (*platform_panel_enable)(struct lcd_panel *panel);

static int local_panel_init(struct lcd_panel *panel,
		struct omapfb_device *fbdev)
{
	int ret;
	
	ret = platform_panel_init(panel, fbdev);
	if (ret < 0)
		return ret;
	
	return 0;
}

static int local_panel_enable(struct lcd_panel *panel)
{
	int ret;
	int i;

	ret = platform_panel_enable(panel);
	if (ret < 0)
		return ret;
	
	/* Init LCD controller with default configuration */
	for ( i = 0; i < TPO_REG_TO_INIT; i++ ) {
		ret = tpo_spi_reg_write( TPOWVGA48_init[i].reg, 
				TPOWVGA48_init[i].val);
		if (ret < 0)
			break;
	}
	
	return ret;
}

static int panel_probe(struct platform_device *pdev)
{
	struct lcd_panel *panel = pdev->dev.platform_data;
	
	/* intercept init and enable functions */
	platform_panel_init = panel->init;
	platform_panel_enable = panel->enable;
	panel->init = local_panel_init;
	panel->enable = local_panel_enable;
	
#ifdef CONFIG_DISPLAY_SUPPORT
	{ struct display_device *ddev;
	ddev = display_device_register(&panel_display_driver, &pdev->dev, NULL);
	if (IS_ERR(ddev))
		dev_dbg(&pdev->dev, "cannot register panel as display driver\n");
	}
#endif
	
	omapfb_register_panel(panel);
	return 0;
}

static int panel_remove(struct platform_device *pdev)
{
	return 0;
}

static int panel_suspend(struct platform_device *pdev,
				   pm_message_t mesg)
{
	return 0;
}

static int panel_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver panel_driver = {
	.probe		= panel_probe,
	.remove		= panel_remove,
	.suspend	= panel_suspend,
	.resume		= panel_resume,
	.driver		= {
		.name	= "tpo_wvga_48",
		.owner	= THIS_MODULE,
	},
};

static int __init panel_drv_init(void)
{
        int err;

        err = spi_register_driver( &tpo_48_driver );
        if (err < 0)
                pr_err("Failed to register TPO 48 client.\n");

	return platform_driver_register(&panel_driver);
}

static void __exit panel_drv_exit(void)
{
	platform_driver_unregister(&panel_driver);
        spi_unregister_driver(&tpo_48_driver);
}

module_init(panel_drv_init);
module_exit(panel_drv_exit);
