/* linux/drivers/spi/spi_34xx_gpio.c
 *
 * Copyright (c) 2008 ARCHOS
 *
 * 34XX GPIO based SPI driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <mach/gpio.h>
#include <mach/spi-gpio.h>

static inline struct spigpio_info *spidev_to_info(struct spi_device *spi)
{
	return spi->controller_data;
}

static inline void setsck(struct spi_device *dev, int on)
{
	struct spigpio_info *info = spidev_to_info(dev);
	omap_set_gpio_dataout(info->pin_clk, on ? 1 : 0);
}

static inline void setmosi(struct spi_device *dev, int on)
{
	struct spigpio_info *info = spidev_to_info(dev);
	omap_set_gpio_dataout(info->pin_mosi, on ? 1 : 0);
}

static inline u32 getmiso(struct spi_device *dev)
{
	struct spigpio_info *info = spidev_to_info(dev);
	return omap_get_gpio_datain(info->pin_miso) ? 1 : 0;
}

static void omap3_spigpio_chipselect(struct spi_device *dev, int value)
{
	struct spigpio_info *info = spidev_to_info(dev);
	omap_set_gpio_dataout(info->pin_cs, value ? 0 : 1);
}

#define spidelay(x) ndelay(x)

#define	EXPAND_BITBANG_TXRX
#include <linux/spi/spi_bitbang.h>


static u32 omap3_spigpio_txrx_mode0(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 0, word, bits);
}

static u32 omap3_spigpio_txrx_mode1(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 0, word, bits);
}

static u32 omap3_spigpio_txrx_mode2(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha0(spi, nsecs, 1, word, bits);
}

static u32 omap3_spigpio_txrx_mode3(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	return bitbang_txrx_be_cpha1(spi, nsecs, 1, word, bits);
}

static int omap3_spigpio_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct spi_bitbang *sp;
	int ret;

	dev_dbg(&pdev->dev, "omap3_spigpio_probe %x\n", pdev->id);
	
	master = spi_alloc_master(&pdev->dev, sizeof(struct spi_bitbang));
	if (master == NULL) {
		dev_err(&pdev->dev, "failed to allocate spi master\n");
		ret = -ENOMEM;
		goto err;
	}

	sp = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, sp);

	master->bus_num = pdev->id;
	master->num_chipselect = 1;
	
	/* setup spi bitbang adaptor */
	sp->master = spi_master_get(master);
	sp->chipselect = omap3_spigpio_chipselect;
	sp->txrx_word[SPI_MODE_0] = omap3_spigpio_txrx_mode0;
	sp->txrx_word[SPI_MODE_1] = omap3_spigpio_txrx_mode1;
	sp->txrx_word[SPI_MODE_2] = omap3_spigpio_txrx_mode2;
	sp->txrx_word[SPI_MODE_3] = omap3_spigpio_txrx_mode3;

	ret = spi_bitbang_start(sp);
	if (ret < 0)
		goto err_no_bitbang;

	return 0;

 err_no_bitbang:
	spi_master_put(sp->master);
 err:
	return ret;

}

static int omap3_spigpio_remove(struct platform_device *dev)
{
	struct spi_bitbang *sp = platform_get_drvdata(dev);

	spi_bitbang_stop(sp);
	spi_master_put(sp->master);

	return 0;
}

/* all gpio should be held over suspend/resume, so we should
 * not need to deal with this
*/

#define omap3_spigpio_suspend NULL
#define omap3_spigpio_resume NULL


static struct platform_driver omap3_spigpio_drv = {
	.probe		= omap3_spigpio_probe,
        .remove		= omap3_spigpio_remove,
        .suspend	= omap3_spigpio_suspend,
        .resume		= omap3_spigpio_resume,
        .driver		= {
		.name	= "omap3-spi-gpio",
		.owner	= THIS_MODULE,
        },
};

static int __init omap3_spigpio_init(void)
{
        return platform_driver_register(&omap3_spigpio_drv);
}

static void __exit omap3_spigpio_exit(void)
{
        platform_driver_unregister(&omap3_spigpio_drv);
}

subsys_initcall(omap3_spigpio_init);
module_exit(omap3_spigpio_exit);

MODULE_DESCRIPTION("OMAP34XX SPI Driver");
MODULE_AUTHOR("ARCHOS");
MODULE_LICENSE("GPL");
