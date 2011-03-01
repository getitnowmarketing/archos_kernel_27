/*
 * hx5091_spi.c
 *
 * Copyright (C) 2008 Archos
 *
 * spi interface for TFT LCD chip HIMAX HX5091-A 
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>

#define SPI_HX5091_ADDR	1

struct hx5091_spi_t {
	struct			spi_device *spi;
	struct			spi_driver driver;
	struct 			spi_transfer t[2];
	struct 			spi_message msg;
	unsigned short dummy;
};

static struct hx5091_spi_t *hx5091_spi;

int hx5091_spi_reg_write( unsigned int addr, unsigned int value) 
{
	unsigned char data[2];
	pr_debug("spi_reg_write: addr=%d, val=%02X\n", addr, value);

	/* Now we prepare the command for transferring */
	data[0] = addr & 0x7f;
	data[1] = value;

	spi_message_init(&hx5091_spi->msg);

	memset(&hx5091_spi->t, 0, sizeof(hx5091_spi->t));
	hx5091_spi->t[0].tx_buf = data;
	hx5091_spi->t[0].rx_buf = NULL;
	hx5091_spi->t[0].len = 2;
	spi_message_add_tail(&hx5091_spi->t[0], &hx5091_spi->msg);
	pr_debug("send sync\n");
	spi_sync( hx5091_spi->spi, &hx5091_spi->msg);

	return 0;
}

int hx5091_spi_reg_read( unsigned int addr, unsigned int *value) 
{

	unsigned char data, cmd;

	cmd = addr | 0x80;	// set read bit

	memset( hx5091_spi->t, 0, sizeof(hx5091_spi->t));
	spi_message_init(&hx5091_spi->msg);

	hx5091_spi->t[0].tx_buf = &cmd;
	hx5091_spi->t[0].rx_buf = &data;
	hx5091_spi->t[0].len = 2;
	spi_message_add_tail( &hx5091_spi->t[0], &hx5091_spi->msg);
	spi_sync( hx5091_spi->spi, &hx5091_spi->msg);

	pr_debug("spi_reg_read: addr=%d, val=%02X\n", addr, data);
	*value = data;	
	return 0;

}


int tpo_spi_reg_write( unsigned int addr, unsigned int value) 
{
	unsigned char data[2];
	
	pr_debug("spitpo_reg_write: addr=%d, val=%02X\n", addr, value);

	/* Now we prepare the command for transferring */
	data[0] = (addr << 1 ) & 0xfe;
	data[1] = value;

	spi_message_init(&hx5091_spi->msg);

	memset(&hx5091_spi->t, 0, sizeof(hx5091_spi->t));
	hx5091_spi->t[0].tx_buf = data;
	hx5091_spi->t[0].rx_buf = NULL;
	hx5091_spi->t[0].len = 2;
	spi_message_add_tail(&hx5091_spi->t[0], &hx5091_spi->msg);
	pr_debug("send sync\n");
	spi_sync( hx5091_spi->spi, &hx5091_spi->msg);

	return 0;
}


int tpo_spi_reg_read( unsigned int addr, unsigned int *value) 
{

	unsigned char data, cmd;

	cmd = (addr << 1) | 0x01;	// set read bit

	memset( hx5091_spi->t, 0, sizeof(hx5091_spi->t));
	spi_message_init(&hx5091_spi->msg);

	hx5091_spi->t[0].tx_buf = &cmd;
	hx5091_spi->t[0].rx_buf = &data;
	hx5091_spi->t[0].len = 2;
	spi_message_add_tail( &hx5091_spi->t[0], &hx5091_spi->msg);
	spi_sync( hx5091_spi->spi, &hx5091_spi->msg);

	pr_debug("spi_reg_read: addr=%d, val=%02X\n", addr, data);
	*value = data;	
	return 0;

}

static int  __devinit hx5091_probe( struct spi_device *spi )
{

	int r;

	pr_debug("hx5091_probe bus %x\n", spi->master->bus_num);

	hx5091_spi = kzalloc( sizeof(*hx5091_spi), GFP_KERNEL );
	if (hx5091_spi == NULL) {
		dev_err(&spi->dev, "out of mem\n");
		return -ENOMEM;
	}

	spi_set_drvdata( spi, &hx5091_spi);
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
static int hx5091_suspend(struct spi_device *spi, pm_message_t message)
{
	return 0;
}

static int hx5091_resume(struct spi_device *spi)
{
	return 0;
}

static int hx5091_remove(struct spi_device *spi)
{
	hx5091_spi = NULL;
	return 0;
}

static struct spi_driver hx5091_driver = {
	.driver = {
		.name	= "hx5091",
		.owner	= THIS_MODULE,
	},
	.probe =	hx5091_probe,
	.remove =	__devexit_p(hx5091_remove),
	.suspend	= hx5091_suspend,
	.resume		= hx5091_resume,
};

static int __init hx5091_spi_init(void)
{
        int err;

        pr_debug("hx5091_spi_init\n");
        err = spi_register_driver( &hx5091_driver );
        if (err < 0) {
                pr_err("Failed to register HX5091 client.\n");
		return err;
        }

	return 0;
}

static void __exit hx5091_spi_exit(void)
{
        spi_unregister_driver(&hx5091_driver);
}

module_init(hx5091_spi_init);
module_exit(hx5091_spi_exit);

EXPORT_SYMBOL(hx5091_spi_reg_read);
EXPORT_SYMBOL(hx5091_spi_reg_write);
EXPORT_SYMBOL(tpo_spi_reg_read);
EXPORT_SYMBOL(tpo_spi_reg_write);

MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("HX5091 TFT LCD Driver");
MODULE_LICENSE("GPL");
