/*
 *   AD5820 driver
 *
 *   Copyright (c) by Jean-Christophe Rona <rona@archos.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/ad5820.h>
#include <mach/gpio.h>
#include <mach/mux.h>


#define AD5820_VERSION		"0.1"
#define AD5820_DATE		"16 July 2009"

/* Read operation will be cached if this is defined */
#define READ_CACHE

/* This define allows to configure/control the cam_reset GPIO.
	NOTE : this code should	be removed since the cam (mt9p012)
	driver will handle this GPIO */
#define GPIO_CODE

static int ad5820_read(void);
static int ad5820_write(u16 value);

struct ad5820_data {
//	struct ad5820_pdata *pdata;
	u16 cache;
};

/* I2C client handle */
struct i2c_client *this_client;

int gpio = 0;


/*
 * AD5820 stuff
 */

static int ad5820_set_power(short power)
{
	int ret = 0;
	u16 data;
	
	data = ad5820_read() & ~AD5820_POWER_MASK;
	ret = ad5820_write(data | (power & AD5820_POWER_MASK));
	return ret;
}

static int ad5820_get_power(void)
{
	u16 data;
	
	data = ad5820_read() & AD5820_POWER_MASK;
	return data;
}

static int ad5820_set_dac_value(short value)
{
	int ret = 0;
	u16 data;
	
	data = ad5820_read() & ~AD5820_DAC_MASK;
	ret = ad5820_write(data |
		((value << AD5820_DAC_SHIFT) & AD5820_DAC_MASK));
	return ret;
}

static int ad5820_get_dac_value(void)
{
	u16 data;
	
	data = ad5820_read() & AD5820_DAC_MASK;
	return (data >> AD5820_DAC_SHIFT);
}

static int ad5820_set_slewrate(short slewrate)
{
	int ret = 0;
	u16 data;
	
	data = ad5820_read() & ~AD5820_SLEW_RATE_MASK;
	ret = ad5820_write(data |
		((slewrate << AD5820_SLEW_RATE_SHIFT) & AD5820_SLEW_RATE_MASK));
	return ret;
}

static int ad5820_get_slewrate(void)
{
	u16 data;
	
	data = ad5820_read() & AD5820_SLEW_RATE_MASK;
	return (data >> AD5820_SLEW_RATE_SHIFT);
}


/*
 * SYSFS callback functions
 */

static ssize_t power_ctrl_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int power = ad5820_get_power();
	return sprintf(buf, "%d\n", power);
}

static ssize_t power_ctrl_store(struct device *dev, struct device_attribute *attr,
			    const char *buf,size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	if (val > 0x01)
		return -EINVAL;
	ad5820_set_power(val);
	return count;
}

static DEVICE_ATTR(power_ctrl, S_IWUSR | S_IRUGO, power_ctrl_show, power_ctrl_store);

static ssize_t dac_value_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int value = ad5820_get_dac_value();
	return sprintf(buf, "%d\n", value);
}

static ssize_t dac_value_store(struct device *dev, struct device_attribute *attr,
			    const char *buf,size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	if (val > 0x3FF)
		return -EINVAL;
	ad5820_set_dac_value(val);
	return count;
}

static DEVICE_ATTR(dac_value, S_IWUSR | S_IRUGO, dac_value_show, dac_value_store);

static ssize_t slewrate_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int slewrate = ad5820_get_slewrate();
	return sprintf(buf, "%d\n", slewrate);
}

static ssize_t slewrate_store(struct device *dev, struct device_attribute *attr,
			    const char *buf,size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	if (val > 0x07)
		return -EINVAL;
	ad5820_set_slewrate(val);
	return count;
}

static DEVICE_ATTR(slewrate, S_IWUSR | S_IRUGO, slewrate_show, slewrate_store);

#ifdef GPIO_CODE
static ssize_t reset_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int reset = gpio;
	return sprintf(buf, "%d\n", reset);
}

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			    const char *buf,size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	if (val > 0x01)
		return -EINVAL;
	gpio = val;
	omap_set_gpio_dataout(126, gpio);
	return count;
}

static DEVICE_ATTR(reset, S_IWUSR | S_IRUGO, reset_show, reset_store);
#endif


/*
 * I2C Stuff
 */

static int ad5820_read(void)
{
#ifndef READ_CACHE
	char buf[2] = {0};

	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = 2,
		 .buf = buf,
		 },
	};


	if (i2c_transfer(this_client->adapter, msgs, 1) < 0) {
		printk(KERN_ERR "ad5820_read: transfer error\n");
		return -EIO;
	} else
		return (buf[0] << 8) | buf[1];
#else
	struct ad5820_data *data = i2c_get_clientdata(this_client);
	return data->cache;
#endif
}

static int ad5820_write(u16 value)
{
#ifdef READ_CACHE
	struct ad5820_data *data = i2c_get_clientdata(this_client);
#endif
	char buf[2];

	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buf,
		 },
	};

	buf[0] = (value >> 8) & 0xFF;
	buf[1] = value & 0xFF;

#ifdef READ_CACHE
	data->cache = value;
#endif

	if (i2c_transfer(this_client->adapter, msg, 1) < 0) {
		printk(KERN_ERR "ad5820_write: transfer error\n");
		return -EIO;
	} else
		return 0;
}


static int ad5820_probe(struct i2c_client *client, 
		const struct i2c_device_id *id)
{
	int err;
	struct ad5820_data *i2c_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s functinality check failed\n", id->name);
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	i2c_data = kzalloc(sizeof(struct ad5820_data), GFP_KERNEL);
	if (!i2c_data) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

/*
	i2c_data->pdata = (struct ad5820_pdata*) client->dev.platform_data;
	if (!i2c_data->pdata) {
		printk(KERN_ERR "ad5820_probe: No platform data !!\n");
		err = -ENODEV;
		goto exit_plat_data_failed;
	}
*/

#ifdef READ_CACHE
	i2c_data->cache = 0x0000;
#endif

	i2c_set_clientdata(client, i2c_data);
	this_client = client;

#ifdef GPIO_CODE
	omap_cfg_reg(D25_3430_GPIO126);

	/* Request and configure gpio pins */
	if (omap_request_gpio(126) != 0)
		return -EIO;

	/* set to output mode */
	omap_set_gpio_direction(126, GPIO_DIR_OUTPUT);
	omap_set_gpio_dataout(126, gpio);
#endif

	err = device_create_file(&client->dev, &dev_attr_power_ctrl);
	if (err) {
		goto exit_create_power_failed;
	}

	err = device_create_file(&client->dev, &dev_attr_dac_value);
	if (err) {
		goto exit_create_dac_failed;
	}

	err = device_create_file(&client->dev, &dev_attr_slewrate);
	if (err) {
		goto exit_create_slewrate_failed;
	}

#ifdef GPIO_CODE
	err = device_create_file(&client->dev, &dev_attr_reset);
	if (err) {
		goto exit_create_reset_failed;
	}
#endif

	return 0;

#ifdef GPIO_CODE
exit_create_reset_failed:
	device_remove_file(&client->dev, &dev_attr_slewrate);
#endif
exit_create_slewrate_failed:
	device_remove_file(&client->dev, &dev_attr_dac_value);
exit_create_dac_failed:
	device_remove_file(&client->dev, &dev_attr_power_ctrl);
exit_create_power_failed:
	kfree(i2c_data);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
	
static int __exit ad5820_remove(struct i2c_client *client)
{
	struct ad5820_data *data = i2c_get_clientdata(client);

	i2c_detach_client(client);
	kfree(data);
	device_remove_file(&client->dev, &dev_attr_power_ctrl);
	device_remove_file(&client->dev, &dev_attr_dac_value);
	device_remove_file(&client->dev, &dev_attr_slewrate);
#ifdef GPIO_CODE
	device_remove_file(&client->dev, &dev_attr_reset);
#endif
	return 0;
}

static int ad5820_suspend(struct i2c_client *client, pm_message_t mesg)
{
//	struct ad5820_data *data = i2c_get_clientdata(client);

	return 0;
}

static int ad5820_resume(struct i2c_client *client)
{
//	struct ad5820_data *data = i2c_get_clientdata(client);

	return 0;
}


static const struct i2c_device_id ad5820_id[] = {
	{"ad5820", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ad5820_id);

static struct i2c_driver ad5820_driver = {

	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ad5820",
	},
	.probe		= ad5820_probe,
	.suspend	= ad5820_suspend,
	.resume		= ad5820_resume,
	.remove		= __exit_p(ad5820_remove),
	.id_table	= ad5820_id,
};

static int __init ad5820_init(void)
{
	int res;
	
	if ((res = i2c_add_driver(&ad5820_driver))) {
		printk("ad5820: Driver registration failed, module not inserted.\n");
		return res;
	}

	printk("AD5820 driver version %s (%s)\n", AD5820_VERSION, AD5820_DATE);
	
	return 0;
}

static void __exit ad5820_exit(void)
{
	i2c_del_driver(&ad5820_driver);
}

MODULE_AUTHOR("Jean-Christophe Rona <rona@archos.com>");
MODULE_DESCRIPTION("AD5820 driver");
MODULE_LICENSE("GPL");

module_init(ad5820_init)
module_exit(ad5820_exit)
