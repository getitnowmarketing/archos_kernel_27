/*
 *   Analog device I2c video encoder Interface.
 *
 *   Copyright (C) 2008 Archos SA
 *   Author: Archos
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
 *
 */
#define DEBUG
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/fb.h>

#include <mach/gpio.h>
#include <mach/mux.h>

#include "adv734x.h"

#define ADV734X_NB_RETRY_I2C	3
#define ADV734X_VERSION	"0.0"
#define ADV734X_DATE	"March-2008"

/* TODO: find header file to export these */
#define RESOLUTION_VIDEOEXT_PAL	0
#define RESOLUTION_VIDEOEXT_NTSC	1
#define RESOLUTION_VIDEOEXT_HDPAL	2
#define RESOLUTION_NB_VIDEOEXT		3

#define ANALOG_OUT_DEFAULT	0x0
#define ANALOG_OUT_CVBS	0x10
#define ANALOG_OUT_SVIDEO	0x20
#define ANALOG_OUT_RGB		0x30
#define ANALOG_OUT_COMPONENT	0x40

#define MAX_ADV_CONFIG_SIZE 50
#define CONFIG_STOP 0xff

struct adv734x_regval {
	int reg;
	int val;
};

static struct adv734x_regval _pal_config[]={
	{0x17,0x2},
	{0x80,0x11},
	{0x82,0x40},
	{0x87,0x80},
	{0x88,0x00},
// 	{0x8a,0x0c},
 	{0x8a,0x0a},
	{0x8c,0xcb},
	{0x8d,0x8a},
	{0x8e,0x09},
	{0x8f,0x2a},
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _ntsc_config[]={
	{0x17,0x2},
	{0x80,0x10},
	{0x82,0x40},
	{0x87,0x80},
	{0x88,0x00},
// 	{0x8a,0x0c},
 	{0x8a,0x0a},  /* mode 1 slave */
 	{0x8c,0x1f},
	{0x8d,0x7c},
	{0x8e,0xf0},
	{0x8f,0x21},
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _hdpal_config[]={
	{0x17,0x2},
	{0x01,0x10},
	{0x30,0x28}, /* format 720p 50 Hz */
	{0x31,0x01}, /* pixel data valid */
	{0x33,0x28}, /*  */
	{0x34,0x40},
// 	{0x35,0x02}, /* RGB input */
 	{0x35,0x00}, /* YPrPb input */
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _composite_config[]={
	{0x00,0x00}, /* all dac off */
	{0x02,0x00}, /* rgb out */	
	{0x00,(PLL_VAL|COMPOSITE_DAC)}, /* dac on*/
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _component_config[]={
	{0x00,0x00}, /* all dac off */
	{0x02,0x20}, /* YPrPb out */	
	{0x00,(PLL_VAL|COMPONENT_DAC)}, /* dac on*/
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _rgb_config[]={
	{0x00,0x00}, /* all dac off */
	{0x02,0x00}, /* rgb out */	
	{0x00,(PLL_VAL|RGB_DAC)}, /* dac on*/
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _svideo_config[]={
	{0x00,0x00}, /* all dac off */
	{0x02,0x00}, /* rgb out */	
	{0x00,(PLL_VAL|SVIDEO_DAC)}, /* dac on*/
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _dump_config[]={
	{0x00,0},
	{0x02,0},
	{0x17,0},
	{0x80,0},
	{0x82,0},
 	{0x84,0},
	{0x87,0},
	{0x88,0},
	{0x8a,0},
	{0x8c,0},
	{0x8d,0},
	{0x8e,0},
	{0x8f,0},
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static struct adv734x_regval _sleepon_config[]={
	{0x17,0x2},
	{0x00,0x01}, /* set on sleep mode */
	{CONFIG_STOP,CONFIG_STOP}, /* end of config */
};

static inline int adv734x_read_value(struct i2c_client *client, int reg, int *value)
{
	unsigned int data;
	int cnt = -1;
	
	do{
		cnt++;
		*value = 0;
		data = i2c_smbus_read_byte_data( client, reg );
	} while((cnt != ADV734X_NB_RETRY_I2C) && (data == -1));
	if((cnt == ADV734X_NB_RETRY_I2C) || (data == -1)){
		pr_debug( "adv734x bad read [%#x]=%#x(%d)\n",reg,data,data);
		*value = 0;
		return -1;
	} else {
		*value = data;
		pr_debug( "adv734x_read_value[%#x]=%#x(%d)\n",reg,data,data);
		return 0;
	}
}

static inline int adv734x_write_value(struct i2c_client *client, int reg, int value)
{
	int cnt = 0;
	int ret = 0;
	
	do {
		cnt++;
		ret = i2c_smbus_write_byte_data(client, reg, value);
	} while((cnt != ADV734X_NB_RETRY_I2C) && (ret == -1));
	
	if(ret != -1)
		pr_debug( "adv734x_write_value[%#x]=%#x(%d)\n",reg,value,value);
	return ret;
}

static int read_config(struct i2c_client *client, struct adv734x_regval *pt) {

	int i=0;
 
	int val;
	int error=0;

	while ( (i<MAX_ADV_CONFIG_SIZE) && (pt->reg != CONFIG_STOP) && (error != -1) ) {
		error = adv734x_read_value(client, pt->reg, &val);
		pt++;
		i++;
	}

	if(error != -1)
		pr_debug( "adv734x read %d value(s)\n",i);
	return error;

}

static int setup_config(struct i2c_client *client, struct adv734x_regval *pt) {

	int i=0;
	int err=0;
	
	while ( (i<MAX_ADV_CONFIG_SIZE) && (pt->reg != CONFIG_STOP) && (err != -1) ) {
		err = adv734x_write_value(client, pt->reg, pt->val);
		pt++;
		i++;
	}

	if(err != -1)
		pr_debug( "adv734x write %d value(s)\n", i);
	return err;

}	

static int adv734x_command(struct i2c_client* client, unsigned int cmd, int arg)
{
	int res = -EIO;
	
	if (arg == 0) {
		/* format select */
		switch (cmd) {
		case RESOLUTION_VIDEOEXT_PAL:
			/* Set Up PAL */
			res = setup_config(client, _pal_config);
			break;
		case RESOLUTION_VIDEOEXT_NTSC:
			/* Set Up NTSC */
			res = setup_config(client, _ntsc_config);
			break;
		case RESOLUTION_VIDEOEXT_HDPAL:
			/* Set Up NTSC */
			res = setup_config(client, _hdpal_config);
			break;
		
		default:
			break;
		}
	}
	else if (arg == 1) {
		/* analog out select */
		switch (cmd) {
		case ANALOG_OUT_DEFAULT:
		case ANALOG_OUT_CVBS:
			/* Set Up composite */
 			res = setup_config(client, _composite_config);
			/* TODO: userspace control needed
			 * _local_tvp_gio_complente(0,1);
			 * _local_tvp_gio_rgb(0,0);*/
			printk(KERN_INFO "[[TVP]] ANALOG_OUT_CVBS\n");
		break;
		case ANALOG_OUT_SVIDEO:
			/* Set Up svideo */
			res = setup_config(client, _svideo_config);
			/* TODO: userspace control needed
			 * _local_tvp_gio_complente(0,1);
			 * _local_tvp_gio_rgb(0,0);*/
			printk(KERN_INFO "[[TVP]] ANALOG_OUT_SVIDEO\n");
		break;
		case ANALOG_OUT_RGB:
			/* Set Up rgb */
			res = setup_config(client, _rgb_config);
			/* TODO: userspace control needed
			 * _local_tvp_gio_complente(0,1);
			 * _local_tvp_gio_rgb(0,1);*/
			printk(KERN_INFO "[[TVP]] ANALOG_OUT_RGB\n");
		break;
		case ANALOG_OUT_COMPONENT:
			/* Set Up rgb */
			res = setup_config(client, _component_config);
			/* TODO: userspace control needed
			 * _local_tvp_gio_complente(0,0);
			 * _local_tvp_gio_rgb(0,0);*/
			printk(KERN_INFO "[[TVP]] ANALOG_OUT_COMPONENT\n");
		break;
		
		default:
			break;
		}
	} else if (arg == 2 ) {
			printk(KERN_INFO "[[TVP]] Reset ADV\n");
			res = setup_config(client, _sleepon_config);
			/* TODO: userspace control needed
			 * _local_tvp_gio_complente(0,0);
			 * _local_tvp_gio_rgb(0,0); */
	} else if (arg == 3 ) {
		/* dump */
		read_config(client, _dump_config);
		
	} else {
	}

	return res;
}

static ssize_t show_format(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "PAL NTSC HDPAL SLEEP\n");
}

static ssize_t store_format(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct i2c_client *c = to_i2c_client(dev);

	if (!strnicmp(buf, "PAL", strlen("PAL")))
		adv734x_command(c, RESOLUTION_VIDEOEXT_PAL, 0);
	else
	if (!strnicmp(buf, "NTSC", strlen("NTSC")))
		adv734x_command(c, RESOLUTION_VIDEOEXT_NTSC, 0);
	else
	if (!strnicmp(buf, "HDPAL", strlen("HDPAL")))
		adv734x_command(c, RESOLUTION_VIDEOEXT_HDPAL, 0);
	else
	if (!strnicmp(buf, "SLEEP", strlen("SLEEP")))
		adv734x_command(c, 0, 2);
	return count;
}

static DEVICE_ATTR(format, S_IRUGO|S_IWUSR, show_format, store_format);

static ssize_t show_analog_out(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "CVBS SVIDEO RGB COMPONENT\n");
}

static ssize_t store_analog_out(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct i2c_client *c = to_i2c_client(dev);

	if (!strnicmp(buf, "CVBS", strlen("CVBS")))
		adv734x_command(c, ANALOG_OUT_CVBS, 1);
	else
	if (!strnicmp(buf, "SVIDEO", strlen("SVIDEO")))
		adv734x_command(c, ANALOG_OUT_SVIDEO, 1);
	else
	if (!strnicmp(buf, "RGB", strlen("RGB")))
		adv734x_command(c, ANALOG_OUT_RGB, 1);
	else
	if (!strnicmp(buf, "COMPONENT", strlen("COMPONENT")))
		adv734x_command(c, ANALOG_OUT_COMPONENT, 1);
	
	return count;
}

static DEVICE_ATTR(analog_out, S_IRUGO|S_IWUSR, show_analog_out, store_analog_out);

/*
 * I2C driver
 */
static int adv734x_probe(struct i2c_client *i2cc, 
		const struct i2c_device_id *id)
{
	int ret =0;
	struct device *dev = &i2cc->dev;
	
	dev_dbg(dev, "probe id %s\n", id->name);
	
	if (!i2c_check_functionality(i2cc->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "adapter functionality check failed\n");
		return -ENODEV;
	}

	ret = i2c_smbus_read_byte_data(i2cc, 0);
	if (ret < 0) {
		dev_err(dev, "probe: I/O error %d\n", ret);
		return ret;
	}
	
	dev_dbg(dev, "ADV734X power mode = %x\n", ret);

	if ( ret != 0x12 ) {
		dev_warn(dev, "adv734x i2C not ready\n");
		ret = i2c_smbus_write_byte_data(i2cc, 0x0, 0x12);
		if (ret < 0)
			dev_dbg(dev, "ADV734X write register 0 error\n");

		ret = i2c_smbus_read_byte_data(i2cc, 0x0);
		if(ret >= 0)
			dev_dbg(dev, "ADV734X power mode = %x\n", ret);
	} else {
		setup_config(i2cc, _sleepon_config);
		ret = device_create_file(dev, &dev_attr_format);
		if (ret < 0)
			dev_dbg(dev, "cannot create format attribute\n");
		ret = device_create_file(dev, &dev_attr_analog_out);
		if (ret < 0)
			dev_dbg(dev, "cannot create analog_out attribute\n");
	}
	
	return 0;
}
	
static int  __exit adv734x_remove(struct i2c_client *i2cc)
{
	struct device *dev = &i2cc->dev;

	device_remove_file(dev, &dev_attr_format);
	device_remove_file(dev, &dev_attr_analog_out);

	return 0;
}

/*-----------------------------------------------------------------------*/

static const struct i2c_device_id adv734x_id[] = {
	{ "adv734x", (kernel_ulong_t)0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, adv734x_id);

static struct i2c_driver adv734x_driver = {
	.driver = {
		.name	= "adv734x",
		.owner = THIS_MODULE,
	},
	.probe = adv734x_probe,
	.remove = __exit_p(adv734x_remove),
	.id_table = adv734x_id,
};


/*
 *  INIT part
 */
static int __init adv734x_init(void)
{
	int res;
	
	if ((res = i2c_add_driver(&adv734x_driver))) {
		printk("adv734x i2c: Driver registration failed, module not inserted.\n");
		return res;
	}
	
	return 0;
}

static void __exit adv734x_exit(void)
{
	i2c_del_driver(&adv734x_driver);
}

MODULE_AUTHOR("ARCHOS S.A.");
MODULE_DESCRIPTION("I2C interface for ADV7343.");
MODULE_LICENSE("GPL");

module_init(adv734x_init)
module_exit(adv734x_exit)
