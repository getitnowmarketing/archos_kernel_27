/*
 *   Texas Instrumens WM87xx audio codec's i2c interface.
 *   
 *   Copyright (c) by Matthias Welwarsky <welwarsky@archos.com>
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <asm/io.h>
#include <mach/mux.h>

#define DBG if(1)

#define WM87xx_VERSION		"0.1"
#define WM87xx_DATE		"06-November-2006"

struct wm87xx_regval {
	int reg;
	int val;
};

static struct i2c_client *new_client;

struct i2c_client * i2c_get_wm87xx_client(void) {

	return new_client;
}

static inline int wm87xx_write_value(struct i2c_client *client, int reg, int value)
{
	int reg2send,val2send;

 	reg2send = (reg<<1) | ((value & 0x100)>>8);
	val2send = value & 0xFF;

	return i2c_smbus_write_byte_data(client, reg2send, val2send);
}

static int wm87xx_command(struct i2c_client* client, unsigned int cmd, void* arg)
{
	struct wm87xx_regval *regval = (struct wm87xx_regval*)arg;
	int res = -EIO;
	
	switch (cmd) {
	case 0:
		res = wm87xx_write_value(client, regval->reg, regval->val);
		break;
	
	default:
		break;
	}

	return res;
}

static int wm87xx_probe(struct i2c_client *client, 
		const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s functinality check failed\n", id->name);
		return -ENODEV;
	}

	new_client = client;
	return 0;
}
	
static int __exit wm87xx_remove(struct i2c_client *client)
{
	new_client = 0;
	return 0;
}

/*-----------------------------------------------------------------------*/

static const struct i2c_device_id wm87xx_id[] = {
	{"wm87xx", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, wm87xx_id);

static struct i2c_driver wm87xx_driver = {

	.driver = {
		.owner	= THIS_MODULE,
		.name	= "wm87xx",
	},
	.probe		= wm87xx_probe,
	.remove		= __exit_p(wm87xx_remove),
	.id_table	= wm87xx_id,
	.command	= wm87xx_command,
};

/*
 *  INIT part
 */

static int __init wm87xx_init(void)
{
	int res;
	
	if ((res = i2c_add_driver(&wm87xx_driver))) {
		printk("wm87xx i2c: Driver registration failed, module not inserted.\n");
		return res;
	}

	printk("WM87xx I2C version %s (%s)\n", WM87xx_VERSION, WM87xx_DATE);
	
	return 0;
}

static void __exit wm87xx_exit(void)
{
	i2c_del_driver(&wm87xx_driver);
}

MODULE_AUTHOR("Matthias Welwarsky <welwarsky@archos.com>");
MODULE_DESCRIPTION("I2C interface for WM87xx codec.");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(i2c_get_wm87xx_client);

module_init(wm87xx_init)
module_exit(wm87xx_exit)
