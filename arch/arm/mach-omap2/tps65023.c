/*
 * tps65023.c : TPS65023 Power management IC driver for OMAP3 
 *
 * by Pratheesh Gangadhar <pratheesh@ti.com>
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 * Based on twl4030_core.c - driver for TWL4030
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * Modifications to defer interrupt handling to a kernel thread:
 * Copyright (C) 2006 MontaVista Software, Inc.
 *
 * Based on tlv320aic23.c:
 * Copyright (c) by Kai Svahn <kai.svahn@nokia.com>
 *
 * Code cleanup and modifications to IRQ handler.
 * by syed khasim <x0khasim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/random.h>
#include <linux/syscalls.h>
#include <linux/kthread.h>
#include <linux/slab_def.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>

#include <mach/power_companion.h>
#include <mach/pm.h>
#include <mach/resource.h>
#include <mach/prcm_34xx.h>

#define DRIVER_NAME			"tps65023"

/**** Macro Definitions */
#define TPS65023_CLIENT_STRING		"TPS65023-ID"
#define TPS65023_CLIENT_USED			1
#define TPS65023_CLIENT_FREE			0

/* Register definitions */
#define TPS65023_VERSION		0x00
#define TPS65023_PGOODZ		0x01
#define TPS65023_MASK			0x02
#define TPS65023_REG_CTRL		0x03
#define TPS65023_CON_CTRL		0x04
#define TPS65023_CON_CTRL2		0x05
#define TPS65023_DEFCORE		0x06
#define TPS65023_DEFSLEW		0x07
#define TPS65023_LDOCTRL		0x08


struct tps65023_client  {
	struct i2c_client* i2cc;
	unsigned char inuse;
} tps65023_module;

#ifdef CONFIG_OMAP3_PM
struct constraint_handle *co_opp_tps65023_vdd2;
static struct constraint_id cnstr_id_vdd2 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd2_opp",
};
#endif

/**
 * @brief tps65023_i2c_write8 - Writes a 8 bit register in TPS65023
 *
 * @param value - the value to be written 8 bit
 * @param reg - register address (just offset will do)
 *
 * @return result of operation - 0 is success
 */
static int tps65023_i2c_write8( int reg, u8 val)
{
	struct tps65023_client *client = &tps65023_module;
	struct i2c_msg xfer_msg[1];
	u8 buf[2] = { reg, val };
	int retry = 0;
	
	if (unlikely(client->inuse != TPS65023_CLIENT_USED)) {
		printk(KERN_ERR "tps65023_i2c_write: I2C Client not initialized\n");
		return -ENODEV;
	}

write_again:
	xfer_msg[0] = (struct i2c_msg) { .addr = client->i2cc->addr, 
		.flags = 0, .buf = buf, .len = 2 };
	
	if (i2c_transfer(client->i2cc->adapter, xfer_msg, 1) != 1) {
		if (retry <= 10) {
			printk(KERN_ERR "Write: retry ... %d\n", retry);
			retry++;
			msleep_interruptible(10);
			goto write_again;
		}
	} else
		return 0;
	
	return -EIO;
}

/**
 * @brief tps65023_i2c_read8 - Reads a 8 bit register from TPS65023
 *
 * @param *val - the value read 8 bit
 * @param reg - register address (just offset will do)
 *
 * @return result of operation - 0 is success
 */
static int tps65023_i2c_read8( int reg, u8 *val)
{
	struct tps65023_client *client = &tps65023_module;
	struct i2c_msg xfer_msg[2];
	int retry = 0;

	u8 buf[1] = { reg };

	if (unlikely(client->inuse != TPS65023_CLIENT_USED)) {
		printk(KERN_ERR "tps65023_i2c_read: I2C Client not initialized\n");
		return -ENODEV;
	}

read_again:
	xfer_msg[0] = (struct i2c_msg) { .addr = client->i2cc->addr, 
				.flags = 0,        .buf = buf, .len = 1 };
	xfer_msg[1] = (struct i2c_msg) { .addr = client->i2cc->addr, 
				.flags = I2C_M_RD, .buf = val, .len = 1 };
	
	if (i2c_transfer(client->i2cc->adapter, xfer_msg, 2) != 2) {
		if (retry <= 10) {
			printk(KERN_ERR "Read: retry ... %d\n", retry);
			retry++;
			msleep_interruptible(10);
			goto read_again;
		}
	} else
		return 0;
	
	return -EIO;
}

/*
 *
 */
static int core_adjust_enable(int enable)
{
	int  ret;
	u8 val;

	struct tps65023_client *client = &tps65023_module;

	if (unlikely(client->inuse != TPS65023_CLIENT_USED)) {
		pr_err("core_adjust_enable: I2C Client is not initialized\n");
		return -ENODEV;
	}

	ret = tps65023_i2c_read8(TPS65023_CON_CTRL2, &val);  
	if (ret < 0) {
		pr_debug("failed: get CON_CTRL2\n");
		goto out;
	}
		
	if (enable)
	        val &= ~0x40;
	else
		val |= 0x40;

        ret = tps65023_i2c_write8(TPS65023_CON_CTRL2, val);  
	if (ret < 0) {
		pr_debug("failed: set CON_CTRL2 %d\n", val);
		goto out;
	}
	
out:
	return ret;
}

u8 mpu_iva2_vdd1_volts [PRCM_NO_VDD1_OPPS] = {
	/* Vsel corresponding to 0.95V (OPP1), 1.00V (OPP2),
				1.20V (OPP3), 1.27V (OPP4), 1.35 (OPP5)
				1.35V (OPP6) 1.425 (OPP7) */

	/* OPP1 (0.95V) */	0x06,
	/* OPP2 (1.00V) */	0x08,
	/* OPP3 (1.20V) */	0x10,
	/* OPP4 (1.27V) */	0x13,
	/* OPP5 (1.35V) */	0x16, 
	/* OPP6 (1.35V) */	0x16,
	/* OPP7 (1.425V) */	0x19,	// according to TI, 1.425 is OK!
};

static unsigned int vsel_vdd1 = -1;

/*
 * This function sets the VSEL values in the Power IC. This is done in
 * the software configurable mode.
 */
int set_voltage_level (u8 vdd, u8 vsel)
{
	int  ret;
	u8 val;

	struct tps65023_client *client = &tps65023_module;

	if (unlikely(client->inuse != TPS65023_CLIENT_USED)) {
		pr_err("set_voltage_level: I2C Client is not initialized\n");
		return -ENODEV;
	}

        if (vdd != PRCM_VDD1) {
		pr_err("DVFS on VDD2: Not supported");
                ret = -EINVAL;
		goto out;		
        } 
	
	pr_debug("Setting voltage on VDD%d to %d \n",  vdd, vsel);

	ret =  tps65023_i2c_write8(TPS65023_DEFCORE, vsel);
	if (ret < 0) {
		pr_debug("failed: set DEFCORE vsel %d\n", vsel);
		goto out;
	}

#ifdef DEBUG
	ret = tps65023_i2c_read8(TPS65023_DEFCORE, &vsel);  
	if (ret < 0) {
		pr_debug("failed: get DEFCORE\n");
		goto out;
	}
	pr_debug("set_voltage_level: DEFCORE value %d\n", vsel);
#endif

	ret = tps65023_i2c_read8(TPS65023_CON_CTRL2, &val);  
	if (ret < 0) {
		pr_debug("failed: get CON_CTRL2\n");
		goto out;
	}
	
	/* set the GO bit to establish the new voltage */
	val |= 0x80;
	
        ret = tps65023_i2c_write8(TPS65023_CON_CTRL2, val);  
	if (ret < 0) {
		pr_debug("failed: set CON_CTRL2 %d\n", val);
		goto out;
	}
	
	vsel_vdd1 = vsel;
	
	/* Wait for voltage to stabilize */
	msleep(1);

out:
	return ret;
}
EXPORT_SYMBOL(set_voltage_level);

static ssize_t show_voltage_level(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d -> %d mV\n", vsel_vdd1, 800 + vsel_vdd1 * 25 );
}

static DEVICE_ATTR(vsel_vdd1, S_IRUGO, show_voltage_level, NULL);

/* 
 * I2C Client Driver
 */
static int tps65023_probe(struct i2c_client *i2cc, 
		const struct i2c_device_id *id)
{
	int ret;
	
	struct tps65023_client *client = &tps65023_module;

 	/* Check basic functionality */
	if (!i2c_check_functionality(i2cc->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("TPS65023 client functionality check failed\n");
		return -EIO;
	}

	if (unlikely(client->inuse)) {
		pr_err("Client %s is already in use\n", id->name);
		return -EBUSY;
	}

	client->i2cc = i2cc;
	client->inuse = TPS65023_CLIENT_USED;
	i2c_set_clientdata(i2cc, client);
	
	ret = device_create_file(&i2cc->dev, &dev_attr_vsel_vdd1);
	if (ret < 0)
		pr_debug("tps65023_probe: failed to add device attribute\n");
	
	return 0;
}

static int __exit tps65023_remove(struct i2c_client *i2cc)
{
	struct tps65023_client *client = i2c_get_clientdata(i2cc);
	
	client->inuse = TPS65023_CLIENT_FREE;
	i2c_set_clientdata(i2cc, NULL);
	return 0;
}

static const struct i2c_device_id tps65023_id[] = {
	{ "tps65023", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tps65023_id);

static struct i2c_driver tps65023_driver = {
	.driver = {
		.name	= "tps65023",
		.owner = THIS_MODULE,
	},
	.probe = tps65023_probe,
	.remove = __exit_p(tps65023_remove),
	.id_table = tps65023_id,
};

static int __init tps65023_init(void)
{
	int res;
        
	tps65023_module.inuse = TPS65023_CLIENT_FREE;

#ifdef CONFIG_OMAP3_PM
	/* FIXME: set a constraint on VDD2, since we can't scale VDD2 */
	co_opp_tps65023_vdd2 = constraint_get("tps65023", &cnstr_id_vdd2);
	constraint_set(co_opp_tps65023_vdd2, CO_VDD2_OPP3);
#endif
	
	if ((res = i2c_add_driver(&tps65023_driver)) < 0) {
		pr_err("TPS65023 PMIC driver registration failed\n");
		return res;
	}

	/* enable core adjustment via i2c */
	core_adjust_enable(1);
	
#ifdef DEBUG
	{
		u8 val;
		tps65023_i2c_read8(TPS65023_VERSION, &val);
		pr_debug("TPS650XX Version: %X \n", val);

		/* read out PGOODZ */
		tps65023_i2c_read8(TPS65023_PGOODZ, &val);
		pr_debug("PGOODZ: %d\n", val);
	}
#endif

	return 0;
}

static void __exit tps65023_exit(void)
{
	i2c_del_driver(&tps65023_driver);
#ifdef CONFIG_OMAP3_PM
	constraint_remove(co_opp_tps65023_vdd2);
	constraint_put(co_opp_tps65023_vdd2);
#endif
}

device_initcall(tps65023_init);
