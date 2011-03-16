/*
 *   Atmel ATMEGA micro controller's i2c interface.
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
#include <linux/delay.h>

#include <asm/io.h>

#include <mach/atmega.h>

#ifdef DEBUG
#define DEBUGMSG(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DEBUGMSG( x... )
#endif

#define ATMEGA_VERSION		"0.1"
#define ATMEGA_DATE		"06-January-2006"
/* on EVM 1 ; on GS06   2  */

#ifdef CONFIG_MACH_ARCHOS_G6H
#undef ATMG_I2C_CHECKSUM
#endif

struct atmega_regval {
	int reg;
	int val;
};

static struct semaphore atmg_i2c_lock;
static struct i2c_client *new_client;

static int atmg_i2c_write_reg( unsigned int reg, unsigned char *value, int size )
{
	int ret;
	char *buf;
	int i;
	
	if ( !new_client )
		return -ENODEV;

	buf = kmalloc( size+1, GFP_KERNEL );
	if ( buf == NULL )
		return -ENOMEM;

	buf[0] = reg;
	memcpy(&buf[1], value, size);

	ret = i2c_master_send( new_client, buf , size + 1 );
	if ( ret < 0 )
		printk( "atmega i2c write: reg = %02x size %i error = %d\n", 
				reg, size, ret );

	kfree( buf );
	return (ret < 0) ? ret:0;
}

static int atmg_i2c_write_table( struct atmega_exchange_table *table )
{
	int ret = 0;
	int i = 0;
	char *buf;

	if ( !new_client )
		return -ENODEV;
	
	buf = kmalloc( ATMEGA_I2C_EXCHANGE_SIZE + 1, GFP_KERNEL );
	if ( buf == NULL )
		return -ENOMEM;

	// fill in sub-address
	buf[i++] = ATMEGA_I2C_EXCHANGE_REG;
	
	// fill in control value
	buf[i++] = table->value & 0xff;
	buf[i++] = (table->value >> 8) & 0xff;
	buf[i++] = (table->value >> 16) & 0xff;
	buf[i++] = (table->value >> 24) & 0xff;

	// fill in control byte and arguments
	buf[i++] = table->control_byte;
	buf[i++] = table->p1;
	buf[i++] = table->p2;
	buf[i++] = table->p3;
	
	// trigger execution of command
	buf[i++] = 1;
	buf[i++] = 0;
	
	ret = i2c_master_send(new_client, buf, i);
	
	kfree( buf );
	return (ret < 0) ? ret:0;
}

static int atmg_i2c_read_reg( unsigned int reg, unsigned char *value, int size )
{
	char sub_addr = reg;
	struct i2c_msg msg[2];
	int ret;

	if ( !new_client )
		return -ENODEV;

	msg[0] = (struct i2c_msg) {
		.addr = new_client->addr,
		.flags = (new_client->flags & I2C_M_TEN),
		.len = 1,
		.buf = &sub_addr,
	};
	msg[1] = (struct i2c_msg) {
		.addr = new_client->addr,
		.flags = (new_client->flags & I2C_M_TEN) | I2C_M_RD,
		.len = size,
		.buf = value,
	};
	
	ret = i2c_transfer(new_client->adapter, msg, 2);
	if ( ret < 0 )
		printk( "atmega i2c read: reg %i send error = %d\n", reg, ret );
	
	return (ret < 0) ? ret:0;
}

/*-----------------------------------------------------------------------*/

static int atmg_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	DEBUGMSG(" atmg_i2c_probe %s id %d", client->adapter->name);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
	
	new_client = client;
	return 0;
}
	
static int __exit atmg_i2c_remove(struct i2c_client *client)
{
	new_client = NULL;
	return 0;
}


static const struct i2c_device_id atmg_id[] = {
	{"atmega", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, atmg_id);

static struct i2c_driver atmg_i2c_driver = {
	.driver = {
		.name	= "atmega",
		.owner	= THIS_MODULE,
	},
	.probe = atmg_i2c_probe,
	.remove = __exit_p(atmg_i2c_remove),
	.id_table = atmg_id,
};

/*
 *  INIT part
 */

static int __init atmg_i2c_init(void)
{
	int res;
	
        init_MUTEX( &atmg_i2c_lock );
        
	if ((res = i2c_add_driver(&atmg_i2c_driver))) {
		printk(KERN_INFO "atmg_i2c: Driver registration failed, module not inserted.\n");
		return res;
	}

	printk(KERN_DEBUG "MSP430 I2C version %s (%s)\n", ATMEGA_VERSION, ATMEGA_DATE);

	return 0;
}

static void __exit atmg_i2c_exit(void)
{
	printk(KERN_DEBUG "atmg_i2c_exit\n");
	i2c_del_driver( &atmg_i2c_driver );
}

static int __atmega_write_value(unsigned int reg, unsigned char* value, int size)
{
	int retry = 20;
	int ret;
	
	do {
		ret = atmg_i2c_write_reg(reg, value, size) ? -1:0;
		if (ret < 0)
			msleep(200);
	} while ((ret < 0) && (retry-- > 0));
	
	return ret;
}

int atmega_write_value( unsigned int reg, unsigned char *value, int size )
{
	int ret;
	
	down( &atmg_i2c_lock );
	ret = __atmega_write_value(reg, value, size);
	up( &atmg_i2c_lock );
	return ret;
}

static int __atmega_write_table( struct atmega_exchange_table *table) 
{	
	int retry = 20;
	int ret;
	
	do {
		ret = atmg_i2c_write_table(table) ? -1:0;
		if (ret < 0)
			msleep(200);
	} while( (ret < 0) && (retry-- > 10));
	
	DEBUGMSG("atmega_write_table cmd = %d\n", table->control_byte );
	return ret;
}

int atmega_write_table( struct atmega_exchange_table *table )
{
	int ret;
	
	down( &atmg_i2c_lock );
	ret = __atmega_write_table(table);
	up( &atmg_i2c_lock );
	return ret;
}

static int __atmega_read_value( unsigned int reg, unsigned char *value, int size )
{
	int retry = 20;
	int ret;
	
	do {
		ret = atmg_i2c_read_reg(reg, value, size) ? -1:0;
		if (ret < 0)
			msleep(20);
	} while ((ret < 0) && (retry-- > 0));
	
	DEBUGMSG("atmega_read_value reg = %d\n", reg );
	return ret;
}

int atmega_read_value( unsigned int reg, unsigned char *value, int size )
{
	int ret;
	
	down( &atmg_i2c_lock );
	ret = __atmega_read_value(reg, value, size);
	up( &atmg_i2c_lock );

	return ret;
}

int atmega_read_eeprom( unsigned char* buf, int offs, int size)
{
	int ret = 0;
	struct atmega_exchange_table table = { 0 };
	
	if (size < 0 || size > 16)
		return -EINVAL;
	
	if (size + offs > 255)
		return -EINVAL;
	
	/* take the lock across all of the transaction to be safe */
	down (&atmg_i2c_lock);
	
	/* read data from eeprom into exchange table */
	table.control_byte = ATMEGA_I2C_CTRL_CMD_EEPROM;
	table.p1 = 1;
	table.p2 = offs;
	table.p3 = size;
	if (__atmega_write_table(&table) < 0) {
		ret = -EIO;
		goto out;
	}
	
	// ugly, wait 1000us until data is available
	udelay(1000);
	
	/* read data from exchange table into caller buffer */
	if (__atmega_read_value(ATMEGA_I2C_READ_EEPROM_REG, buf, size) < 0) {
		ret = -EIO;
		goto out;
	}
	
 out:
	up(&atmg_i2c_lock);
	return ret;
}

int atmega_write_eeprom( const unsigned char *buf, int offs, int size )
{
	int ret = 0;
	struct atmega_exchange_table table = { 0 };
	
	if (size < 0 || size > 16)
		return -EINVAL;
	
	if (size + offs > 255)
		return -EINVAL;
	
	/* take the lock across all of the transaction to be safe */
	down(&atmg_i2c_lock);

	/* write data into exchange table */
	if (__atmega_write_value(ATMEGA_I2C_WRITE_EEPROM_REG, buf, size) < 0) {
		ret = -EIO;
		goto out;
	}

	/* write data from exchange table into eeprom */
	table.control_byte = ATMEGA_I2C_CTRL_CMD_EEPROM;
	table.p1 = 2;
	table.p2 = offs;
	table.p3 = size;
	if (__atmega_write_table(&table) < 0) {
		ret = -EIO;
		goto out;
	}

 out:
	up(&atmg_i2c_lock);
	return ret;
}

MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("I2C interface for ATMEGA uC.");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL(atmega_write_value);
EXPORT_SYMBOL(atmega_write_table);
EXPORT_SYMBOL(atmega_read_value);
EXPORT_SYMBOL(atmega_read_eeprom);
EXPORT_SYMBOL(atmega_write_eeprom);

subsys_initcall(atmg_i2c_init);
module_exit(atmg_i2c_exit);
