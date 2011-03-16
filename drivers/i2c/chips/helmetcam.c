/*
 *   helmetcam i2c interface.
 *
 *   Copyright (c) 2006 Archos
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

#include <linux/helmet.h>

//#define DEBUG
#ifdef DEBUG
#define DEBUGMSG(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DEBUGMSG( x... )
#endif

#define HELMET_CAM_VERSION	"1.0"
#define HELMET_CAM_DATE		"11-September-2008"

#define HELMET_CAM_ADDR		0x5c

#define HELMET_CAM_READ_REG_BASE	0x00
#define HELMET_CAM_STATUS_REG		HELMET_CAM_READ_REG_BASE
#define HELMET_CAM_STATUS_SIZE		2
#define HELMET_CAM_VERSION_REG		( HELMET_CAM_STATUS_REG + HELMET_CAM_STATUS_SIZE )
#define HELMET_CAM_VERSION_SIZE	2

#define  HELMET_CAM_WRITE_REG_BASE	0x00
#define  HELMET_CAM_REREAD_REG_BASE	0x20
#define  HELMET_CAM_COMMAND_REG		HELMET_CAM_WRITE_REG_BASE
#define  HELMET_CAM_COMMAND_SIZE	2
#define  HELMET_CAM_CMD_REQ_REG		( HELMET_CAM_COMMAND_REG + HELMET_CAM_COMMAND_SIZE )
#define  HELMET_CAM_CMD_REQ_SIZE	1

/* standard i2c insmod options */
static unsigned short normal_i2c[] = { HELMET_CAM_ADDR >> 1, I2C_CLIENT_END };
I2C_CLIENT_INSMOD;

static struct i2c_client *new_client;

struct i2c_client * i2c_get_helmetcam_client(void) {

	return new_client;
}


/****************************************************************************
			I2C Command
 ****************************************************************************/

static int helmet_cam_read_reg( unsigned int reg, unsigned int *value, int size )
{
	int ret = 0;
	int err;
	char *tmp;
	int i;

	*value = 0xff;

	if ( !new_client || !new_client->adapter )
		return -ENODEV;

	tmp = kmalloc( size, GFP_KERNEL );
	if ( tmp == NULL )
		return -ENOMEM;

	tmp[0] = reg;
	err = i2c_master_send( new_client, tmp, 1 );
	if ( err < 0 ) {
		printk( "helmet cam i2c read: send error = %d\n", err );
		ret = 1;
		goto exit;
	}

	tmp[0] = 0xff;
	err = i2c_master_recv( new_client, tmp, size );
	if ( err < 0 ) {
		printk( "helmet cam i2c read: recv error = %d\n", err );
		ret = 1;
		goto exit;
	}

	*value = 0;
	for ( i=0 ; i<size ; i++ ) {
		*value |= ( tmp[i] << ( 8 * i ) );
DEBUGMSG( "helmet cam i2c read: reg = %02x data = %02x\n", reg + i, tmp[i] );
	}

exit:
	kfree( tmp );
	return ret;
}

static int helmet_cam_write_reg( unsigned int reg, unsigned char *value, int size )
{
	int ret = 0;
	int err;
	char *tmp;
	int i;

	if ( !new_client || !new_client->adapter )
		return -ENODEV;

	tmp = kmalloc( size + 1, GFP_KERNEL );
	if ( tmp == NULL )
		return -ENOMEM;

	tmp[0] = reg;
	for ( i=0 ; i<size ; i++ ) {
		tmp[i+1] = value[i];
DEBUGMSG( "helmet cam i2c write: reg = %02x data = %02x\n", reg + i, tmp[i+1] );
	}
	err = i2c_master_send( new_client, tmp , size + 1 );
	if ( err < 0 ) {
		printk( "helmet cam i2c write: error = %d\n", err );
		ret = 1;
	}

	kfree( tmp );

	return ret;
}

static int helmet_cam_get_status( void *arg )
{
	unsigned int *status = (unsigned int *)arg;

	if ( helmet_cam_read_reg( HELMET_CAM_STATUS_REG, status, HELMET_CAM_STATUS_SIZE ) ) {
			printk( "helmet_cam_get_status: helmet read failed\n");
			return 1;
	}
	return 0;
}

static int helmet_cam_get_version( void *arg )
{
	unsigned int *version = (unsigned int *)arg;

	if ( helmet_cam_read_reg( HELMET_CAM_VERSION_REG, version, HELMET_CAM_VERSION_SIZE ) ) {
			printk( "helmet_cam_get_version: helmet read failed\n");
			return 1;
	}
	return 0;
}

static int helmet_cam_set_command( void *arg )
{
	char buffer[HELMET_CAM_COMMAND_SIZE + HELMET_CAM_CMD_REQ_SIZE];
	int size = HELMET_CAM_COMMAND_SIZE + HELMET_CAM_CMD_REQ_SIZE;

	buffer[0] = *(unsigned char *)arg;
	buffer[1] = 0;
	buffer[2] = 1;	// command request

	if ( helmet_cam_write_reg( HELMET_CAM_COMMAND_REG, buffer, size ) ) {
			printk( "helmet_cam_set_command: helmet write failed\n");
			return 1;
	}
	return 0;
}

static int helmetcam_command( struct i2c_client *client,
			   unsigned int cmd, void *arg )
{
	switch ( cmd ) {

	case HELMET_CAM_GET_STATUS:
		DEBUGMSG( "helmet i2c cam : HELMET_CAM_GET_STATUS\n" );
		return helmet_cam_get_status( arg );

	case HELMET_CAM_GET_VERSION:
		DEBUGMSG( "helmet i2c cam : HELMET_CAM_GET_VERSION\n" );
		return helmet_cam_get_version( arg );

	case HELMET_CAM_SET_COMMAND:
		DEBUGMSG( "helmet i2c cam : HELMET_CAM_SET_COMMAND\n" );
		return helmet_cam_set_command( arg );

	default:
		return -EINVAL;
	}
}

/****************************************************************************
			I2C Client & Driver
 ****************************************************************************/
static int helmetcam_probe(struct i2c_client *client, 
		const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA| I2C_FUNC_SMBUS_WRITE_BYTE )) {
		pr_err("%s functinality check failed\n", id->name);
		return -ENODEV;
	}

	new_client = client;
	return 0;
}
	
static int __exit helmetcam_remove(struct i2c_client *client)
{
	new_client = 0;
	return 0;
}


/* ----------------------------------------------------------------------- */
static const struct i2c_device_id helmetcam_id[] = {
	{"helmetcam", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, helmetcam_id);

static struct i2c_driver helmet_cam_driver = {
	.driver = {
		.name	= "helmetcam",
	},
	.id_table	= helmetcam_id,
	.probe		= helmetcam_probe,
	.remove		= __exit_p(helmetcam_remove),
	.command 	= helmetcam_command,
};

static int __init helmet_cam_init( void )
{
	int res;
	
	if ( ( res = i2c_add_driver( &helmet_cam_driver ) ) ) {
		printk("helmet cam i2c: Driver registration failed, module not inserted.\n");
		return res;
	}

	printk( "HELMETCAM version %s (%s)\n", HELMET_CAM_VERSION, HELMET_CAM_DATE );

	return 0;
}

static void __exit helmet_cam_exit( void)
{
	i2c_del_driver( &helmet_cam_driver );
}

EXPORT_SYMBOL(i2c_get_helmetcam_client);

module_init( helmet_cam_init );
module_exit( helmet_cam_exit );

/* Module information */
MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("I2C interface for HelmetCam on Archos devices");
MODULE_LICENSE("GPL");
