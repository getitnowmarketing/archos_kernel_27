/*
 * helmet.c
 *
 * Copyright (c) 2008 Archos
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/freezer.h>

#include <linux/helmet.h>

#include <asm/uaccess.h>


#define HELMET_VERSION		"1.0"

#define HELMET_IS_OPEN		0x01	/* means /dev/helmetcam is in use */

#define HELMET_POLL_DELAY	(HZ/3)	/* poll helmetcam every 0.3 sec */

#define HELMET_STATUS_NOT_VALID	0xff

static unsigned char helmet_open_status;
static struct i2c_client *helmet_handle = NULL;
static struct semaphore helmet_i2c_lock;

#ifdef	HELMET_POLLING_IN_DRIVER
static unsigned int helmet_status = HELMET_STATUS_NOT_VALID;
static wait_queue_head_t helmet_wait_queue;
static int helmet_status_available = 0;
static struct task_struct *helmet_monitor_handle = NULL;
#endif

//#define DEBUG
#ifdef DEBUG
#define DEBUGMSG(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DEBUGMSG( x... )
#endif

extern struct i2c_client * i2c_get_helmetcam_client(void);


/****************************************************************************
			helmetcam i2c driver access
*****************************************************************************/

static int helmet_setcmd( unsigned long arg )
{
	unsigned char request;
	int res;

	if ( helmet_handle == NULL )
		return -ENODEV;

	request = ( unsigned char )arg;
	down( &helmet_i2c_lock );
	res = helmet_handle->driver->command( helmet_handle, HELMET_CAM_SET_COMMAND, &request );
	up( &helmet_i2c_lock );
	return res;
}

static int helmet_getversion( unsigned long arg )
{
	void __user *version = (void __user *)arg;
	unsigned int value;

	if ( helmet_handle == NULL )
		return -ENODEV;

	down( &helmet_i2c_lock );
	helmet_handle->driver->command( helmet_handle, HELMET_CAM_GET_VERSION, &value );
	up( &helmet_i2c_lock );
	return copy_to_user( version, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int helmet_getstatus( unsigned long arg )
{
	void __user *status = (void __user *)arg;
	unsigned int value;

	if ( helmet_handle == NULL )
		return -ENODEV;

	down( &helmet_i2c_lock );
	helmet_handle->driver->command( helmet_handle, HELMET_CAM_GET_STATUS, &value );
	up( &helmet_i2c_lock );
	return copy_to_user( status, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int helmet_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
	DEBUGMSG( "helmet : helmet_ioctl cmd->%d\n", cmd );

	switch( cmd ) {
	case HELMET_CAM_GET_STATUS:
		DEBUGMSG( "helmet : HELMET_GET_STATUS\n" );
		return helmet_getstatus( arg );

	case HELMET_CAM_GET_VERSION:
		DEBUGMSG( "helmet : HELMET_GET_VERSION\n" );
		return helmet_getversion( arg );

	case HELMET_CAM_SET_COMMAND:
		DEBUGMSG( "helmet : HELMET_SET_COMMAND\n" );
		return helmet_setcmd( arg );

	default:
		return -EINVAL;
	}
}

#ifdef	HELMET_POLLING_IN_DRIVER
static void helmet_monitor_read( unsigned int *status )
{
	if ( helmet_handle == NULL ) {
		*status = HELMET_STATUS_NOT_VALID;
	} else {
		down( &helmet_i2c_lock );
		if ( helmet_handle->driver->command( helmet_handle, HELMET_CAM_GET_STATUS, status ) ) {
			*status = HELMET_STATUS_NOT_VALID;
		}
		up( &helmet_i2c_lock );
	}
}

static int helmet_monitor_status( void *data )
{
	current->flags &= ~PF_NOFREEZE;

	while ( !kthread_should_stop() ) {
		unsigned int new_status;

		set_current_state( TASK_INTERRUPTIBLE );
		schedule_timeout( HELMET_POLL_DELAY );

		/* swsusp cooperativity */
		try_to_freeze();

		if ( helmet_status_available ) {
DEBUGMSG("helmet_monitor_status : helmet_status_available is already set\n" );
			continue;
		}

		helmet_monitor_read( &new_status );
DEBUGMSG("helmet_monitor_status : new_status %d\n", new_status );
		if ( new_status != helmet_status ) {
			unsigned int ctrl_status;
			msleep( 30 );
			helmet_monitor_read( &ctrl_status );
DEBUGMSG("helmet_monitor_status : ctrl_status %d\n", ctrl_status );
			if ( ctrl_status != new_status ) {
				continue;
			}
		}
		helmet_status = new_status;
		helmet_status_available = 1;
		wake_up_interruptible( &helmet_wait_queue );
	}

	return 0;
}
#endif	// HELMET_POLLING_IN_DRIVER

static int helmet_open( struct inode *inode, struct file *file )
{
	if ( helmet_open_status & HELMET_IS_OPEN )
		return -EBUSY;

	if ( helmet_handle == NULL) {

		helmet_handle = i2c_get_helmetcam_client();

		if ( helmet_handle == NULL ) {
			printk( "helmet: no helmet found!\n" );
			return -ENODEV;
		}
	}

#ifdef	HELMET_POLLING_IN_DRIVER
	if ( helmet_monitor_handle == NULL ) {
		helmet_handle->driver->command( helmet_handle, HELMET_CAM_GET_STATUS, &helmet_status );
		helmet_monitor_handle = kthread_run( helmet_monitor_status, NULL, "helmetmon" );
	}
#endif

	helmet_open_status |= HELMET_IS_OPEN;

	return 0;
}

static int helmet_release( struct inode *inode, struct file *file )
{
	helmet_handle = NULL;

#ifdef	HELMET_POLLING_IN_DRIVER
	if ( helmet_monitor_handle != NULL ) {
		kthread_stop( helmet_monitor_handle );
		helmet_monitor_handle = NULL;
	}
#endif

	helmet_open_status &= ~HELMET_IS_OPEN;
	return 0;
}

#ifdef	HELMET_POLLING_IN_DRIVER
static ssize_t helmet_read( struct file *file, char __user *buffer, size_t count, loff_t *ppos )
{
	int len = 0;

	if ( count > 0 ) {
		*buffer = helmet_status;
		len = 1;
	}

	helmet_status_available = 0;

	return len;
}

static unsigned int helmet_poll( struct file *file, poll_table *wait )
{
	unsigned int mask = 0;

	if ( helmet_status_available )
		mask |= POLLIN | POLLRDNORM;

	poll_wait( file, &helmet_wait_queue, wait );

	return mask;
}
#endif

/*
 *	The various file operations we support.
 */

static struct file_operations helmet_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= helmet_ioctl,
	.open		= helmet_open,
	.release	= helmet_release,
#ifdef	HELMET_POLLING_IN_DRIVER
	.read		= helmet_read,
	.poll		= helmet_poll,
#endif
};

static struct miscdevice helmet_dev =
{
	.minor		= HELMET_MINOR,
	.name		= "helmet",
	.fops		= &helmet_fops,
};

static int __init helmet_init( void )
{

	printk( "%s:\n", __FUNCTION__ );

	/* register helmet device */
	misc_register( &helmet_dev );

#ifdef	HELMET_POLLING_IN_DRIVER
	init_waitqueue_head( &helmet_wait_queue );
#endif

	init_MUTEX( &helmet_i2c_lock );

	return 0;
}

static void __exit helmet_exit( void )
{
	misc_deregister( &helmet_dev );

}

module_init( helmet_init );
module_exit( helmet_exit );


/****************************************************************************
			Module Info
*****************************************************************************/

MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("HELMET Driver for Archos Series");
MODULE_LICENSE("GPL");
