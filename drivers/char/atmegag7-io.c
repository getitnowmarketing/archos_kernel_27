/*
 * atmega-io.c
 *
 * Copyright (c) 2006 Archos
 * Author: Xavier Leclercq
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
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/wakelock.h>

#include <mach/atmega-io.h>
#include <mach/atmega.h>
#include <mach/archos-battery.h>

#include <mach/board.h>
#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/archos-gpio.h>

#include <linux/switch.h>

#define ATMEGA_IO_VERSION	"0.05"

#define ATMEGA_IO_IS_OPEN	0x01	/* means /dev/atmg is in use */

#define ATMEGA_KB_DEV_NAME "atmega-key"

//Signalling other events
#define DRIVER_CALL "ATMEGA_SWITCH" 
#define ACCESSORY_SWITCH_NAME "accessory"

enum USB_CABLE_STATE {
	USB_UNKNOWN=0,
	USB_ATTACHED,
	USB_DETACHED,
};

static struct atmega_io {
	unsigned long status;
	unsigned long vbatt;
	int irq;
	unsigned long version;
	struct workqueue_struct *workqueue;
	struct work_struct work;
	wait_queue_head_t wait_queue;
	struct input_dev* input;
	int atmega_open_counter;
	struct list_head clients_lh;
	struct wake_lock wake_lock;

	struct switch_dev usb_switch;
	struct switch_dev accessory_switch;
	unsigned long module_resistor;
} *atmega_io;

struct atmega_status {
	struct list_head lh;
	unsigned long status;
	int status_available;
};

static const struct str_module_resistors {
	int module_id;
	int min_resistor;
	int max_resistor;
}  module_resistors[] = 
{
	{ MODULE_ID_TV_CRADLE,		43,	51 },
	{ MODULE_ID_REMOTE_FM,		51,	62 },
	{ MODULE_ID_VRA_GEN6,		62,	75 },
	{ MODULE_ID_GPS_WITHOUT_TMC,	75,	91 },
	{ MODULE_ID_USB_HOST_CABLE,	91,	110},
	{ MODULE_ID_BATTERY_DOCK_GEN6,	110,	135},
	{ MODULE_ID_POWER_CABLE,	135,	165},
	{ MODULE_ID_SERIAL_ADAPTER,	165,	200},
	{ MODULE_ID_PC_CABLE,		200,	245},
	{ MODULE_ID_UNUSED10,		245,	300},
	{ MODULE_ID_GPS_WITH_TMC,	300,	360},
	{ MODULE_ID_HDMI_DOCK,		360,	430},
	{ MODULE_ID_MINI_DOCK,		430,	515},
	{ MODULE_ID_DVBT_SNAP_ON,	516,	620},
	{ MODULE_ID_MUSIC_DOCK,		620,	750},
};

extern void atmega_rtc_set_diff(long newDiff);
extern long atmega_rtc_get_diff(void);

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DEBUGMSG(ARGS...)  do { printk(KERN_DEBUG "<%s>: ",__FUNCTION__); printk(ARGS); } while (0)
#else
#define DEBUGMSG( x... ) do {} while (0)
#endif

static const int use_wake_lock = 1;
static int atmega_io_getalarm(unsigned long *arg);

extern void platform_enable_irr_io(int en);
extern int platform_get_dock_dc(struct device* dev);

static long time_offset[4];

static int atmega_update_module_id(struct atmega_io *io)
{
	unsigned long value;
	unsigned long adc1 = 0, adc2 = 0;
	int i;

	atmega_read_value( ATMEGA_I2C_ADC1_REG, (unsigned char*)&adc1, ATMEGA_I2C_ADC1_SIZE );
	atmega_read_value( ATMEGA_I2C_ADC2_REG, (unsigned char*)&adc2, ATMEGA_I2C_ADC2_SIZE );

	value = adc1*510*1000/(2*adc2-adc1)/1000;
	io->module_resistor = value;
	
	for ( i = 0; i < ARRAY_SIZE(module_resistors); i++ ) {
		if (( value < module_resistors[i].max_resistor ) 
				&& (value > module_resistors[i].min_resistor)) 
		{
			switch_set_state(&io->accessory_switch, module_resistors[i].module_id);
			return 0;
		}
	}
	
	switch_set_state(&io->accessory_switch, 16);
	return 0;
}

/* IRQ Handler */ 
static void atmega_io_handle_status(struct atmega_io *io)
{
	unsigned long status = 0, changed, do_wake_lock;
	unsigned long old_usb, new_usb;
	struct atmega_status *cl_status;

	atmega_read_value( ATMEGA_I2C_STATUS_REG, (unsigned char*)&status, ATMEGA_I2C_STATUS_SIZE );
	DEBUGMSG( "Status %lx\n", status );

	changed = io->status ^ status;
	
	if (changed & ATMEGA_IO_STATUS_KBON_PRESSED_BIT) {
		printk( "ATMEGA KBON %s\n", status & ATMEGA_IO_STATUS_KBON_PRESSED_BIT ? "pressed" : "released");
		
		input_event(io->input, EV_KEY, KEY_POWER, status & ATMEGA_IO_STATUS_KBON_PRESSED_BIT ? 1:0);
		input_sync(io->input);
	}
			
	if (changed & (ATMEGA_IO_STATUS_AUX_DETECTED_BIT 
		     | ATMEGA_IO_STATUS_DC_DETECTED_BIT
		     | ATMEGA_IO_STATUS_USB_EXT_DETECTED_BIT
		     | ATMEGA_IO_STATUS_USB_LOCAL_DETECTED_BIT)) {
		archos_needs_battery(status & (ATMEGA_IO_STATUS_USB_LOCAL_DETECTED_BIT | ATMEGA_IO_STATUS_USB_EXT_DETECTED_BIT) ? 1:0, 
						status & ATMEGA_IO_STATUS_DC_DETECTED_BIT ? 1:0);
	}

	if (status & ATMEGA_IO_STATUS_ALARM_REACHED_BIT) {
		unsigned long almtime;
		
		atmega_io_getalarm(&almtime);
		printk(KERN_DEBUG "ATMEGA Alarm reached: alarm counter %lu\n", almtime);
	}

	old_usb = io->status & (ATMEGA_IO_STATUS_USB_LOCAL_DETECTED_BIT | ATMEGA_IO_STATUS_USB_EXT_DETECTED_BIT);
	new_usb = status & (ATMEGA_IO_STATUS_USB_LOCAL_DETECTED_BIT | ATMEGA_IO_STATUS_USB_EXT_DETECTED_BIT);

	if ( old_usb && !new_usb) {
		printk(KERN_DEBUG "ATMEGA USB CABLE DETACHED\n");	
		switch_set_state(&io->usb_switch, USB_DETACHED);
	} else if ( !old_usb && new_usb) {
		printk(KERN_DEBUG "ATMEGA USB CABLE ATTACHED\n");	
		switch_set_state(&io->usb_switch, USB_ATTACHED);
	}

	if (use_wake_lock) {
		/* wake lock if dc in or power button pressed */
		do_wake_lock = status & (ATMEGA_IO_STATUS_KBON_PRESSED_BIT 
				| ATMEGA_IO_STATUS_DC_DETECTED_BIT
				| ATMEGA_IO_STATUS_USB_EXT_DETECTED_BIT
				| ATMEGA_IO_STATUS_USB_LOCAL_DETECTED_BIT);	
		if (!wake_lock_active(&io->wake_lock) && do_wake_lock)
			wake_lock(&io->wake_lock);
		else if (wake_lock_active(&io->wake_lock) && !do_wake_lock)
			wake_unlock(&io->wake_lock);
	}

	if (changed & ATMEGA_IO_STATUS_AUX_DETECTED_BIT)
		atmega_update_module_id(io);
	
	io->status = status;
	
	/* update all clients' status */
	list_for_each_entry(cl_status, &io->clients_lh, lh) {
		cl_status->status = status;
		cl_status->status_available = 1;
	}
	wake_up_interruptible(&io->wait_queue);
}

static void atmega_io_worker(struct work_struct *work)
{
	struct atmega_io *io = container_of(work, struct atmega_io, work);

	atmega_io_handle_status(io);
}

/* ----- Utility functions --------------------------------------------	*/

static irqreturn_t atmega_io_statusirq( int irq, void *dev_id )
{
	struct platform_device *pdev = dev_id;
	struct atmega_io *io = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "statusirq\n");
	queue_work( io->workqueue, &io->work );

	return IRQ_HANDLED;
}

static int update_time_offset(int clocknum)
{
	if (clocknum < USER_TIME || clocknum > GPS_TIME)
		return -EINVAL;
	
	if (time_offset[clocknum] == 0) {
		int res;
		
		/* no cached value, read from eeprom */
		res = atmega_read_eeprom((unsigned char*)&time_offset[clocknum], 
				clocknum * sizeof(long), sizeof(long));
		if (res < 0)
			return res;
	}
	return 0;
}

int atmega_io_get_clockval( struct rtc_time *arg, int clocknum)
{
	unsigned long time_counter = 0;
	long time_diff;
	int res;
	
	if (clocknum < USER_TIME || clocknum > ALARM_TIME)
		return -EINVAL;

	printk("atmega_io_get_clockval: clocknum %i ", clocknum);
	
	/* read either alarm register or seconds counter */
	if (clocknum == ALARM_TIME) {
		res = atmega_read_value( ATMEGA_I2C_ALARM_REG, 
			(unsigned char*)&time_counter, ATMEGA_I2C_ALARM_SIZE );
		/* alarm time is also an offset to user time */
		clocknum = USER_TIME;
	} else {
		res = atmega_read_value( ATMEGA_I2C_TIME_REG, 
			(unsigned char*)&time_counter, ATMEGA_I2C_TIME_SIZE );
	}
	if (res < 0)
		return res;

	res = update_time_offset(clocknum);
	if (res < 0)
		return res;
	
	time_diff = time_offset[clocknum];

	printk("time_counter: %lu time_diff %li\n", time_counter, time_diff);
	
	time_counter += time_diff;
	rtc_time_to_tm(time_counter, arg);
	
	
	return 0;
}

int atmega_io_set_clockval( const struct rtc_time *arg, int clocknum)
{
	unsigned long time_counter = 0;
	unsigned long newtime;
	long time_diff;
	int res;
	
	if (clocknum < USER_TIME || clocknum > ALARM_TIME)
		return -EINVAL;
	
	if (clocknum == ALARM_TIME) {
		struct atmega_exchange_table table = { 0,0,0,0,0 };

		res = update_time_offset(USER_TIME);
		if (res < 0)
			return res;
		
		rtc_tm_to_time((struct rtc_time*)arg, &newtime);
		newtime -= time_offset[USER_TIME];
		
		printk("atmega_io_set_clockval: alarm time %lu\n", newtime);
		
		table.value = newtime;
		table.control_byte = ATMEGA_I2C_CTRL_CMD_SET_ALARM;
		return atmega_write_table( &table );		
	}
	
	res = atmega_read_value( ATMEGA_I2C_TIME_REG, 
			(unsigned char*)&time_counter, ATMEGA_I2C_TIME_SIZE );
	if (res < 0)
		return res;
	
	rtc_tm_to_time((struct rtc_time*)arg, &newtime);
	time_diff = newtime - time_counter;
	time_offset[clocknum] = time_diff;
	
	printk("atmega_io_set_clockval: time_counter: %lu "
			"time_diff: %li, clocknum %i\n", time_counter, time_diff, clocknum);
	
	res = atmega_write_eeprom((unsigned char*)&time_diff, 
			clocknum * sizeof(long), sizeof(long));
	if (res < 0)
		return res;
	
	return 0;
}

static int atmega_io_dump( unsigned long arg )
{
	unsigned int i;
	unsigned char data;

	for (i=ATMEGA_I2C_READ_REG_BASE; i < ATMEGA_I2C_READ_REG_BASE+26; i++) {
		atmega_read_value(i, &data, 1);
		printk(KERN_INFO "atmg reg[%02X]: %02x\n", i, data);
	}
	for (i=ATMEGA_I2C_REREAD_REG_BASE; i < ATMEGA_I2C_REREAD_REG_BASE+10; i++) {
		data = atmega_read_value(i, &data, 1);
		printk(KERN_INFO "atmg exchangtable[%02X]: %02x\n", i, data);
	}
	return 0;
}

static int atmega_io_getalarm( unsigned long *alrtime )
{
	unsigned long nowtime = 0;
	int res;
	
	if (alrtime == NULL)
		return -EINVAL;
	
	res = atmega_read_value( ATMEGA_I2C_ALARM_REG, 
			(unsigned char*)&nowtime, ATMEGA_I2C_ALARM_SIZE );
	if (res < 0)
		return res;
	
	*alrtime = nowtime;
	
	return res;
}

static int atmega_io_setalarm( unsigned long alrtime )
{
	struct atmega_exchange_table table = { 0,0,0,0,0 };

	if ( alrtime ) {
		table.value = alrtime;
		table.control_byte = ATMEGA_I2C_CTRL_CMD_SET_ALARM;
		printk("atmega_io_setalarm: SET nowtime %lu\n", alrtime);
	} else {
		table.control_byte = ATMEGA_I2C_CTRL_CMD_RESET_ALARM;
		printk("atmega_io_setalarm: CLEAR\n");
	}

	return ( atmega_write_table( &table ));
}

static int atmega_io_reset_omap( void )
{
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_RESET_DAVINCI, 0,0,0 };
	return ( atmega_write_table( &table ) );
}

static int atmega_io_set_shutdown( void )
{
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_SHUTDOWN, 0,0,0 };
	return ( atmega_write_table( &table ) );
}

static int atmega_io_set_plug_power( unsigned long arg )
{
	return 0;
}

struct str_led_cmd {
	unsigned char period;
	unsigned char blink;
};

static int atmega_io_set_led_charge( unsigned long arg )
{
	struct str_led_cmd *led_cmd = (struct str_led_cmd *)arg;

	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_CHG_LED_TOGGLE, 0,0,0 };

	DEBUGMSG(KERN_DEBUG "Set charge led: arg %lx, p1:%d(%04x), p2:%d(%04x)\n",
			arg,led_cmd->period,led_cmd->period,led_cmd->blink,led_cmd->blink);
	table.p1 = 1;
	table.p2 = led_cmd->period;
	table.p3 = led_cmd->blink;

	return ( atmega_write_table( &table ) );
}

static int atmega_io_set_led_charge_off( unsigned long arg )
{
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_CHG_LED_TOGGLE, 0,0,0 };
	return ( atmega_write_table( &table ) );
}

static int atmegag7_io_set_charge_mode( unsigned long arg )
{
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_CHARGEMODE, 0,0,0 };
	
	table.value = arg;
	return atmega_write_table( &table );
}

static int atmega_io_getversion( unsigned long arg )
{
	void __user *version = (void __user *)arg;
	unsigned long value = 0;

	atmega_read_value( ATMEGA_I2C_VERSION_REG, (unsigned char*)&value, ATMEGA_I2C_VERSION_SIZE );
	return copy_to_user( version, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_getboardID( unsigned long arg )
{
	void __user *boardID = (void __user *)arg;
	unsigned long value = 0;

	atmega_read_value( ATMEGA_I2C_BOARDID_REG, (unsigned char*)&value, ATMEGA_I2C_BOARDID_SIZE );
	return copy_to_user( boardID, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_getchecksum( unsigned long arg )
{
	void __user *version = (void __user *)arg;
	unsigned long value = 0;

	atmega_read_value( ATMEGA_I2C_CS_REG, (unsigned char*)&value, ATMEGA_I2C_CS_SIZE );
	return copy_to_user( version, &value, sizeof(value) ) ? -EFAULT : 0;
}

/* flush the queue and get the last status value */
static int atmega_io_getstatus( struct atmega_status *priv_status, unsigned long arg )
{
	void __user *status = (void __user *)arg;
	unsigned long value = 0;

	//DEBUGMSG("atmega_io_getstatus\n");
	
	value = priv_status->status;
	priv_status->status_available = 0;
	return copy_to_user( status, (unsigned char*)&value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_getvbatt( struct atmega_io *io, unsigned long arg )
{
	void __user *vbatt = (void __user *)arg;
	unsigned long adc0 = 0, adc3 = 0,value;

	atmega_read_value( ATMEGA_I2C_ADC0_REG, (unsigned char*)&adc0, ATMEGA_I2C_ADC0_SIZE );
	atmega_read_value( ATMEGA_I2C_ADC3_REG, (unsigned char*)&adc3, ATMEGA_I2C_ADC3_SIZE );

	//DEBUGMSG("atmega_io : adc0:%lu adc3:%lu\n",adc0,adc3);
	//value =  adc0*11*43047/1024/100;
	//value_ref = adc3*11*430/1024;
	
	if ( adc3 == 0 ) {
		value = io->vbatt;
		printk(KERN_DEBUG "atmega_io : getVbatt ADC3 is 0 \n");
	} else {
		value = adc0 * 2500 / adc3;
		io->vbatt = value;
	}
	//DEBUGMSG("atmega_io : atmega_io_getvbatt: %lu vbatt reference: %lu\n",value, value_ref);
	return copy_to_user( vbatt, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_getmodule( unsigned long arg )
{
	void __user *module = (void __user *)arg;
	unsigned long value;
	unsigned long adc1 = 0, adc2 = 0;

	atmega_read_value( ATMEGA_I2C_ADC1_REG, (unsigned char*)&adc1, ATMEGA_I2C_ADC1_SIZE );
	atmega_read_value( ATMEGA_I2C_ADC2_REG, (unsigned char*)&adc2, ATMEGA_I2C_ADC2_SIZE );

	DEBUGMSG("atmega_io : GET MODULE TYPE adc1:%lu adc2:%lun\n",adc1,adc2);

	value = adc1*510*1000/(2*adc2-adc1)/1000;

	DEBUGMSG("atmega_io : GET MODULE TYPE resistor value %lu\n",value);	
	return copy_to_user( module, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmegag7_io_getModuleType( void )
{
	unsigned long value;
	unsigned long adc1 = 0, adc2 = 0;
	int i;

	atmega_read_value( ATMEGA_I2C_ADC1_REG, (unsigned char*)&adc1, ATMEGA_I2C_ADC1_SIZE );
	atmega_read_value( ATMEGA_I2C_ADC2_REG, (unsigned char*)&adc2, ATMEGA_I2C_ADC2_SIZE );

// 	DEBUGMSG("atmega_io : GET MODULE TYPE adc1:%lu adc2:%lun\n",adc1,adc2);

	value = adc1*510*1000/(2*adc2-adc1)/1000;

	for ( i = 0; i < sizeof( module_resistors ) / sizeof( struct str_module_resistors ); i++ ) {
		if (( value < module_resistors[i].max_resistor ) && value > module_resistors[i].min_resistor) {
			return module_resistors[i].module_id;
			DEBUGMSG("atmega_io : GET MODULE TYPE resistor value %lu, modules number %d\n",value,i);	
			break;
		}
	}
	return 0;
}

static int atmegag7_io_getUsbType( void )
{
	unsigned long status = 0;
	unsigned usb_type = NO_USB_PLUG;

	atmega_read_value( ATMEGA_I2C_STATUS_REG, (unsigned char*)&status, ATMEGA_I2C_STATUS_SIZE );
	DEBUGMSG( "Status %lx\n", status );

	if ( status & ATMEGA_IO_STATUS_USB_LOCAL_DETECTED_BIT ) 
		usb_type |= USB_LOCAL;
	if ( status & ATMEGA_IO_STATUS_USB_EXT_DETECTED_BIT ) 
		usb_type |= USB_EXT;

	return usb_type;
}

static int atmegag7_io_battery_dock_check_dcin (void)
{
	int dc_in = 0;
	omap_cfg_reg(AF14_3430_GPIO184);
	mdelay(1);
	if ( omap_get_gpio_datain(184) ) {
		DEBUGMSG("not really got DC-in from battery dock.\n");
	} else {
		DEBUGMSG("really got DC-in from battery dock.\n");
		dc_in = 1;
	} 

	mdelay(1);
	omap_cfg_reg(AF14_3430_I2C3_SCL);
	return dc_in;
}

static void atmega_io_battery_dock_reget_battery (void)
{
	if ( atmegag7_io_battery_dock_check_dcin() ) {
		archos_needs_battery( 1,1 );
	}
	else {
		archos_needs_battery( 1,0 );
	}
}

static int atmega_io_battery_dock_read_DC_connect( unsigned long arg )
{
	void __user *dcin = (void __user *)arg;
	unsigned long value;

//DEBUGMSG("atmega_io_read_DC_connect\n");
	value = atmegag7_io_battery_dock_check_dcin();
	return copy_to_user( dcin, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_clean_eeprom(void)
{
	int i;
	
	for (i=0; i < 16; i++) {
		char blank_table[16];
		int ret;
		int x;

		memset(blank_table, 0, sizeof(blank_table));

		DEBUGMSG("%s: erasing table %i\n", __FUNCTION__, i);

		if ((ret = atmega_write_eeprom(blank_table, i*sizeof(blank_table), sizeof(blank_table))) < 0)
			return ret;

		if ((ret = atmega_read_eeprom(blank_table, i*sizeof(blank_table), sizeof(blank_table))) < 0)
			return ret;
		
		for (x=0; x < sizeof(blank_table); x++) {
			if (blank_table[x]) {
				printk(KERN_ERR "%s: table[%d][%d] not blank.\n", __FUNCTION__, i, x);
				return -EIO;
			}
		}
	}

	DEBUGMSG("%s: atmega eeprom cleaned.\n", __FUNCTION__);

	return 0;
}

static int atmega_io_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
	struct atmega_status *priv_status = file->private_data;
	struct atmega_io *io = atmega_io;
	struct rtc_time tm;
	int ret;
	long diff; 

	if (!io)
		return -ENODEV;

	//printk( "atmega_io : atmega_io_ioctl cmd->%d\n", cmd );
	
	switch( cmd ) {
	case ATMEGA_IO_CLEAN_EEPROM:
		DEBUGMSG( "atmega_io : ATMEGA_IO_CLEAN_EEPROM\n" );
		return atmega_io_clean_eeprom();

	case ATMEGA_IO_GET_SECURE_DIFF:
		diff = atmega_rtc_get_diff();
		DEBUGMSG( "atmega_io : ATMEGA_IO_GET_SECURE_DIFF\n" );
		return copy_to_user( (void*)arg, &diff, sizeof(diff) ) ? -EFAULT : 0;

	case ATMEGA_IO_READ_SECURE_TIME:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_SECURETIME\n" );
		ret = atmega_io_get_clockval(&tm, SECURE_TIME);
		if (ret < 0)
			return ret;
		return copy_to_user((void*)arg, &tm, sizeof(tm)) ? -EFAULT : 0;

	case ATMEGA_IO_SET_SECURE_TIME:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_SECURETIME\n" );
		if (copy_from_user(&tm, (void*)arg, sizeof(tm)))
			return -EFAULT;		
		return atmega_io_set_clockval(&tm, SECURE_TIME);

	case ATMEGA_IO_READ_GPS_TIME:
		DEBUGMSG("atmega_io: ATMEGA_IO_READ_GPS_TIME\n");
		ret = atmega_io_get_clockval(&tm, GPS_TIME);
		if (ret < 0)
			return ret;
		return copy_to_user((void*)arg, &tm, sizeof(tm)) ? -EFAULT : 0;

	case ATMEGA_IO_SET_GPS_TIME:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_GPS_TIME\n" );
		if (copy_from_user(&tm, (void*)arg, sizeof(tm)))
			return -EFAULT;		
		return atmega_io_set_clockval(&tm, GPS_TIME);

#if 0
	case ATMEGA_IO_READ_ALARM:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_ALARM\n" );
		return atmega_io_getalarm( arg );

	case ATMEGA_IO_SET_ALARM:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_ALARM\n" );
		return atmega_io_setalarm( arg );
#endif
		
	case ATMEGA_IO_READ_VERSION:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_VERSION\n" );
		return atmega_io_getversion( arg );

	case ATMEGA_IO_READ_STATUS:
 		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_STATUS\n" );
		return atmega_io_getstatus( priv_status, arg );

	case ATMEGA_IO_READ_VBATT:
 		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_VBATT\n" );
		return atmega_io_getvbatt( io, arg );

	case ATMEGA_IO_READ_MODULE_TYPE:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_MODULE_TYPE\n" );
		return atmega_io_getmodule( arg );

	case ATMEGA_IO_RESET_OMAP:
		DEBUGMSG( "atmega_io : ATMEGA_IO_RESET_OMAP\n" );
		return atmega_io_reset_omap();

	case ATMEGA_IO_SET_SHUTDOWN:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_SHUTDOWN\n" );
		return atmega_io_set_shutdown();

	case ATMEGA_IO_SET_PLUG_POWER:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_PLUG_POWER\n" );
		return atmega_io_set_plug_power( arg );

	case ATMEGA_IO_SET_LED_CHARGE:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_LED_CHARGE\n" );
		return atmega_io_set_led_charge( arg );

	case ATMEGA_IO_SET_LED_CHARGE_OFF:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_LED_CHARGE\n" );
		return atmega_io_set_led_charge_off( arg );

	case ATMEGA_IO_SET_CHARGE_MODE:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_CHARGE_MODE\n" );
		return atmegag7_io_set_charge_mode( arg );

	case ATMEGA_IO_TOGGLE_IRR_IO:
		DEBUGMSG( "atmega_io : ATMEGA_IO_TOGGLE_IRR_IO\n" );
		DEBUGMSG("ATMEGA_IO_TOGGLE_IRR_IO: %ld\n", arg);
#if defined(CONFIG_INPUT_AV600_IRR) || defined(CONFIG_DAVINCI_IRBLASTER)
		platform_enable_irr_io( arg );
#endif
		return 0;

	case ATMEGA_DUMP:
		atmega_io_dump(arg);
		return 0;

	case ATMEGA_IO_READ_BOARD_ID:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_BOARD_ID\n" );
		atmega_io_getboardID(arg);
		return 0;

	case ATMEGA_IO_READ_CHECK_SUM:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_CHECK_SUM\n" );
		atmega_io_getchecksum(arg);
		return 0;

	case ATMEGA_IO_BATTERYDOCK_REGET_BATTERY:
		DEBUGMSG( "atmega_io : ATMEGA_IO_BATTERYDOCK_REGET_BATTERY\n" );
		atmega_io_battery_dock_reget_battery();
		return 0;

	case ATMEGA_IO_BATTERYDOCK_READ_DC_CONNECT:
		DEBUGMSG( "atmega_io : ATMEGA_IO_BATTERYDOCK_READ_DC_CONNECT\n" );
		atmega_io_battery_dock_read_DC_connect(arg);
		return 0;
	default:
		return -EINVAL;
	}
}

static int atmega_io_open( struct inode *inode, struct file *file )
{
	struct atmega_status *priv_status;
	struct atmega_io *io = atmega_io;

	if (!io)
		return -ENODEV;

	priv_status = kzalloc(sizeof(struct atmega_status), GFP_KERNEL);
	if (!priv_status)
		return -ENOMEM;
	
	INIT_LIST_HEAD(&priv_status->lh);
	list_add_tail(&priv_status->lh, &io->clients_lh);

	// Initialize IO status
	atmega_read_value( ATMEGA_I2C_STATUS_REG, (unsigned char*)&priv_status->status, ATMEGA_I2C_STATUS_SIZE );
	priv_status->status_available = 1;
	
	/*archos_needs_battery(!!(io->status & 
			(ATMEGA_IO_STATUS_USB_EXT_DETECTED_BIT | ATMEGA_IO_STATUS_USB_LOCAL_DETECTED_BIT)),
			io->status & ATMEGA_IO_STATUS_DC_DETECTED_BIT);*/

	io->atmega_open_counter++;
	DEBUGMSG( "atmega_io : open[%s] count %d\n", current->comm, io->atmega_open_counter );

	file->private_data = priv_status;
	return 0;
}

static int atmega_io_release( struct inode *inode, struct file *file )
{
	struct atmega_status *priv_status = file->private_data;
	struct atmega_io *io = atmega_io;

	if (!io)
		return -ENODEV;

	io->atmega_open_counter--;
	list_del(&priv_status->lh);
	kfree(priv_status);
	file->private_data = NULL;
	return 0;
}

/* only for blocking read from the status queue. The get status ioctl flushs the queue */
static ssize_t atmega_io_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	struct atmega_io *io = atmega_io;
	struct atmega_status *priv_status = file->private_data;
	int len = 0;
	
	if (!io)
		return -ENODEV;
		
	DEBUGMSG("[%s]\n", current->comm);
	
	if (count >= 4) {
		if (copy_to_user(buffer, &priv_status->status, sizeof(unsigned long)))
			return -EFAULT;		
		len = 4;
	}
	
	priv_status->status_available = 0;

	return len;
}

static unsigned int atmega_io_poll(struct file * file, poll_table * wait)
{
	struct atmega_io *io = atmega_io;
	struct atmega_status *priv_status = file->private_data;
	unsigned int mask = 0;
	
	if (!io)
		return -ENODEV;
	
	if (priv_status->status_available)
		mask |= POLLIN | POLLRDNORM;
	
	poll_wait(file, &io->wait_queue, wait);

	if (priv_status->status_available)
		mask |= POLLIN | POLLRDNORM;
	
	return mask;
}

static ssize_t store_charge_rate(struct device *dev, struct device_attribute *attr, const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	atmegag7_io_set_charge_mode( on_off );
	return len;
}

static DEVICE_ATTR(charge_rate, S_IWUSR, NULL, store_charge_rate);

/*
 * switch class handlers
 */
static ssize_t usb_switch_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", sdev->name);
}

static ssize_t usb_switch_print_state(struct switch_dev *sdev, char *buf)
{
	ssize_t buflen = 0;

	switch (sdev->state) {
	case USB_UNKNOWN:
	default:
		buflen = sprintf(buf, "UNKNOWN\n");
		break;
	case USB_ATTACHED:
		buflen = sprintf(buf, "ATTACHED\n");
		break;
	case USB_DETACHED:
		buflen = sprintf(buf, "DETACHED\n");
		break;
	}
	return buflen;
}

static ssize_t accessory_switch_print_name(struct switch_dev *sdev, char *buf)
{
	int buflen = 0;
	
	switch (sdev->state) {
	default:
	case MODULE_ID_NONE:
		buflen = sprintf(buf, "UNKNOWN\n");
		break;
	case MODULE_ID_BATTERY_DOCK_GEN6:
		buflen = sprintf(buf, "BATTERY_DOCK\n");
		break;
	case MODULE_ID_DVBT_SNAP_ON:
		buflen = sprintf(buf, "TV_SNAP_ON\n");
		break;
	case MODULE_ID_GPS_WITHOUT_TMC:
		buflen = sprintf(buf, "GPS_WITHOUT_TMC\n");
		break;
	case MODULE_ID_GPS_WITH_TMC:
		buflen = sprintf(buf, "GPS_WITH_TMC\n");
		break;
	case MODULE_ID_HDMI_DOCK:
		buflen = sprintf(buf, "HDMI_DOCK\n");
		break;
	case MODULE_ID_MINI_DOCK:
		buflen = sprintf(buf, "MINI_DOCK\n");
		break;
	case MODULE_ID_MUSIC_DOCK:
		buflen = sprintf(buf, "MUSIC_DOCK\n");
		break;
	case MODULE_ID_PC_CABLE:
		buflen = sprintf(buf, "PC_CABLE\n");
		break;
	case MODULE_ID_POWER_CABLE:
		buflen = sprintf(buf, "CAR_POWER_CABLE\n");
		break;
	case MODULE_ID_REMOTE_FM:
		buflen = sprintf(buf, "REMOTE_FM\n");
		break;
	case MODULE_ID_SERIAL_ADAPTER:
		buflen = sprintf(buf, "SERIAL_ADAPTER\n");
		break;
	case MODULE_ID_TV_CRADLE:
		buflen = sprintf(buf, "DVR_STATION\n");
		break;
	case MODULE_ID_USB_HOST_CABLE:
		buflen = sprintf(buf, "USB_HOST_CABLE\n");
		break;
	case MODULE_ID_VRA_GEN6:
		buflen = sprintf(buf, "DVR_SNAP_ON\n");
		break;
	}
	return buflen;
}

static ssize_t accessory_switch_print_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%i\n", sdev->state);
}

/*
 *	The various file operations we support.
 */

static struct file_operations atmega_io_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= atmega_io_ioctl,
	.open		= atmega_io_open,
	.release	= atmega_io_release,
	.read		= atmega_io_read,
	.poll		= atmega_io_poll,
};
static struct miscdevice atmega_io_dev =
{
	.minor		= ATMEGA_IO_MINOR,
	.name		= "atmg",
	.fops		= &atmega_io_fops,
};


static int atmega_key_probe(struct atmega_io *io)
{
	int error;

	DEBUGMSG("atmega_key_probe \n" );

	io->input = input_allocate_device();
	if (!io->input) {
		return -ENOMEM;
	}
	
	io->input->evbit[0] = BIT(EV_KEY) | BIT(EV_REP);

	io->input->name = ATMEGA_KB_DEV_NAME;
	io->input->phys = "atmega-key/input0";

	input_set_capability(io->input, EV_KEY, KEY_POWER);

	error = input_register_device(io->input);
	if (error) {
		printk(KERN_ERR "Unable to register atmega-key input device\n");
		input_free_device(io->input);
		return error;
	}

	return 0;
}

static int atmegag7_io_probe( struct platform_device *pdev )
{
	struct device *dev = &pdev->dev;
	int ret;
	
	struct atmega_io *io = kzalloc(sizeof(struct atmega_io), GFP_KERNEL);
	if (!io)
		return -ENOMEM;
	
	platform_set_drvdata(pdev, io);

	/* find the IRQ */
	io->irq = platform_get_irq( pdev, 0 );
	if ( io->irq <= 0 ) {
		dev_err( &pdev->dev, "no irq for atmega status\n" );
		ret = -ENOENT;
		goto fail;
	}

	dev_dbg(&pdev->dev, "status irq %d on gpio %d\n", 
			io->irq, irq_to_gpio(io->irq) );
	
	init_waitqueue_head(&io->wait_queue);
	INIT_LIST_HEAD(&io->clients_lh);
	
	/* Create the workqueue */
	io->workqueue = create_singlethread_workqueue("atmega-io");
	if (!io->workqueue) {
		ret = -ENOMEM;
		goto fail;
	}

	if ((ret = atmega_key_probe(io)) < 0) {
		printk(KERN_ERR "atmega input device probe error!\n");
		goto fail2;
	}
		
	atmega_io_ops_init( atmegag7_io_getUsbType,
				atmegag7_io_getModuleType,
				atmegag7_io_set_charge_mode,
				atmegag7_io_battery_dock_check_dcin);
	
	io->version = 0;
	atmega_read_value( ATMEGA_I2C_VERSION_REG, (unsigned char*)&io->version, ATMEGA_I2C_VERSION_SIZE );
	DEBUGMSG("ATMEGA No.%lX\n",io->version );
	
	wake_lock_init(&io->wake_lock, WAKE_LOCK_SUSPEND, "atmega-g7");

	/* initialize switches */
	io->usb_switch.name = DRIVER_CALL;
	io->usb_switch.print_name = usb_switch_print_name;
	io->usb_switch.print_state = usb_switch_print_state;
	if (switch_dev_register(&io->usb_switch) < 0)
		printk(KERN_ERR "Error creating USB switch\n");

	io->accessory_switch.name = ACCESSORY_SWITCH_NAME;
	io->accessory_switch.print_name = accessory_switch_print_name;
	io->accessory_switch.print_state = accessory_switch_print_state;
	if (switch_dev_register(&io->accessory_switch) < 0)
		printk(KERN_ERR "Error creating Accessory switch\n");

	atmega_io = io;
	atmega_io_handle_status(io);
	
	if (io->status & ATMEGA_IO_STATUS_UC_COLD_STARTED_BIT) {
		const long time_diffs[3] = {0, 0, 0};
		
		dev_info(&pdev->dev, "cold start detected, erasing user_time_diff\n");

		// wait for 1 second to allow atmega to settle down a bit
		// msleep(1000);
		
		if (atmega_write_eeprom((unsigned char*)time_diffs, 
				0, sizeof(time_diffs)) < 0)
			dev_err(&pdev->dev, "ERROR: failed to erase user_time_diff\n");

		atmega_rtc_set_diff(0);
	}

	
	INIT_WORK(&io->work, atmega_io_worker);

	if ( request_irq( io->irq, atmega_io_statusirq, IRQF_SHARED|IRQF_TRIGGER_RISING, "atmega-io", pdev) ) {
		DEBUGMSG( "IRQ%d already in use\n", io->irq );
		ret = -EBUSY;
		goto fail3;
	}

	/* register our misc device */
	if ((ret = misc_register(&atmega_io_dev)) != 0) {
		printk(KERN_ERR "wdt: cannot register miscdev on minor=%d (err=%d)\n",
			ATMEGA_IO_MINOR, ret);
		goto fail4;
	}

	if ((ret = device_create_file( dev, &dev_attr_charge_rate)) < 0) {
		printk(KERN_DEBUG "unable to create charge_rate attribute\n");
		goto fail5;
	}

	/* reset Alarm to be sure ATMega does not have unwanted alarm */
	atmega_io_setalarm(0);

	return 0;

fail5:
	misc_deregister( &atmega_io_dev );
fail4:
	free_irq(io->irq, NULL);
fail3:
	input_unregister_device(io->input);
	switch_dev_unregister(&io->accessory_switch);
	switch_dev_unregister(&io->usb_switch);
fail2:
	destroy_workqueue(io->workqueue);
fail:
	platform_set_drvdata(pdev, NULL);
	atmega_io = NULL;
	kfree(io);
	return ret;
}

static int atmegag7_io_remove( struct platform_device *pdev )
{
	struct device *dev = &pdev->dev;
	struct atmega_io *io = platform_get_drvdata(pdev);

	device_remove_file( dev, &dev_attr_charge_rate );
	free_irq( io->irq, NULL );
	misc_deregister( &atmega_io_dev );
	input_unregister_device(io->input);
	switch_dev_unregister(&io->usb_switch);
	switch_dev_unregister(&io->accessory_switch);
	
	platform_set_drvdata(pdev, NULL);
	kfree(io);
	return 0;
}

#ifdef CONFIG_PM

static int atmegag7_io_suspend( struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int atmegag7_io_resume( struct platform_device *pdev )
{
	return 0;
}
#endif

static struct platform_driver atmegag7_io_drv = {
	.probe		= atmegag7_io_probe,
	.remove		= atmegag7_io_remove,
#ifdef CONFIG_PM
	.suspend	= atmegag7_io_suspend,
	.resume		= atmegag7_io_resume,
#endif
	.driver	= {
		.name = "atmegag7-io"
	},
};

/* ++++++++++++++++++ Module Init +++++++++++++++++++++++*/

static int __init atmegag7_io_init( void )
{
	int ret;
	
	ret = platform_driver_register( &atmegag7_io_drv );
	if ( ret < 0 )
		return ret;

	DEBUGMSG( "ATMEGA G7 IO Driver register success\n" );
	return 0;
}

static void __exit atmegag7_io_exit( void )
{
	platform_driver_unregister( &atmegag7_io_drv );
} 

module_init( atmegag7_io_init );
module_exit( atmegag7_io_exit );

/* Module information */
MODULE_DESCRIPTION("ATMEGA IO Driver for G7X");
MODULE_LICENSE("GPL");
