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

#include <mach/atmega-io.h>
#include <mach/atmega.h>
#include <mach/archos-battery.h>

#include <mach/board.h>
#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/archos-gpio.h>

#define ATMEGA_IO_VERSION	"0.04"

#define ATMEGA_IO_IS_OPEN	0x01	/* means /dev/atmg is in use */

#define ATMEGA_KB_DEV_NAME "atmega-key"

#define ATMEGA_IO_IS_OPEN	0x01	/* means /dev/atmg is in use */

#define ATMEGA_KB_DEV_NAME "atmega-key"

#define ATMEGA_IO_READ_TIME			0
#define ATMEGA_IO_READ_VERSION			1
#define ATMEGA_IO_READ_STATUS			2
#define ATMEGA_IO_READ_VBATT			3
#define ATMEGA_IO_READ_ALARM			4
#define ATMEGA_IO_READ_MODULE_TYPE		5
#define ATMEGA_IO_READ_DC_CONNECT		6		// new gen5
#define ATMEGA_DUMP				7		// ATMEGA Dump all registers
#define ATMEGA_IO_GET_CURRENT_SAT		8		
#define ATMEGA_IO_GET_NEXT_SAT			9		

#define ATMEGA_IO_SET_TIME			10
#define ATMEGA_IO_SET_ALARM			11
#define ATMEGA_IO_SET_POWER_OFF			12
#define ATMEGA_IO_RESET_ALARM			13
#define ATMEGA_IO_RESET_OMAP			14
#define ATMEGA_IO_SET_SHUTDOWN			15
#define ATMEGA_IO_SET_PLUG_POWER		16
#define ATMEGA_IO_SET_LED_CHARGE		17
#define ATMEGA_IO_SET_CHARGE_MODE		18		// new gen5
#define ATMEGA_IO_CLR_SD_FLAG			19		// new gen5

#define ATMEGA_IO_TOGGLE_IRR_IO			20
#define ATMEGA_IO_SET_LED_CHARGE_OFF		21
// #define ATMEGA_IO_SET_GREEN_BLINK		22
#define ATMEGA_IO_SET_PTH			23		// new gen5
#define ATMEGA_IO_SET_LED_CHG			24		// for AMP
#define ATMEGA_IO_SET_LED_PWR			25		// for AMP
#define ATMEGA_IO_READ_LED_CHG_STATUS		26		// for AMP
#define ATMEGA_IO_READ_LED_PWR_STATUS		27		// for AMP
#define ATMEGA_IO_SET_REBOOT			28		// dvr205
#define ATMEGA_IO_CLR_COLDSTART			29		// dvr205
#define ATMEGA_IO_READ_BOARD_ID			30		// new gen6  G06S:60vv G06H: 61vv
#define ATMEGA_IO_READ_CHECK_SUM		31	

#define ATMEGA_IO_BATTERYDOCK_REGET_BATTERY	33		// new gen6  check the real DC-in status for battery dock and set the charge level
#define ATMEGA_IO_BATTERYDOCK_READ_DC_CONNECT	34		// new gen6  check the real DC-in status for battery dock

// ATMEGA STATUS register bit mask
#define ATMEGA_IO_STATUS_DC_DETECTED_BIT	0x0001
#define ATMEGA_IO_STATUS_USB_DETECTED_BIT	0x0002
#define ATMEGA_IO_STATUS_BATTERY_OK_BIT		0x0004
#define ATMEGA_IO_STATUS_DDRAM_COLD_STARTED_BIT	0x0008
#define ATMEGA_IO_STATUS_MSP_COLD_STARTED_BIT	0x0010
#define ATMEGA_IO_STATUS_AUX_DETECTED_BIT	0x0020
#define ATMEGA_IO_STATUS_ALARM_REACHED_BIT	0x0040
#define ATMEGA_IO_STATUS_REQ_POWER_OFF_BIT	0x0080

#define ATMEGA_IO_STATUS_CHARGE_HIGH_BIT	0x0100		// new gen5
#define ATMEGA_IO_STATUS_CHARGE_ON_BIT		0x0200		// new gen5
#define ATMEGA_IO_STATUS_SD_CHANGED_BIT		0x0400		// new gen5
#define ATMEGA_IO_STATUS_KBON_PRESSED_BIT	0x0800		// new gen6
#define ATMEGA_IO_STATUS_DCJACK_DETECT_BIT	0x1000		// new gen6  for G06L, valid from ATMEGA version 0x0b

#define ATMEGA_IO_STATUS_RED_BLINK_BIT		0x0100		// dvr205
#define ATMEGA_IO_STATUS_GREEN_BLINK_BIT	0x0200		// dvr205

static struct atmega_io {
	unsigned int open_status;
	unsigned long status;
	unsigned long vbatt;
	int status_available;
	int irq;
	unsigned long version;
	struct workqueue_struct *workqueue;
	struct work_struct work;
	wait_queue_head_t wait_queue;
	struct input_dev* input;
} *atmega_io;

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

#undef DEBUG
#ifdef DEBUG
#define DEBUGMSG(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DEBUGMSG( x... )
#endif

extern void platform_enable_irr_io(int en);
extern int platform_get_dock_dc(struct device* dev);

static int atmegag6_io_getModuleType( void );


/* IRQ Handler */
 
static void atmega_io_getstatus_on_irq(struct atmega_io *io)
{
	unsigned long status = 0;
	int aux_change = 0;

	atmega_read_value( ATMEGA_I2C_STATUS_REG, (unsigned char*)&status, ATMEGA_I2C_STATUS_SIZE );
	DEBUGMSG( "Status %lx\n", status );

	if ( io->version == 0 ) {
		atmega_read_value( ATMEGA_I2C_VERSION_REG, (unsigned char*)&io->version, ATMEGA_I2C_VERSION_SIZE );
		DEBUGMSG("ATMEGA No.%lX\n",io->version );
	}

#ifdef CONFIG_MACH_ARCHOS_G6TV
	if ( !machine_is_archos_g6tv() ) 
#endif	
	{

		if ( io->version >=0x060a ) {
			if ( !(io->status & ATMEGA_IO_STATUS_KBON_PRESSED_BIT) ) {
				if (status & ATMEGA_IO_STATUS_KBON_PRESSED_BIT) {
					printk( "ATMEGA KBON pressed\n");
					input_event(io->input, EV_KEY, KEY_POWER, 1);
					input_sync(io->input);
				}
			} else {		
				if (!(status & ATMEGA_IO_STATUS_KBON_PRESSED_BIT)) {
					printk( "ATMEGA KBON released\n");
					input_event(io->input, EV_KEY, KEY_POWER, 0);
					input_sync(io->input);
				}
			}
		}

		if ( (io->status & ATMEGA_IO_STATUS_AUX_DETECTED_BIT) != (status & ATMEGA_IO_STATUS_AUX_DETECTED_BIT))
			aux_change = 1;

		if ( ( io->version <= 0x060a ) 
#ifdef CONFIG_MACH_ARCHOS_G6L
				|| ( !machine_is_archos_g6l() && io->version >= 0x060b ) 
#endif		
				) {
			if ( ( aux_change == 1 ) 
				|| ( ( io->status & ATMEGA_IO_STATUS_USB_DETECTED_BIT) != (status & ATMEGA_IO_STATUS_USB_DETECTED_BIT)) 
				|| ( (io->status & ATMEGA_IO_STATUS_DC_DETECTED_BIT) != (status & ATMEGA_IO_STATUS_DC_DETECTED_BIT) ) )	{
				archos_needs_battery( !!(status & ATMEGA_IO_STATUS_USB_DETECTED_BIT), status & ATMEGA_IO_STATUS_DC_DETECTED_BIT );
			}
		} else if ( ( aux_change == 1 ) || ( io->status & ( ATMEGA_IO_STATUS_DCJACK_DETECT_BIT | ATMEGA_IO_STATUS_DC_DETECTED_BIT) ) 
				!= ( status & ( ATMEGA_IO_STATUS_DCJACK_DETECT_BIT | ATMEGA_IO_STATUS_DC_DETECTED_BIT)) ) { 
			
			if ( (status & ATMEGA_IO_STATUS_DCJACK_DETECT_BIT) ) {
				printk( "ATMEGA DCJACK detected\n");
				archos_needs_battery( 0, 1 );
			} else if ( (status & ATMEGA_IO_STATUS_DC_DETECTED_BIT) && ( atmegag6_io_getModuleType() == MODULE_ID_TV_CRADLE ) ){
				printk( "ATMEGA G6L TV_cradle detected\n");
				archos_needs_battery( 0, 1 );
			} else {
 				archos_needs_battery( 0, 0 );
			}
		}
	}

	if ( status & ATMEGA_IO_STATUS_ALARM_REACHED_BIT ){
		printk( "ATMEGA Alarm reached\n");
	} 

	io->status = status;
	io->status_available = 1;
	wake_up_interruptible(&io->wait_queue);
}

static void atmega_io_worker(struct work_struct *work)
{
	struct atmega_io *io = container_of(work, struct atmega_io, work);
	atmega_io_getstatus_on_irq(io);
}

/* ----- Utility functions --------------------------------------------	*/

static irqreturn_t atmega_io_statusirq( int irq, void *dev_id )
{
	struct platform_device *pdev = dev_id;
	struct atmega_io *io = platform_get_drvdata(pdev);

	DEBUGMSG("atmega_io_statusirq\n");
	queue_work( io->workqueue, &io->work );

	return IRQ_HANDLED;
}

/* Register read/write */
static int atmega_io_gettime( struct rtc_time * arg )
{
	struct rtc_time *rtc_tm = arg;
	unsigned long nowtime = 0;

	int res = atmega_read_value( ATMEGA_I2C_TIME_REG, (unsigned char*)&nowtime, ATMEGA_I2C_TIME_SIZE );
	DEBUGMSG("rtc_time_tp_tm,%lu\n",nowtime);
	rtc_time_to_tm( nowtime, rtc_tm );
	DEBUGMSG("isdst: %d day: %d wday: %d year: %d mon: %d mday; %d hour: %d min: %d sec: %d\t\n",
		rtc_tm->tm_isdst,rtc_tm->tm_yday,rtc_tm->tm_wday,rtc_tm->tm_year,rtc_tm->tm_mon,rtc_tm->tm_mday,rtc_tm->tm_hour,rtc_tm->tm_min,rtc_tm->tm_sec);
	return res;
}

static int atmega_io_settime( struct rtc_time * arg )
{
	struct rtc_time *rtc_tm = arg;
	unsigned long nowtime;
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_SET_RTC, 0,0,0 };
	rtc_tm_to_time( rtc_tm, &nowtime);
	table.value = nowtime;
	return ( atmega_write_table( &table ));
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
		atmega_read_value(i, &data, 1);
		printk(KERN_INFO "atmg exchangtable[%02X]: %02x\n", i, data);
	}
	return 0;
}

static int atmega_io_getalarm( unsigned long arg )
{
	struct rtc_wkalrm *rtc_alr = (struct rtc_wkalrm *)arg;
	unsigned long nowtime = 0;

	int res = atmega_read_value( ATMEGA_I2C_ALARM_REG, (unsigned char*)&nowtime, ATMEGA_I2C_ALARM_SIZE );

	if ( nowtime ) {
		struct rtc_time *alr_tm = &rtc_alr->time;

		rtc_alr->enabled = 1;
		rtc_time_to_tm( nowtime, alr_tm );
	} else {
		rtc_alr->enabled = 0;
	}
	
	return res;
}

static int atmega_io_setalarm( unsigned long arg )
{
	struct rtc_wkalrm *rtc_alr = (struct rtc_wkalrm *)arg;
	struct atmega_exchange_table table = { 0,0,0,0,0 };

	if ( rtc_alr->enabled ) {
		unsigned long nowtime;
		struct rtc_time *alr_tm = &rtc_alr->time;

		rtc_tm_to_time( alr_tm, &nowtime);
		table.value = nowtime;
		table.control_byte = ATMEGA_I2C_CTRL_CMD_SET_ALARM;
	} else {
		table.control_byte = ATMEGA_I2C_CTRL_CMD_RESET_ALARM;
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

	struct atmega_exchange_table table = { 1, ATMEGA_I2C_CTRL_CMD_CHG_LED_TOGGLE, 0,0,0 };

	DEBUGMSG(KERN_DEBUG "Set charge led: arg %lx, p1:%d(%04x), p2:%d(%04x)\n",arg,led_cmd->period,led_cmd->period,led_cmd->blink,led_cmd->blink);
	table.p1 = led_cmd->period;
	table.p2 = led_cmd->blink;

	return ( atmega_write_table( &table ) );
}

static int atmega_io_set_led_charge_off( unsigned long arg )
{
	struct atmega_exchange_table table = { 1, ATMEGA_I2C_CTRL_CMD_CHG_LED_TOGGLE, 10,0,0 };
	return ( atmega_write_table( &table ) );
}

static int atmegag6_io_set_charge_mode( unsigned long arg )
{
	struct atmega_exchange_table table = { 0, ATMEGA_I2C_CTRL_CMD_CHARGEMODE, 0,0,0 };
	table.value = arg;

	return ( atmega_write_table( &table ) );
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
static int atmega_io_getstatus( struct atmega_io *io, unsigned long arg )
{
	void __user *status = (void __user *)arg;
	unsigned long value = 0;

	value = io->status;
	return copy_to_user( status, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_getvbatt( struct atmega_io *io, unsigned long arg )
{
	void __user *vbatt = (void __user *)arg;
	unsigned long adc0 = 0, adc3 = 0, value;

	atmega_read_value( ATMEGA_I2C_ADC0_REG, (unsigned char*)&adc0, ATMEGA_I2C_ADC0_SIZE );
	atmega_read_value( ATMEGA_I2C_ADC3_REG, (unsigned char*)&adc3, ATMEGA_I2C_ADC3_SIZE );
	
// 	DEBUGMSG("atmega_io : adc0:%lu adc3:%lu\n",adc0,adc3);
// 	value =  adc0*11*43047/1024/100;
// 	value_ref = adc3*11*430/1024;
	
	if ( adc3 == 0 ) {
		value = io->vbatt;
		printk(KERN_DEBUG "atmega_io : getVbatt ADC3 is 0 \n");
	} else {
		value = adc0 * 2500 / adc3;
		io->vbatt = value;
	}
// 	DEBUGMSG("atmega_io : atmega_io_getvbatt: %lu vbatt reference: %lu\n",value, value_ref);
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

static int atmegag6_io_getModuleType( void )
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

static int atmegag6_io_battery_dock_check_dcin (void)
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
	if ( atmegag6_io_battery_dock_check_dcin() ) {
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
	value = atmegag6_io_battery_dock_check_dcin();
	return copy_to_user( dcin, &value, sizeof(value) ) ? -EFAULT : 0;
}

static int atmega_io_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
	struct atmega_io *io = atmega_io;
	if (!io)
		return -ENODEV;

// 	DEBUGMSG( "atmega_io : atmega_io_ioctl cmd->%d\n", cmd );


	switch( cmd ) {
	case ATMEGA_IO_READ_TIME:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_TIME\n" );
		return atmega_io_gettime( (struct rtc_time *)arg );

	case ATMEGA_IO_READ_ALARM:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_ALARM\n" );
		return atmega_io_getalarm( arg );

	case ATMEGA_IO_READ_VERSION:
		DEBUGMSG(KERN_DEBUG "atmega_io : ATMEGA_IO_READ_VERSION\n" );
		return atmega_io_getversion( arg );

	case ATMEGA_IO_READ_STATUS:
// 		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_STATUS\n" );
		return atmega_io_getstatus( io, arg );

	case ATMEGA_IO_READ_VBATT:
// 		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_VBATT\n" );
		return atmega_io_getvbatt( io, arg );

	case ATMEGA_IO_READ_MODULE_TYPE:
		DEBUGMSG( "atmega_io : ATMEGA_IO_READ_MODULE_TYPE\n" );
		return atmega_io_getmodule( arg );

	case ATMEGA_IO_SET_TIME:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_TIME\n" );
		return atmega_io_settime( (struct rtc_time *)arg );

	case ATMEGA_IO_SET_ALARM:
		DEBUGMSG( "atmega_io : ATMEGA_IO_SET_ALARM\n" );
		return atmega_io_setalarm( arg );

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
		return atmegag6_io_set_charge_mode( arg );

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
	struct atmega_io *io = atmega_io;
	if (!io)
		return -ENODEV;

	if ( io->open_status & ATMEGA_IO_IS_OPEN )
		return -EBUSY;

	// Initialize IO status
	io->status = 0;
	atmega_read_value( ATMEGA_I2C_STATUS_REG, (unsigned char*)&io->status, ATMEGA_I2C_STATUS_SIZE );

#ifdef CONFIG_MACH_ARCHOS_G6TV
	if (machine_is_archos_g6tv()) {
		// no charging on TV
	} else
#endif
	{
		archos_needs_battery(!!(io->status & ATMEGA_IO_STATUS_USB_DETECTED_BIT),io->status & ATMEGA_IO_STATUS_DC_DETECTED_BIT);
	}

	io->open_status |= ATMEGA_IO_IS_OPEN;

	return 0;
}

static int atmega_io_release( struct inode *inode, struct file *file )
{
	struct atmega_io *io = atmega_io;
	if (!io)
		return -ENODEV;

	io->open_status &= ~ATMEGA_IO_IS_OPEN;
	return 0;
}

/* only for blocking read from the status queue. The get status ioctl flushs the queue */
static ssize_t atmega_io_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	struct atmega_io *io = atmega_io;
	int len = 0;
	if (!io)
		return -ENODEV;
		
	if (count > 0) {
		*buffer = io->status;
		len = 1;
	}
	
	io->status_available = 0;

	return len;
}

static unsigned int atmega_io_poll(struct file * file, poll_table * wait)
{
	struct atmega_io *io = atmega_io;
	unsigned int mask = 0;
	if (!io)
		return -ENODEV;
	
	if (io->status_available)
		mask |= POLLIN | POLLRDNORM;
	
	poll_wait(file, &io->wait_queue, wait);

	return mask;
}

static ssize_t store_charge_rate(struct device *dev, struct device_attribute *attr, const char* buf, size_t len)
{
	//struct atmega_io *io = dev_get_drvdata(dev);
	int on_off = simple_strtol(buf, NULL, 10);	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	atmegag6_io_set_charge_mode( on_off );
	return len;
}

static DEVICE_ATTR(charge_rate, S_IWUSR, NULL, store_charge_rate);

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

static int atmegag6_io_probe( struct platform_device *pdev )
{
	struct device *dev = &pdev->dev;
	int ret;
	struct atmega_io *io = kzalloc(sizeof(struct atmega_io), GFP_KERNEL);
	if (!io) {
		return -ENOMEM;
	}

printk("%s\n", __FUNCTION__);
	platform_set_drvdata(pdev, io);

	/* find the IRQ */
	io->irq = platform_get_irq( pdev, 0 );
	if ( io->irq <= 0 ) {
		dev_err( &pdev->dev, "no irq for atmega status\n" );
		ret = -ENOENT;
		goto fail;
	}

	DEBUGMSG("atmega_io: status irq %d on gpio %d\n", io->irq, irq_to_gpio(io->irq) );

	init_waitqueue_head(&io->wait_queue);

	/* Create the workqueue */
	io->workqueue = create_singlethread_workqueue("atmega-io");
	if (!io->workqueue) {
		ret = -ENOMEM;
		goto fail;
	}

	INIT_WORK(&io->work, atmega_io_worker);

	if ( request_irq( io->irq, atmega_io_statusirq, IRQF_SHARED|IRQF_TRIGGER_RISING, "atmega-io", pdev) ) {
		DEBUGMSG( "IRQ%d already in use\n", io->irq );
		ret = -EBUSY;
		goto fail2;
	}

	/* register our misc device */
	if ((ret = misc_register(&atmega_io_dev)) != 0) {
		printk(KERN_ERR "wdt: cannot register miscdev on minor=%d (err=%d)\n",
			MISC_DYNAMIC_MINOR, ret);
		goto fail3;
	}

// 	if ( io->version >= 0x060a ) {
		// or (hardware_rev = 3 and msp versoin >=9)
		if ((ret = atmega_key_probe(io)) < 0) {
			printk(KERN_ERR "atmega input device probe error!\n");
			goto fail4;
		}
// 	}
		
	if ((ret = device_create_file( dev, &dev_attr_charge_rate)) < 0) {
		printk(KERN_DEBUG "unable to create charge_rate attribute\n");
		goto fail5;
	}
	
	atmega_io_ops_init(NULL, atmegag6_io_getModuleType,
				atmegag6_io_set_charge_mode,
				atmegag6_io_battery_dock_check_dcin);
	
	atmega_io = io;
	return 0;

fail5:
	input_unregister_device(io->input);
fail4:
	misc_deregister( &atmega_io_dev );
fail3:
	free_irq(io->irq, NULL);
fail2:
	destroy_workqueue(io->workqueue);
fail:
	platform_set_drvdata(pdev, NULL);
	atmega_io = NULL;
	kfree(io);
	return ret;
}

static int atmegag6_io_remove( struct platform_device *pdev )
{
	struct device *dev = &pdev->dev;
	struct atmega_io *io = platform_get_drvdata(pdev);

	device_remove_file( dev, &dev_attr_charge_rate );
	free_irq( io->irq, NULL );

	misc_deregister( &atmega_io_dev );
	input_unregister_device(io->input);

	platform_set_drvdata(pdev, NULL);
	kfree(io);
	return 0;
}

#ifdef CONFIG_PM
static int atmegag6_io_suspend( struct platform_device *pdev, pm_message_t msg)
{
	struct atmega_io *io = platform_get_drvdata(pdev);
	disable_irq( io->irq );
	return 0;
}

static int atmegag6_io_resume( struct platform_device *pdev )
{
	struct atmega_io *io = platform_get_drvdata(pdev);
	enable_irq( io->irq );
	// refresh status wake up event
	atmega_io_getstatus_on_irq(io);
	return 0;
}
#endif

static struct platform_driver atmegag6_io_drv = {
	.probe		= atmegag6_io_probe,
	.remove		= atmegag6_io_remove,
#ifdef CONFIG_PM
	.suspend	= atmegag6_io_suspend,
	.resume		= atmegag6_io_resume,
#endif
	.driver	= {
		.name = "atmegag6-io"
	},
};

/* ++++++++++++++++++ Module Init +++++++++++++++++++++++*/

static int __init atmegag6_io_init( void )
{
	int ret;
	
	printk( "ATMEGA G6 IO Driver v%s for G6X\n", ATMEGA_IO_VERSION );

	ret = platform_driver_register( &atmegag6_io_drv );
	if ( ret < 0 ) {
		return ret;
	}
	
	DEBUGMSG( "ATMEGA G6 IO Driver register success\n" );
	return 0;

}

static void __exit atmegag6_io_exit( void )
{
	platform_driver_unregister( &atmegag6_io_drv );
} 

module_init( atmegag6_io_init );
module_exit( atmegag6_io_exit );

/* Module information */
MODULE_DESCRIPTION("ATMEGA IO Driver for G6X");
MODULE_LICENSE("GPL");
