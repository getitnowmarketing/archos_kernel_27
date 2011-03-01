#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <mach/archos-battery.h>
#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/archos-gpio.h>
#include <mach/board-archos.h>

#include <mach/archos_supervisor.h>

#define DBG if (1)

static int charge_state;
static int charge_level;

static int _usb;
static int _dcin;
static int _usb_level;
static struct archos_gpio charge_gpio;

static struct archos_sv_client *sv_client_battery;

static void update_charge_state(void);

static void set_stop_chg(int on_off)
{
	if ( GPIO_PIN( charge_gpio) != 0)
		omap_set_gpio_dataout( GPIO_PIN( charge_gpio ), !!on_off );
}

#ifdef CONFIG_PM
static int battery_suspend(struct sys_device* dev, pm_message_t state)
{
	set_stop_chg(1);
	return 0;
}

static int battery_resume(struct sys_device* dev)
{
	set_stop_chg(!charge_state);
	return 0;
}
#endif

static struct sysdev_class omap_battery_sysclass = {
	.name		= "archos-battery",
#ifdef CONFIG_PM
	.suspend	= battery_suspend,
	.resume		= battery_resume,
#endif
};

static struct sys_device omap_battery0_device = {
	.id		= 0,
	.cls		= &omap_battery_sysclass,
};

static ssize_t show_battery0_charge_state(struct sys_device* dev, struct sysdev_attribute *attr, char* buf)
{
	return sprintf(buf, "%d\n", _usb_level); 
}

static ssize_t show_battery0_charge_level(struct sys_device* dev, struct sysdev_attribute *attr, char* buf)
{
	return sprintf(buf, "%d\n", charge_level); 
}

static SYSDEV_ATTR(battery0_charge_state, S_IRUGO|S_IWUSR, show_battery0_charge_state, NULL);
static SYSDEV_ATTR(battery0_charge_level, S_IRUGO|S_IWUSR, show_battery0_charge_level, NULL);

static void enable_high_charging(void)
{
#if defined (CONFIG_MACH_ARCHOS_G6) && !defined (CONFIG_MACH_OMAP_3430SDP)
	if (!machine_is_archos_g6tv()) {
		set_stop_chg(0);
		atmega_io_set_charge_mode(CHARGER_ON_HIGH);
	}
#endif

	charge_level=2;
	charge_state=1;
	
	kobject_uevent( &omap_battery0_device.kobj, KOBJ_CHANGE);
}

static void enable_low_charging(void)
{
#if defined (CONFIG_MACH_ARCHOS_G6) && !defined (CONFIG_MACH_OMAP_3430SDP)
	if (!machine_is_archos_g6tv()) {
		set_stop_chg(1);
		atmega_io_set_charge_mode(CHARGER_OFF);
		atmega_io_set_charge_mode(CHARGER_ON_LOW);
		set_stop_chg(0);
	}
#endif

	charge_level=1;
	charge_state=1;
	kobject_uevent( &omap_battery0_device.kobj, KOBJ_CHANGE);
}

static void disable_charging(void)
{
	
#if defined (CONFIG_MACH_ARCHOS_G6) && !defined (CONFIG_MACH_OMAP_3430SDP)
	if (!machine_is_archos_g6tv()) {
		set_stop_chg(1);
		atmega_io_set_charge_mode(CHARGER_OFF);
	}
#endif

	charge_state = 0;
	charge_level = 0;
	kobject_uevent( &omap_battery0_device.kobj, KOBJ_CHANGE);
}

static int is_module_capable_high_charging( unsigned long usb_type )
{	
	switch (usb_type) {
		// List of modules supporting high charging
		case MODULE_ID_TV_CRADLE:
		case MODULE_ID_VRA_GEN6:
		case MODULE_ID_BATTERY_DOCK_GEN6:
		case MODULE_ID_MUSIC_DOCK:
		case MODULE_ID_HDMI_DOCK:
		case MODULE_ID_MINI_DOCK:
		case MODULE_ID_DVBT_SNAP_ON:
		case MODULE_ID_SERIAL_ADAPTER:
		case MODULE_ID_GPS_WITHOUT_TMC:
		case MODULE_ID_GPS_WITH_TMC:
		case MODULE_ID_POWER_CABLE:
			return 1;

		default:
			break;
	}
	return 0;
}

// verification at the USB side for autorisation of the the charge
static int usb_charge_level(void)
{
	return _usb_level;
}

static void update_charge_state(void)
{
	int usb_type = 0;

	if( _usb && !_dcin ){
		// usb without DC-in 
		// no charge with only usb & can't happen when plugging the PC cable
		disable_charging();
		return;
	}

	if( _dcin && !_usb ){
		// only DC-in non usb; always high charging
#ifdef CONFIG_MACH_ARCHOS_G6L
		if ( machine_is_archos_g6l()) 
			enable_high_charging();
		else 
#endif
		{
			// ! warning ! this case should appears with pc_cable during plug or with
			// not complete connection, don't enable high charge in this case
			// enable high charge only if an allowed module is identified
			usb_type = archos_sv_get_usb_type(sv_client_battery);
DBG 			printk("usb_type=%d\n",usb_type);

			if ( is_module_capable_high_charging(usb_type) ) {
				enable_high_charging();
			}
		}
		return;
	}

	if( _usb && _dcin ){
		// Means normally that both the USB cable and DC-in are plugged.
		// But be careful that both flags are also enabled when plugging the PC cable
		usb_type = archos_sv_get_usb_type(sv_client_battery);
DBG 		printk("usb_type=%d\n",usb_type);

		if ( is_module_capable_high_charging(usb_type) ) {
			enable_high_charging();
		} else {
			if ( usb_charge_level() ) {
				enable_high_charging();
			} else {
				enable_low_charging();
			}
		}	
		return;
	}

	if( charge_state && ( !_usb && !_dcin )){
		disable_charging();	
		return;
	}
}

/* when usb_need is not 0 or 1, we keep it's value */

void archos_needs_battery(int usb_need, int dcin_need)
{
DBG 	printk("archos_needs_battery usb: %d dcin:%d\n",usb_need,dcin_need);

	if ( usb_need == 1 || usb_need == 0 )
		_usb = usb_need;

	if ( dcin_need == 1 || dcin_need == 0 )
		_dcin = dcin_need;

// printk("[BATTERY], usb: %d, dc_in: %d\n",_usb,_dcin);

	_usb_level = 0;


DBG 	printk("[BATTERY], usb: %d, dc_in: %d\n",_usb,_dcin);

	update_charge_state();
}

void archos_usb_high_charge(int charge)
{
	_usb_level = charge;

DBG 	printk("[BATTERY], usb: %d, dc_in: %d _usb_level: %d\n",_usb,_dcin, _usb_level);

	update_charge_state();
}

void archos_sv_client_battery_event(struct archos_sv_client *sv_client, struct archos_sv_event *ev)
{
printk("%s\n", __FUNCTION__);
	switch(ev->id)
	{
		case ARCHOS_SV_EVENT_STATUS_CHANGED:
			break;
		
		default:
			break;
	}
}

static int __init omap_battery_init_devicefs(void)
{
	const struct archos_charge_config *charge_cfg;
	int ret;

	sysdev_class_register(&omap_battery_sysclass);
	
	ret = sysdev_register(&omap_battery0_device);
	if (ret < 0)
		return ret;

	sysdev_create_file(&omap_battery0_device, &attr_battery0_charge_state);
	sysdev_create_file(&omap_battery0_device, &attr_battery0_charge_level);

	/* charge pin */
	charge_cfg = omap_get_config( ARCHOS_TAG_CHARGE, struct archos_charge_config );
	if (charge_cfg && (hardware_rev < charge_cfg->nrev)) {
		charge_gpio = charge_cfg->rev[hardware_rev];
		GPIO_INIT_OUTPUT( charge_gpio );
		omap_set_gpio_dataout( GPIO_PIN( charge_gpio ), 1);	
	} else
		printk(KERN_DEBUG "archosg6_batt_init: no board configuration found\n");
	
	return 0;
}

//arch_initcall(omap_battery_init_devicefs);

static int archos_sv_client_battery_probe(struct archos_sv_client *sv_client)
{
printk("archos_sv_client_battery_probe\n");
	sv_client_battery = sv_client;
	
	return 0;
}

static int archos_sv_client_battery_remove(struct archos_sv_client *sv_client)
{	
	sv_client_battery = NULL;
	return 0;
}

static struct archos_sv_client_driver sv_client_driver = {
	.name 	= "archos-battery",
	.probe 	= archos_sv_client_battery_probe,	
	.remove = archos_sv_client_battery_remove,
	.event 	= archos_sv_client_battery_event,	
	.drv		= {
		.name	= "archos-battery",
	},
};

static int __init archos_battery_init(void) {
	return archos_sv_register_client(&sv_client_driver);	
}

static void __exit archos_battery_exit(void) {
	archos_sv_unregister_client(&sv_client_driver);	
}

module_init( archos_battery_init );
module_exit( archos_battery_exit );

EXPORT_SYMBOL(archos_needs_battery);
EXPORT_SYMBOL(archos_usb_high_charge);
