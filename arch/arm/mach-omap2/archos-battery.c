#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>

#include <mach/archos-battery.h>
#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/archos-gpio.h>
#include <mach/board-archos.h>

#include "mach/atmega-io.h"
#include "mach/atmega.h"

#define DBG if (1)

static int charge_state;
static int charge_level;

static int _usb;
static int _dcin;
static int _usb_level;
static struct archos_gpio charge_gpio = UNUSED_GPIO;

#ifdef CONFIG_POWER_SUPPLY
static struct power_supply main_battery;
#endif

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
	return sprintf(buf, "%d\n", charge_state); 
}

static ssize_t show_battery0_charge_level(struct sys_device* dev, struct sysdev_attribute *attr, char* buf)
{
	return sprintf(buf, "%d\n", charge_level); 
}

static ssize_t show_battery0_charge_usblevel(struct sys_device* dev, struct sysdev_attribute *attr, char* buf)
{
	return sprintf(buf, "%d\n", _usb_level); 
}

static SYSDEV_ATTR(battery0_charge_state,    S_IRUGO|S_IWUSR, show_battery0_charge_state,    NULL);
static SYSDEV_ATTR(battery0_charge_level,    S_IRUGO|S_IWUSR, show_battery0_charge_level,    NULL);
static SYSDEV_ATTR(battery0_charge_usblevel, S_IRUGO|S_IWUSR, show_battery0_charge_usblevel, NULL);

static void enable_high_charging(void)
{
DBG	printk("enable_high_charging\r\n");
	set_stop_chg(0);
	atmega_io_set_charge_mode(CHARGER_ON_HIGH);

	charge_level=2;
	charge_state=1;
	
	kobject_uevent( &omap_battery0_device.kobj, KOBJ_CHANGE);
}

static void enable_low_charging(void)
{
DBG	printk("enable_low_charging\r\n");
	set_stop_chg(1);
	atmega_io_set_charge_mode(CHARGER_OFF);
	set_stop_chg(0);
	atmega_io_set_charge_mode(CHARGER_ON_LOW);

	charge_level=1;
	charge_state=1;
	kobject_uevent( &omap_battery0_device.kobj, KOBJ_CHANGE);
}

static void disable_charging(void)
{
DBG	printk("disable_charging\r\n");
	set_stop_chg(1);
	atmega_io_set_charge_mode(CHARGER_OFF);

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
	int module_type = 0;
	int usb_type;

	if( _usb && !_dcin ) {
		// usb without DC-in
		usb_type = atmega_io_getUsbType();
DBG 		printk("usb_type=%d\n", usb_type);

		if (usb_type & USB_LOCAL) {
			if ( usb_charge_level() ) {
				enable_high_charging();
			} else {
				enable_low_charging();
			}
		} else if (usb_type == USB_EXT) {
			module_type = atmega_io_getModuleType();
DBG 			printk("module_type=%d\n", module_type);

			if ( module_type == MODULE_ID_SERIAL_ADAPTER ) {
				enable_low_charging();
			} else {
				disable_charging();
			}
		} else {
			// no charge with only ext usb & can't happen when plugging the PC cable
			disable_charging();
		}
		goto out;
	}

	if( _dcin && !_usb ){
		// ! warning ! this case should appears with pc_cable during plug or with
		// not complete connection, don't enable high charge in this case
		// enable high charge only if an allowed module is identified
		module_type = atmega_io_getModuleType();
DBG 		printk("module_type=%d\n", module_type);

		if ( is_module_capable_high_charging(module_type) ) {
			enable_high_charging();
		}
		goto out;
	}

	if( _usb && _dcin ){
		// Means normally that both the USB cable and DC-in are plugged.
		// But be careful that both flags are also enabled when plugging the PC cable
		module_type = atmega_io_getModuleType();
DBG 		printk("module_type=%d\n", module_type);

		if ( is_module_capable_high_charging(module_type) ) {
			enable_high_charging();
		} else {	// pc cable only or pc cable and micro usb
			usb_type = atmega_io_getUsbType();
DBG 			printk("usb_type=%d\n", usb_type);

			if (usb_type == USB_EXT) {		// pc cable only
				if ( usb_charge_level() ) {
					enable_high_charging();
				} else {
					enable_low_charging();
				}
			} else	// in this case, usb source to check is not the power source
				enable_low_charging();

		}	
		goto out;
	}

	if( charge_state && ( !_usb && !_dcin )){
		disable_charging();	
		goto out;
	}
	
 out:
#ifdef CONFIG_POWER_SUPPLY
	 power_supply_changed(&main_battery);
#endif
	 /* make compiler happy if CONFIG_POWER_SUPPLY is not defined */
	 return;
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

#ifdef CONFIG_POWER_SUPPLY
/* power supply abstraction for built-in battery */

static int current_mV = 3850;
static int current_capacity = 75;

static int archos_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	if (strcmp(psy->name, "ac") == 0) {
		if (psp == POWER_SUPPLY_PROP_ONLINE) {
			val->intval = _dcin;
			return 0;
		}
		return -EINVAL;
	}
	
	if (strcmp(psy->name, "usb") == 0) {
		if (psp == POWER_SUPPLY_PROP_ONLINE) {
			val->intval = _usb;
			return 0;
		}
		return -EINVAL;
	}
	
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!_dcin && !_usb) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		if (charge_state)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
		
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1; /* always present */
		break;
	
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = current_capacity;
		break;
		
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = current_mV;
		break;
		
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = current_mV;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 23; /* we have no sensor for that */
		break;
		
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
		
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4200;
		break;
		
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3500;
		break;
		
	default:
		return -EINVAL;
	}
	
	return 0;
}

static void archos_battery_external_power_changed(struct power_supply *psy)
{
}

static char *main_battery_supplied_to[] = {
	"battery",
};

static enum power_supply_property main_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
};

static struct power_supply main_battery = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = main_battery_properties,
	.num_properties = ARRAY_SIZE(main_battery_properties),
	.get_property = archos_battery_get_property,
	.external_power_changed = archos_battery_external_power_changed,
};

static enum power_supply_property ac_supply_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply ac_supply = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = ac_supply_properties,
	.num_properties = ARRAY_SIZE(ac_supply_properties),
	.get_property = archos_battery_get_property,
	.external_power_changed = archos_battery_external_power_changed,
};

static struct power_supply usb_supply = {
	.name = "usb",
	.supplied_to = main_battery_supplied_to,
	.num_supplicants = ARRAY_SIZE(main_battery_supplied_to),
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = ac_supply_properties,
	.num_properties = ARRAY_SIZE(ac_supply_properties),
	.get_property = archos_battery_get_property,
	.external_power_changed = archos_battery_external_power_changed,
};

static struct power_supply *archos_power_supplies[] = {
	&main_battery,
	&ac_supply,
	&usb_supply,
	NULL,
};

static ssize_t battery_show_voltage(struct device* dev, 
		    struct device_attribute *attr, char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%i", current_mV);
}
static ssize_t battery_store_voltage(struct device* dev,
		    struct device_attribute *attr, const char* buf, size_t len)
{
	/* struct power_supply **psy = dev_get_drvdata(dev); */
	
	int new_mV = simple_strtol(buf, NULL, 10);

	if (new_mV != current_mV) {
		current_mV = new_mV;
		/* update only on capacity changes!
		   power_supply_changed(psy[0]); */
	}
	return len;	
}
static DEVICE_ATTR(voltage_avg, S_IRUGO|S_IWUSR, battery_show_voltage, battery_store_voltage);

static ssize_t battery_show_capacity(struct device* dev, 
		    struct device_attribute *attr, char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%i", current_capacity);
}

static ssize_t battery_store_capacity(struct device* dev,
		    struct device_attribute *attr, const char* buf, size_t len)
{
	struct power_supply **psy = dev_get_drvdata(dev);
	
	int new_capacity = simple_strtol(buf, NULL, 10);

	if (new_capacity != current_capacity) {
		current_capacity = new_capacity;
		power_supply_changed(psy[0]);
	}
	return len;		
}
static DEVICE_ATTR(capacity, S_IRUGO|S_IWUSR, battery_show_capacity, battery_store_capacity);

static ssize_t ac_show_online(struct device* dev, 
		    struct device_attribute *attr, char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%i", _dcin);	
}
static ssize_t ac_store_online(struct device* dev,
		    struct device_attribute *attr, const char* buf, size_t len)
{
	return len;
}
static DEVICE_ATTR(ac_online, S_IRUGO|S_IWUSR, ac_show_online, ac_store_online);

static ssize_t usb_show_online(struct device* dev, 
		    struct device_attribute *attr, char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%i", _usb);
}
static ssize_t usb_store_online(struct device* dev,
		    struct device_attribute *attr, const char* buf, size_t len)
{
	return len;
}
static DEVICE_ATTR(usb_online, S_IRUGO|S_IWUSR, usb_show_online, usb_store_online);

static int archos_battery_probe(struct platform_device *pdev)
{
	struct power_supply **psy = pdev->dev.platform_data;
	int ret;
	int i = 0;
	
	platform_set_drvdata(pdev, psy);
	
	while (psy[i] != NULL) {
		ret = power_supply_register(&pdev->dev, psy[i]);
		if (ret < 0) {
			printk(KERN_DEBUG "archos_battery_probe: "
					"cannot register psy %s: %i\n", 
					psy[i]->name, ret);
		}
		i++;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_voltage_avg);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot create voltage_avg attribute\n");
	ret = device_create_file(&pdev->dev, &dev_attr_capacity);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot create capacity attribute\n");
	ret = device_create_file(&pdev->dev, &dev_attr_ac_online);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot create ac_online attribute\n");
	ret = device_create_file(&pdev->dev, &dev_attr_usb_online);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot create usb_online attribute\n");
	
	return 0;	
}

static struct platform_driver archos_battery_driver = {
	.probe = archos_battery_probe,
	.driver = {
		.name = "battery",
	},
};

static struct platform_device archos_battery_device = {
	.name = "battery",
	.id = -1,
	.dev.platform_data = &archos_power_supplies,
};

static int archos_battery_driver_init(void) {
	int ret;
	
	ret = platform_driver_register(&archos_battery_driver);
	if (ret < 0) {
		printk(KERN_INFO "omap_battery_init_devicefs: failed to register battery driver\n");
		return ret;
	}
	
	return 0;
}

device_initcall(archos_battery_driver_init);
#endif

static int __init omap_battery_init_devicefs(void)
{
	const struct archos_charge_config *charge_cfg;
	int ret;

	sysdev_class_register(&omap_battery_sysclass);
	
	ret = sysdev_register(&omap_battery0_device);
	if (ret < 0)
		return ret;

#ifdef CONFIG_POWER_SUPPLY
	ret = platform_device_register(&archos_battery_device);
	if (ret < 0) {
		printk(KERN_INFO "omap_battery_init_devicefs: failed to register battery device\n");
		return ret;
	}
#endif

	sysdev_create_file(&omap_battery0_device, &attr_battery0_charge_state);
	sysdev_create_file(&omap_battery0_device, &attr_battery0_charge_level);
	sysdev_create_file(&omap_battery0_device, &attr_battery0_charge_usblevel);

	/* charge pin */
	charge_cfg = omap_get_config( ARCHOS_TAG_CHARGE, struct archos_charge_config );
	if (charge_cfg && (hardware_rev < charge_cfg->nrev)) {
		charge_gpio = charge_cfg->rev[hardware_rev];
		GPIO_INIT_OUTPUT( charge_gpio );
		omap_set_gpio_dataout( GPIO_PIN( charge_gpio ), 1);	
	} else
		printk(KERN_DEBUG "omap_battery_init_devicefs: no board configuration found\n");
	
	return 0;
}

arch_initcall(omap_battery_init_devicefs);

EXPORT_SYMBOL(archos_needs_battery);
EXPORT_SYMBOL(archos_usb_high_charge);
