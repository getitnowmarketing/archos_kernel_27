#include <linux/kernel.h>
#include <mach/atmega-io.h>

static struct atmega_io_ops ops;

int atmega_io_battery_dock_check_dcin (void)
{
	if (!ops.battery_dock_check_dcin) {
		printk("%s: Error! Function callback not set\n", __FUNCTION__);
		return -1;
	}
	
	return ops.battery_dock_check_dcin();
}

int atmega_io_set_charge_mode( unsigned long arg )
{
	if (!ops.set_charge_mode) {
		printk("%s: Error! Function callback not set\n", __FUNCTION__);
		return -1;
	}
	
	return ops.set_charge_mode(arg);
}

int atmega_io_getUsbType( void )
{
	if (!ops.get_usb_type) {
		printk("%s: Error! Function callback not set\n", __FUNCTION__);
		return -1;
	}
	
	return ops.get_usb_type();
}

int atmega_io_getModuleType( void )
{
	if (!ops.get_module_type) {
		printk("%s: Error! Function callback not set\n", __FUNCTION__);
		return -1;
	}
	
	return ops.get_module_type();
}

void atmega_io_ops_init(int (*get_usb_type)(void),
			int (*get_module_type)(void),
			int (*set_charge_mode)( unsigned long arg ),
			int (*battery_dock_check_dcin) (void))
{
	ops.get_usb_type = get_usb_type;	
	ops.get_module_type = get_module_type;	
	ops.set_charge_mode = set_charge_mode;	
	ops.battery_dock_check_dcin = battery_dock_check_dcin;	
}
