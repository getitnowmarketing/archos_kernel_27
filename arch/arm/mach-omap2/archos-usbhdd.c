#include <linux/platform_device.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/delay.h>

#include <asm/mach-types.h>

#include <mach/clock.h>
#include <mach/gpio.h>
#include <mach/archos-gpio.h>
#include <mach/prcm.h>
#include <mach/mux.h>
#include <mach/board-archos.h>

static struct archos_gpio gpio_hdd_pwron = UNUSED_GPIO;

static int hddvcc;

extern int archos_enable_ehci( int enable );

static struct platform_device usbhdd_device = {
	.name	= "usbhdd",
	.id 	= -1,
};

/* export usbhdd power switch to drivers */
void usbhdd_power(int on_off)
{
	if (hddvcc == on_off)
		return;
	printk( KERN_DEBUG "usbhdd_power %d\n", on_off);
	hddvcc = on_off;

	if (GPIO_PIN(gpio_hdd_pwron))
		omap_set_gpio_dataout( GPIO_PIN( gpio_hdd_pwron ), on_off );

}

EXPORT_SYMBOL(usbhdd_power);

static ssize_t show_usbhdd_hddvcc(struct device *dev,
	struct device_attribute *attr, char* buf)
{
	return sprintf(buf, "%d\n", hddvcc);
}

static ssize_t set_usbhdd_hddvcc(struct device *dev,
	struct device_attribute *attr, const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);

	if( on_off > 1 || on_off < 0 )
		return -EINVAL;

	usbhdd_power( on_off );

	return len;
}

static DEVICE_ATTR(hddvcc, S_IRUGO|S_IWUSR, show_usbhdd_hddvcc, set_usbhdd_hddvcc);

static int archos_usbhdd_probe(struct platform_device *dev)
{
	return 0;
}

static int archos_usbhdd_suspend(struct platform_device *dev, pm_message_t pm)
{
printk("archos_usbhdd_suspend\n");
	omap_set_gpio_dataout( GPIO_PIN( gpio_hdd_pwron ), 0 );
	return 0;
}

static int archos_usbhdd_resume(struct platform_device *dev)
{
printk("archos_usbhdd_resume\n");
	omap_set_gpio_dataout( GPIO_PIN( gpio_hdd_pwron ), hddvcc );
	return 0;
}


static struct platform_driver archos_usbhdd_driver = {
	.driver.name = "archos_usbhdd",
	.probe = archos_usbhdd_probe,
#ifdef CONFIG_PM
	.suspend_late = archos_usbhdd_suspend,
	.resume_early = archos_usbhdd_resume,
#endif
};

int __init archos_usbhdd_init(void)
{
	int ret;
	const struct archos_usbhdd_config *hdd_cfg;

	/* usb hdd */
	hdd_cfg = omap_get_config( ARCHOS_TAG_USBHDD, struct archos_usbhdd_config );
	if (hdd_cfg == NULL) {
		printk(KERN_DEBUG "archos_usbhdd_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= hdd_cfg->nrev ) {
		printk(KERN_DEBUG "archos_usbhdd_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, hdd_cfg->nrev);
		return -ENODEV;
	}

	/* sysfs setup */
	ret = platform_device_register(&usbhdd_device);
	if (ret)
		return ret;

	/* HDD power switch */
	gpio_hdd_pwron = hdd_cfg->rev[hardware_rev].hdd_power;
	if(GPIO_EXISTS(gpio_hdd_pwron)) {
		ret = device_create_file(&usbhdd_device.dev, &dev_attr_hddvcc);
		if (ret == 0) {
			printk(KERN_DEBUG "archos_usbhdd_init: hdd_pwr on GPIO%i\n",
				GPIO_PIN(gpio_hdd_pwron));
			GPIO_INIT_OUTPUT(gpio_hdd_pwron);
			omap_set_gpio_dataout(GPIO_PIN(gpio_hdd_pwron), 0);
		}
	}

	usbhdd_power( 0 );
	
	return platform_driver_register(&archos_usbhdd_driver);
}
