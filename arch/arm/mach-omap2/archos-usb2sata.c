#include <linux/types.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <mach/gpio.h>
#include <mach/archos-gpio.h>
#include <mach/prcm.h>
#include <mach/mux.h>
#include <mach/board-archos.h>

#define DELAY_500GB_SEAGATE

static struct archos_gpio gpio_sata_pwron = UNUSED_GPIO;
static struct archos_gpio gpio_hdd_pwron = UNUSED_GPIO;
static struct archos_gpio gpio_sata_rdy = UNUSED_GPIO;

static int satavcc = -1;
static struct clk *clkout1;

extern int archos_enable_ehci( int enable );

static struct platform_device usb2sata_device = {
  .name = "usb2sata",
  .id   = -1, 
};

/* export usbsata_power to drivers for power management */
void usbsata_power(int on_off)
{
	if (satavcc == on_off)
		return;

	printk("usbsata_power %i\n", on_off);
	satavcc = on_off;

	if (on_off) {
		omap_set_gpio_dataout(GPIO_PIN(gpio_hdd_pwron), on_off);
		msleep(100);
#ifdef DELAY_500GB_SEAGATE
		msleep(400);
#endif
		omap_set_gpio_dataout(GPIO_PIN(gpio_sata_pwron), on_off);
#ifdef DELAY_500GB_SEAGATE
		msleep(500);
#endif
		//clk_enable(clkout1);
		//archos_enable_ehci( 1 );
	} else {
		omap_set_gpio_dataout(GPIO_PIN(gpio_hdd_pwron), on_off);
		omap_set_gpio_dataout(GPIO_PIN(gpio_sata_pwron), on_off);
		/* wait another 100ms to propagate the disconnect through
		 * the phy, then switch if off */
		//msleep(100);
		//archos_enable_ehci( 0 );
		//clk_disable(clkout1);
	}		
}
EXPORT_SYMBOL(usbsata_power);

void usbsata_controller_power( int on_off)
{
	printk("usbsata_power %i\n", on_off);
	omap_set_gpio_dataout(GPIO_PIN(gpio_sata_pwron), on_off);
}
EXPORT_SYMBOL(usbsata_controller_power);

static ssize_t show_usb2sata_satardy(struct device* dev, 
    struct device_attribute *attr, char* buf)
{
	int satardy = omap_get_gpio_datain(GPIO_PIN(gpio_sata_rdy));
	return snprintf(buf, PAGE_SIZE, "%d\n", satardy); 
}

static ssize_t show_usb2sata_satavcc(struct device* dev,
    struct device_attribute *attr, char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", satavcc); 
}

static ssize_t set_usb2sata_satavcc(struct device* dev,
    struct device_attribute *attr, const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	usbsata_power(on_off);

	return len;
}

static DEVICE_ATTR(satardy, S_IRUGO, show_usb2sata_satardy, NULL);
static DEVICE_ATTR(satavcc, S_IRUGO|S_IWUSR, show_usb2sata_satavcc, set_usb2sata_satavcc);

int __init  archos_usb2sata_init(void) {
	int ret;
	const struct archos_sata_config *sata_cfg;
	sata_cfg = omap_get_config( ARCHOS_TAG_SATA, struct archos_sata_config );
	if (sata_cfg == NULL) {
		printk(KERN_DEBUG "archos_sata_init: no board configuration found\n");
		return -ENODEV;
	}
	if (hardware_rev >= sata_cfg->nrev) {
		printk(KERN_DEBUG "archos_sata_init: hardware_rev (%i) >= nrev (%i)\n",
				hardware_rev, sata_cfg->nrev);
		return -ENODEV;
	}

	printk(KERN_DEBUG "archos_usb2sata_init\n");

	clkout1 = clk_get(NULL, "sys_clkout1");
	if (IS_ERR(clkout1)) {
		printk(KERN_ERR "clk_get(sys_clkout1) failed\n");
		return PTR_ERR(clkout1);
	}

	/* sysfs setup */
	ret = platform_device_register(&usb2sata_device);
	if (ret)
		return ret;

	/* SATA bridge */
	gpio_sata_pwron = sata_cfg->rev[hardware_rev].sata_power;
	
	ret = device_create_file(&usb2sata_device.dev, &dev_attr_satavcc);
	if (ret == 0) {
		printk(KERN_DEBUG "archos_usb2sata_init: sata_pwron on GPIO%i\n",
				GPIO_PIN(gpio_sata_pwron));
		GPIO_INIT_OUTPUT(gpio_sata_pwron);
		omap_set_gpio_dataout(GPIO_PIN(gpio_sata_pwron), 0);
	}

	/* HDD power switch */
	gpio_hdd_pwron = sata_cfg->rev[hardware_rev].hdd_power;
	printk(KERN_DEBUG "archos_usb2sata_init: sata_pwron on GPIO%i\n",
			GPIO_PIN(gpio_hdd_pwron));
	GPIO_INIT_OUTPUT(gpio_hdd_pwron);
	omap_set_gpio_dataout(GPIO_PIN(gpio_hdd_pwron), 0);

	/* SATA_RDY signal */
	gpio_sata_rdy = sata_cfg->rev[hardware_rev].sata_ready;
	ret = device_create_file(&usb2sata_device.dev, &dev_attr_satardy);
	if (ret == 0) {
		printk(KERN_DEBUG "archos_usb2sata_init: sata_ready on GPIO%i\n",
				GPIO_PIN(gpio_sata_rdy));
		GPIO_INIT_INPUT(gpio_sata_rdy);
	}

	clk_enable(clkout1);
	usbsata_power(0);

	return 0;
}
