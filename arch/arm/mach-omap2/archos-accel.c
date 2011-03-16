/*
 * MMA7456L Accelerometer board configuration
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mma7456l.h>

#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/archos-gpio.h>
#include <asm/mach-types.h>
#include <mach/board.h>
#include <mach/board-archos.h>

static struct archos_accel_conf accel_gpio;


int __init archos_accel_init(struct mma7456l_pdata *pdata)
{
	const struct archos_accel_config *accel_cfg;
	
	accel_cfg = omap_get_config( ARCHOS_TAG_ACCEL, struct archos_accel_config );
	if (accel_cfg == NULL) {
		printk(KERN_DEBUG "archos_accel_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= accel_cfg->nrev ) {
		printk(KERN_DEBUG "archos_accel_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, accel_cfg->nrev);
		return -ENODEV;
	}

	accel_gpio = accel_cfg->rev[hardware_rev];

	/* irq needed by the driver */
	pdata->irq1 = gpio_to_irq(GPIO_PIN( accel_gpio.accel_int1 ));
	pdata->irq2 = gpio_to_irq(GPIO_PIN( accel_gpio.accel_int2 ));
	printk("archos_accel_init: irq1 %d, irq2 %d\n",pdata->irq1,pdata->irq2);

	GPIO_INIT_INPUT( accel_gpio.accel_int1 );
	GPIO_INIT_INPUT( accel_gpio.accel_int2 );
	omap_set_gpio_debounce(GPIO_PIN( accel_gpio.accel_int1 ),1);
	omap_set_gpio_debounce(GPIO_PIN( accel_gpio.accel_int2 ),1);

	return 0;
}
