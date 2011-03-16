/*
 * IRRemote Board configuration
 *
 */

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <asm/mach-types.h>
#include <linux/module.h>

#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/archos-gpio.h>
#include <mach/board-archos.h>
#include <mach/dmtimer.h>
#include <mach/irremote.h>

/* We will use platform_data to pass the timer to the driver */
struct irremote_data plat_data = {
	.timer = -1,
};

static struct archos_pwm_conf irremote_timer;

struct platform_device irremote_device = {
	.name		= "archos-irremote",
	.id		= -1,
	.dev		= {
		.platform_data	= &plat_data,
	},
};

static int __init archos_irremote_init(void)
{
	const struct archos_irremote_config *irremote_cfg;

	irremote_cfg = omap_get_config( ARCHOS_TAG_IRREMOTE, struct archos_irremote_config );
	if (irremote_cfg == NULL) {
		printk(KERN_DEBUG "archos_irremote_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= irremote_cfg->nrev ) {
		printk(KERN_DEBUG "archos_irremote_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, irremote_cfg->nrev);
		return -ENODEV;
	}

	irremote_timer = irremote_cfg->rev[hardware_rev].irremote_timer;
		
	if (irremote_timer.timer == -1)
		return -ENODEV;
		
	plat_data.timer = irremote_timer.timer;

	return platform_device_register(&irremote_device);
}

arch_initcall(archos_irremote_init);
