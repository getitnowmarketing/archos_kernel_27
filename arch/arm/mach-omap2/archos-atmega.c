#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>

#include <mach/board-archos.h>

static struct resource atmegag7_io_resources = {
	.flags		= IORESOURCE_IRQ,
};

static struct platform_device atmegag7_io_device = {
	.id		= -1,
	.num_resources	= 1,
	.resource	= &atmegag7_io_resources,
};

static struct platform_device atmega_rtc_device = {
	.name = "atmega-rtc",
	.id = -1,
	.num_resources = 1,
	.resource = &atmegag7_io_resources,
};

int __init archos_atmega_init(void)
{
	const struct archos_atmega_config *atmega_config;
	
	atmega_config = omap_get_config( ARCHOS_TAG_ATMEGA, struct archos_atmega_config );
	if (atmega_config == NULL) {
		printk(KERN_DEBUG "archos_atmega_init: no board configuration found\n");
		return -ENODEV;
	}

	GPIO_INIT_INPUT( atmega_config->irq );

	atmegag7_io_device.name 		= atmega_config->name;
	atmegag7_io_device.resource[0].start 	= OMAP_GPIO_IRQ(GPIO_PIN(atmega_config->irq));
	atmegag7_io_device.resource[0].end 	= OMAP_GPIO_IRQ(GPIO_PIN(atmega_config->irq));
	
	atmega_rtc_device.resource[0].start 	= OMAP_GPIO_IRQ(GPIO_PIN(atmega_config->irq));
	atmega_rtc_device.resource[0].end 	= OMAP_GPIO_IRQ(GPIO_PIN(atmega_config->irq));

	platform_device_register(&atmegag7_io_device);
	platform_device_register(&atmega_rtc_device);
	
	return 0;
}
