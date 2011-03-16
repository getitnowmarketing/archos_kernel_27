#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/board-archos.h>
#include <mach/archos-gpio.h>

 struct archos_gpio wake_up_omap = {
 	.nb = 0,
 	.mux_cfg = AF26_3430_GPIO0, 
};

int __init archosg7_init(void)
{
 	GPIO_INIT_OUTPUT(wake_up_omap);
 	omap_set_gpio_dataout( GPIO_PIN( wake_up_omap ), 0 );

	return 0;
}
