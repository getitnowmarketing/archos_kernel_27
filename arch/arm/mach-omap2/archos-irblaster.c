/*
 * IRBlaster Board configuration
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
#include <mach/irblaster.h>

static struct archos_pwm_conf irblaster_pwm;
static struct archos_pwm_conf irblaster_ctrl_timer;

struct omap_pwm_timer gpt_pwm_list[] = {
	[ IRBLASTER_PWM ] = {
			.name = "irblaster pwm",
			.source = OMAP_TIMER_SRC_SYS_CLK,
	}, 
	[ IRBLASTER_TIMER_CTRL ] = {
			.name = "irblaster control",
			.source = OMAP_TIMER_SRC_SYS_CLK,
	},
};
EXPORT_SYMBOL(gpt_pwm_list);

struct platform_device irblaster_device = {
	.name		= "archos-irblaster",
	.id		= -1,
	.dev		= {
		.platform_data	= gpt_pwm_list, //&plat_data,
	},
};

static int __init archos_irblaster_init(void)
{
	const struct archos_irblaster_config *irblaster_cfg;

	irblaster_cfg = omap_get_config( ARCHOS_TAG_IRBLASTER, struct archos_irblaster_config );
	if (irblaster_cfg == NULL) {
		printk(KERN_DEBUG "archos_irblaster_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= irblaster_cfg->nrev ) {
		printk(KERN_DEBUG "archos_irblaster_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, irblaster_cfg->nrev);
		return -ENODEV;
	}

	irblaster_pwm = irblaster_cfg->rev[hardware_rev].irblaster_pwm;
	irblaster_ctrl_timer = irblaster_cfg->rev[hardware_rev].irblaster_ctrl_timer;

	if (irblaster_pwm.timer == -1 || irblaster_ctrl_timer.timer == -1)
		return -ENODEV;

	gpt_pwm_list[IRBLASTER_PWM].no = irblaster_pwm.timer;
	gpt_pwm_list[IRBLASTER_PWM].mux_config = irblaster_pwm.mux_cfg;
	gpt_pwm_list[IRBLASTER_TIMER_CTRL].no = irblaster_ctrl_timer.timer;
	gpt_pwm_list[IRBLASTER_TIMER_CTRL].mux_config = irblaster_ctrl_timer.mux_cfg;
	
	return platform_device_register(&irblaster_device);
}

arch_initcall(archos_irblaster_init);
