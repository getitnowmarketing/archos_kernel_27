/*
 * PWM Generator (on IRBlaster Pin)
 *
 */

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <asm/mach-types.h>

#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/archos-gpio.h>
#include <mach/board-archos.h>
#include <mach/dmtimer.h>

static int duty_cycle = 0;
static int frequency = 0;

static int irb_state = 0;
static struct omap_dm_timer *irb_pwm_timer;
static struct archos_pwm_conf irblaster_pwm; // = { .timer = 8, .mux_cfg = N8_3430_GPT8 };

static void pwm_set_speed(struct omap_dm_timer *gpt,
		int frequency, int duty_cycle)
{
	u32 val;
	u32 period;
	struct clk *timer_fclk;

	/* and you will have an overflow in 1 sec         */
	/* so,                              */
	/* freq_timer     -> 1s             */
	/* carrier_period -> 1/carrier_freq */
	/* => carrier_period = freq_timer/carrier_freq */

	timer_fclk = omap_dm_timer_get_fclk(gpt);
	period = clk_get_rate(timer_fclk) / frequency;

	val = 0xFFFFFFFF+1-period;
	omap_dm_timer_set_load(gpt, 1, val);

	val = 0xFFFFFFFF+1-(period*duty_cycle/256);
	omap_dm_timer_set_match(gpt, 1, val);

	/* assume overflow first: no toogle if first trig is match */
	omap_dm_timer_write_counter(gpt, 0xFFFFFFFE);
}


static ssize_t show_irblaster_freq(struct device *dev, 
		struct device_attribute *attr, char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", frequency); 
}

static ssize_t store_irblaster_freq(struct device *dev, struct device_attribute *attr, 
		const char* buf, size_t len)
{
	frequency = simple_strtol(buf, NULL, 10);

	/* Disable pwm */
	if (frequency == 0) {
		omap_dm_timer_stop(irb_pwm_timer);
		omap_dm_timer_disable(irb_pwm_timer);
		irb_state = 0;
		return 0;
	}

	/* Enable pwm */
	if (irb_state == 0) {
		omap_dm_timer_enable(irb_pwm_timer);
		omap_dm_timer_set_pwm(irb_pwm_timer, 0, 1, 2);
		omap_dm_timer_start(irb_pwm_timer);
		irb_state = 1;
	}

	pwm_set_speed(irb_pwm_timer, frequency, duty_cycle);
	
	return len;
}

static ssize_t show_irblaster_duty(struct device *dev, 
		struct device_attribute *attr, char* buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", duty_cycle); 
}

static ssize_t store_irblaster_duty(struct device *dev, struct device_attribute *attr, 
		const char* buf, size_t len)
{
	duty_cycle = simple_strtol(buf, NULL, 10);
	if (irb_state) pwm_set_speed(irb_pwm_timer, frequency, duty_cycle);
	
	return len;
}

static DEVICE_ATTR(frequency, S_IRUGO|S_IWUSR, show_irblaster_freq, store_irblaster_freq);
static DEVICE_ATTR(duty_cycle, S_IRUGO|S_IWUSR, show_irblaster_duty, store_irblaster_duty);


static struct platform_device irblaster_pwm_device = {
	.name = "irblaster_pwm",
	.id = -1,
};

static int __init archos_irblaster_pwn_init(void)
{
	int ret;

	const struct archos_irblaster_config *irblaster_cfg;
	irblaster_cfg = omap_get_config( ARCHOS_TAG_IRBLASTER, struct archos_irblaster_config );
	if (irblaster_cfg == NULL) {
		printk(KERN_DEBUG "archos_irblaster_pwn_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= irblaster_cfg->nrev ) {
		printk(KERN_DEBUG "archos_irblaster_pwn_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, irblaster_cfg->nrev);
		return -ENODEV;
	}

	irblaster_pwm = irblaster_cfg->rev[hardware_rev].irblaster_pwm;

	irb_pwm_timer = omap_dm_timer_request_specific(irblaster_pwm.timer);
	if (irb_pwm_timer) {
		omap_dm_timer_set_source(irb_pwm_timer, OMAP_TIMER_SRC_SYS_CLK);
		omap_cfg_reg(irblaster_pwm.mux_cfg);
	} else
		pr_err("archos_irblaster_pwn_init: no irblaster PWM\n");


	ret = platform_device_register(&irblaster_pwm_device);
	if (!ret) {
		device_create_file(&irblaster_pwm_device.dev, &dev_attr_frequency);	
		device_create_file(&irblaster_pwm_device.dev, &dev_attr_duty_cycle);
	}


	return ret;
}

late_initcall(archos_irblaster_pwn_init);
