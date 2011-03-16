#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/archos-audio.h>
#include <mach/archos-gpio.h>
#include <asm/mach-types.h>
#include <mach/board.h>
#include <mach/board-archos.h>

static struct archos_audio_conf audio_gpio;

static void _set_ampli(int onoff)
{
	if ( onoff ==  PLAT_ON)
		omap_set_gpio_dataout( GPIO_PIN( audio_gpio.spdif), 0);
	else
		omap_set_gpio_dataout( GPIO_PIN( audio_gpio.spdif), 1);
}

static void _set_hp(int onoff)
{
	if ( onoff ==  PLAT_ON)
		omap_set_gpio_dataout( GPIO_PIN( audio_gpio.hp_on), 1);
	else
		omap_set_gpio_dataout( GPIO_PIN( audio_gpio.hp_on), 0);
}

static int _get_headphone_plugged(void)
{
	return omap_get_gpio_datain( GPIO_PIN( audio_gpio.headphone_plugged) );
}

static int _get_headphone_irq(void)
{
	return gpio_to_irq(GPIO_PIN( audio_gpio.headphone_plugged));
}

static void _sys_clkout1_en(int en)
{
	struct clk *sys_clkout1;

	sys_clkout1 = clk_get(NULL, "sys_clkout1");
	if (!IS_ERR(sys_clkout1)) {
		if (en == PLAT_ON) {
			if ( clk_enable(sys_clkout1) != 0) {
				printk(KERN_ERR "failed to enable sys_clkout1\n");
			}
		} else
			clk_disable(sys_clkout1);
		
		clk_put(sys_clkout1);
	}
}

static struct audio_device_config audio_device_io = {
	.set_spdif = &_set_ampli,
	.get_headphone_plugged =&_get_headphone_plugged,
	.get_headphone_irq =&_get_headphone_irq,
	.set_codec_master_clk_state = &_sys_clkout1_en,
	.set_speaker_state = &_set_hp,
};

struct audio_device_config *archos_audio_get_io(void) {
		return &audio_device_io;
}


int __init archos_audio_gpio_init(void)
{
	const struct archos_audio_config *audio_cfg;

	/* audio  */
	audio_cfg = omap_get_config( ARCHOS_TAG_AUDIO, struct archos_audio_config );
	if (audio_cfg == NULL) {
		printk(KERN_DEBUG "archos_audio_gpio_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= audio_cfg->nrev ) {
		printk(KERN_DEBUG "archos_audio_gpio_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, audio_cfg->nrev);
		return -ENODEV;
	}

	audio_gpio = audio_cfg->rev[hardware_rev];

	GPIO_INIT_OUTPUT( audio_gpio.spdif );

	GPIO_INIT_OUTPUT( audio_gpio.hp_on );

	GPIO_INIT_INPUT( audio_gpio.headphone_plugged );

	omap_set_gpio_debounce(GPIO_PIN( audio_gpio.headphone_plugged ),1);

	printk("Audio GPIO init done\n");
	return 0;
}

EXPORT_SYMBOL(archos_audio_get_io);

