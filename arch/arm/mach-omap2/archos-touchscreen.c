#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <asm/mach-types.h>
#include <mach/archos-gpio.h>
#include <mach/board-archos.h>
#include <linux/delay.h>

/* GPIO used for TSC2046 (touch screen)
 *
 * Also note that the tsc2046 is the same silicon as the ads7846, so
 * that driver is used for the touch screen. */
static struct archos_gpio ts_pwron = UNUSED_GPIO;
static struct archos_gpio ts_irq = UNUSED_GPIO;

static int ads7846_get_pendown_state(void)
{
	return !omap_get_gpio_datain( GPIO_PIN( ts_irq ) );
}

/* This enable(1)/disable(0) the voltage for TS */
static int ads7846_vaux_control(int vaux_cntrl)
{
	/* on Gen7, TSC_PWRON is inverted */
	int enable = machine_is_archos_g6h() || machine_is_archos_g6s()
			/* || machine_is_archos_g6l() || machine_is_archos_g6plus() */;

	if (vaux_cntrl == VAUX_ENABLE) {
		omap_set_gpio_dataout( GPIO_PIN( ts_pwron ), enable);
		msleep(2);		// rampup for tsc2008
	} else if (vaux_cntrl == VAUX_DISABLE)
		omap_set_gpio_dataout( GPIO_PIN( ts_pwron ), !enable);

	return 0;
}

static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};

static struct ads7846_platform_data tsc2046_config = {
	.get_pendown_state = ads7846_get_pendown_state,
	.keep_vref_on	   = 0,
	.vaux_control	   = ads7846_vaux_control,
	.settle_delay_usecs = 100,
	.x_plate_ohms      = 765,
	.pressure_min	   = 100,
	.pressure_max	   = 750,
	.penirq_recheck_delay_usecs = 3000,
	.model = 2008,
};

struct spi_board_info ts_spi_board_info[] = {
	[0] = {
		.modalias	= "ads7846",
		.bus_num	= 1,
		.chip_select	= 0,
		.max_speed_hz   = 3000000,
		.controller_data= &tsc2046_mcspi_config,
		.platform_data  = &tsc2046_config,
	},
};

int __init ads7846_dev_init(void)
{
	const struct archos_tsp_config *tsp_cfg;
	tsp_cfg = omap_get_config( ARCHOS_TAG_TSP, struct archos_tsp_config );
	/* might be NULL */
	if (tsp_cfg == NULL) {
		printk(KERN_DEBUG "ads7846_dev_init: no board configuration found\n");
		return -ENODEV;
	}
	if ( hardware_rev >= tsp_cfg->nrev ) {
		printk(KERN_DEBUG "ads7846_dev_init: hardware_rev (%i) >= nrev (%i)\n",
			hardware_rev, tsp_cfg->nrev);
		return -ENODEV;
	}

	ts_irq = tsp_cfg->rev[hardware_rev].irq_gpio;
	ts_pwron = tsp_cfg->rev[hardware_rev].pwr_gpio;

	tsc2046_config.x_plate_ohms = tsp_cfg->rev[hardware_rev].x_plate_ohms;
	tsc2046_config.pressure_max = tsp_cfg->rev[hardware_rev].pressure_max;

	printk(KERN_DEBUG "ads7846_dev_init: irq_gpio %i, pwr_gpio %i\n",
			ts_irq.nb, ts_pwron.nb);

	GPIO_INIT_OUTPUT(ts_pwron);
	GPIO_INIT_INPUT(ts_irq);
	ads7846_vaux_control( VAUX_DISABLE );

	omap_set_gpio_debounce(GPIO_PIN(ts_irq), 1);
	omap_set_gpio_debounce_time(GPIO_PIN(ts_irq), 0xa);

	/* fix config for Gen6 */
	if (machine_is_archos_g6h() || machine_is_archos_g6s()
			/* || machine_is_archos_g6l() || machine_is_archos_g6plus() */) {
		tsc2046_config.keep_vref_on	   = 1;
		tsc2046_config.x_plate_ohms      = 745;
		tsc2046_config.pressure_max	   = 700;
		tsc2046_config.penirq_recheck_delay_usecs = 3000;
		tsc2046_config.model = 7846;
		ts_spi_board_info[0].max_speed_hz   = 400000;
	}

	/* fix spi irq gio nb */
	ts_spi_board_info[0].irq = OMAP_GPIO_IRQ(GPIO_PIN(ts_irq));
	spi_register_board_info(ts_spi_board_info, 1);

	return 0;
}
