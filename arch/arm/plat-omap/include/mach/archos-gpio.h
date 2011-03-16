#ifndef _ARCH_ARCHOS_GPIO_H_
#define _ARCH_ARCHOS_GPIO_H_

#include <mach/gpio.h>
#include <mach/mux.h>

struct archos_gpio {
	int nb;
	int mux_cfg;
};

#define GPIO_EXISTS(x) ( x.nb >= 0 )
#define GPIO_MUX(x) ( x.mux_cfg )
#define GPIO_PIN(x) ( x.nb )
#define UNUSED_GPIO (struct archos_gpio){ .nb = -1, .mux_cfg = -1 }
#define INITIALIZE_GPIO(pin, cfg) (struct archos_gpio){ .nb = pin, .mux_cfg = cfg }

static inline void archos_gpio_init_output(struct archos_gpio *x)
{
	int pin = x->nb;
	int cfg = x->mux_cfg;

	if (pin < 0)
		return;

	if (omap_request_gpio(pin) < 0) {
		pr_debug("archos_gpio_init_output: cannot acquire GPIO%d \n", pin);
		return;
	}
	omap_cfg_reg( cfg ) ;
	omap_set_gpio_direction(pin, GPIO_DIR_OUTPUT);
}
#define GPIO_INIT_OUTPUT(x) archos_gpio_init_output(&x)

static inline void archos_gpio_init_input(struct archos_gpio *x)
{
	int pin = x->nb;
	int cfg = x->mux_cfg;

	if (pin < 0)
		return;

	if (omap_request_gpio(pin) < 0) {
		pr_debug("archos_gpio_init_input: cannot acquire GPIO%d \n", pin);
		return;
	}
	omap_cfg_reg( cfg ) ;
	omap_set_gpio_direction(pin, GPIO_DIR_INPUT);
}
#define GPIO_INIT_INPUT(x) archos_gpio_init_input(&x)

#endif
