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
#include <mach/archos-accessory.h>


#define NEED_UART	1<<0
#define NEED_IR		1<<1
#define NEED_TX_GPIO_OUT 1<<2
#define NEED_RX_GPIO_OUT 1<<3
#define NEED_TX_HIGH	1<<4
#define NEED_RX_HIGH	1<<5

/* Default mode */
#define DEFAULT_IR_MODE	NEED_UART


static struct archos_gpio gpio_5Vusb = UNUSED_GPIO;

static int uart3_rx_mux;
static int uart3_tx_mux;
static struct archos_gpio gpio_uart3_rx;
static struct archos_gpio gpio_uart3_tx;
static struct archos_pwm_conf irblaster_pwm;
static struct archos_gpio irblaster_pwm_disable;
static struct archos_pwm_conf irremote_timer;
static struct archos_gpio irremote_timer_disable;

static int current_ir_mode;

static ssize_t show_5vusb(struct device *dev, 
		struct device_attribute *attr, char* buf)
{
	int val_5vusb = omap_get_gpio_datain( GPIO_PIN( gpio_5Vusb ) );
	return snprintf(buf, PAGE_SIZE, "%i\n", val_5vusb); 
}

static ssize_t store_5vusb(struct device *dev, struct device_attribute *attr, 
		const char* buf, size_t len)
{
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	omap_set_gpio_dataout( GPIO_PIN( gpio_5Vusb ), on_off);	
	return len;
}

static DEVICE_ATTR(5vusb, S_IRUGO|S_IWUSR, show_5vusb, store_5vusb);


int archos_accessory_set_irmode(const char *mode)
{

	int need = 0;

	if ( strncmp(mode,"uart",4) == 0 ) {
		need = NEED_UART;
	} else if ( strncmp(mode,"ir",2) == 0 ) {
		need = NEED_IR;
	} else if ( strncmp(mode,"both",4) == 0 ) {
		need = NEED_UART | NEED_IR;
	} else if ( strncmp(mode,"hdmi_dock_enable",17) == 0 ) {
		need = NEED_TX_GPIO_OUT;
	} else if ( strncmp(mode,"hdmi_dock_disable",17) == 0 ) {
		need = NEED_TX_GPIO_OUT | NEED_TX_HIGH;
	} else if ( strncmp(mode,"none",4) == 0 ) {
		need = 0;
	} else if ( strncmp(mode,"default",7) == 0 ) {
		need = DEFAULT_IR_MODE;
	} else  {
		/* default */
		need = current_ir_mode;
	}


	if ( need & NEED_UART ) {
		printk(KERN_DEBUG "archos_accessory_set_irmode: enabling UART-3...\n");
		/* connect uart3 rx/tx */
		omap_cfg_reg(uart3_rx_mux);
		omap_cfg_reg(uart3_tx_mux);
	} else {
		printk(KERN_DEBUG "archos_accessory_set_irmode: disabling UART-3...\n");
		omap_cfg_reg(gpio_uart3_rx.mux_cfg);
		omap_cfg_reg(gpio_uart3_tx.mux_cfg);

		if ( need & NEED_TX_GPIO_OUT ) {
			printk(KERN_DEBUG "archos_accessory_set_irmode: enabling GPIO TX at %dL...\n", !!(need & NEED_TX_HIGH));
			omap_set_gpio_direction( GPIO_PIN( gpio_uart3_tx ), GPIO_DIR_OUTPUT);
			omap_set_gpio_dataout( GPIO_PIN(gpio_uart3_tx), need & NEED_TX_HIGH);
		} else {
			omap_set_gpio_direction( GPIO_PIN( gpio_uart3_tx ), GPIO_DIR_INPUT);
		}

		if ( need & NEED_RX_GPIO_OUT ) {
			printk(KERN_DEBUG "archos_accessory_set_irmode: enabling GPIO RX at %dL...\n", !!(need & NEED_RX_HIGH));
			omap_set_gpio_direction( GPIO_PIN( gpio_uart3_rx ), GPIO_DIR_OUTPUT);
			omap_set_gpio_dataout( GPIO_PIN(gpio_uart3_rx), need & NEED_RX_HIGH);
		} else {
			omap_set_gpio_direction( GPIO_PIN( gpio_uart3_rx ), GPIO_DIR_INPUT);
		}
	}

	if ( need & NEED_IR ) {
		printk(KERN_DEBUG "archos_accessory_set_irmode: enabling IR...\n");
		/* connect ir remote */
		omap_cfg_reg(irremote_timer.mux_cfg);
		/* connect ir blaster */
		omap_cfg_reg(irblaster_pwm.mux_cfg);
	} else {
		printk(KERN_DEBUG "archos_accessory_set_irmode: disabling IR...\n");
		/* disconnect ir remote */
		/* irremote => gpio as input */
		omap_set_gpio_direction( GPIO_PIN( irremote_timer_disable ), GPIO_DIR_INPUT);
		omap_cfg_reg(irremote_timer_disable.mux_cfg);
		/* disconnect ir blaster */
		/* irblaster => gpio as input */
		omap_set_gpio_direction( GPIO_PIN( irblaster_pwm_disable ), GPIO_DIR_INPUT);
		omap_cfg_reg(irblaster_pwm_disable.mux_cfg);
	}

	current_ir_mode = need;
	
	return 0;
}

EXPORT_SYMBOL(archos_accessory_set_irmode);

static ssize_t show_irmode(struct device *dev,
		struct device_attribute *attr, char* buf)
{
	int ret;
	switch(current_ir_mode) {
		case NEED_UART:
			ret = sprintf(buf, "uart\n"); 
		break;
		case NEED_IR:
			ret = sprintf(buf, "ir\n"); 
		break;
		case (NEED_IR|NEED_UART):
			ret = sprintf(buf, "both\n"); 
		break;
		case (NEED_TX_GPIO_OUT):
			ret = sprintf(buf, "hdmi_dock_enable\n");
		break;
		case (NEED_TX_GPIO_OUT|NEED_TX_HIGH):
			ret = sprintf(buf, "hdmi_dock_disable\n");
		break;
		case 0:
			ret = sprintf(buf, "none\n"); 
		break;
		default:
			ret = sprintf(buf, "unknown\n"); 
		break;
	}
	
	return ret; 
}

static ssize_t store_irmode(struct device *dev, struct device_attribute *attr,
		const char* buf, size_t len)
{

	archos_accessory_set_irmode(buf);

	return len;
}

static DEVICE_ATTR(ir_mode, S_IRUGO|S_IWUSR, show_irmode, store_irmode);


static struct platform_device accessory_device = {
	.name = "accessory_plug",
	.id = -1,
};

static int __init archos_accessory_init(void)
{
	unsigned long aux;
	int ret;
	/* USB0 PHY VBUS */
	const struct archos_vbus_config *vbus_cfg;
	const struct archos_irremote_config *irremote_cfg;
	const struct archos_irblaster_config *irblaster_cfg;
	const struct archos_uart3_config *uart3_cfg;

	/* Get VBUS config */
	vbus_cfg = omap_get_config( ARCHOS_TAG_VBUS0, struct archos_vbus_config );
	if (vbus_cfg == NULL)
		printk(KERN_ERR "archos_accessory_init: no board configuration found (VBUS)\n");
	else if ( hardware_rev >= vbus_cfg->nrev ) {
		printk(KERN_ERR "archos_accessory_init: hardware_rev (%i) >= nrev (%i) (VBUS)\n",
			hardware_rev, vbus_cfg->nrev);
		vbus_cfg = NULL;
	}

	/* Get IR-Blaster config */
	irblaster_cfg = omap_get_config( ARCHOS_TAG_IRBLASTER, struct archos_irblaster_config );
	if (irblaster_cfg == NULL)
		printk(KERN_ERR "archos_accessory_init: no board configuration found (IR-Blaster)\n");
	else if ( hardware_rev >= irblaster_cfg->nrev ) {
		printk(KERN_ERR "archos_accessory_init: hardware_rev (%i) >= nrev (%i) (IR-Blaster)\n",
			hardware_rev, irblaster_cfg->nrev);
		irblaster_cfg = NULL;
	}


	/* Get IR-Remote config */
	irremote_cfg = omap_get_config( ARCHOS_TAG_IRREMOTE, struct archos_irremote_config );
	if (irremote_cfg == NULL)
		printk(KERN_ERR "archos_accessory_init: no board configuration found (IR-Remote)\n");
	else if ( hardware_rev >= irremote_cfg->nrev ) {
		printk(KERN_ERR "archos_accessory_init: hardware_rev (%i) >= nrev (%i) (IR-Remote)\n",
			hardware_rev, irremote_cfg->nrev);
		irremote_cfg = NULL;
	}

	
	/* Get UART-3 config */
	uart3_cfg = omap_get_config( ARCHOS_TAG_UART3, struct archos_uart3_config );
	if (uart3_cfg == NULL)
		printk(KERN_ERR "archos_accessory_init: no board configuration found (UART-3)\n");
	else if ( hardware_rev >= uart3_cfg->nrev ) {
		printk(KERN_ERR "archos_accessory_init: hardware_rev (%i) >= nrev (%i) (UART-3)\n",
			hardware_rev, uart3_cfg->nrev);
		uart3_cfg = NULL;
	}

	ret = platform_device_register(&accessory_device);
	if (ret) {
		printk(KERN_ERR "archos_accessory_init: register device failed\n");
		return -ENODEV;
	}
	
	if (irblaster_cfg && irremote_cfg && uart3_cfg) {

		irblaster_pwm = irblaster_cfg->rev[hardware_rev].irblaster_pwm;
		irblaster_pwm_disable = irblaster_cfg->rev[hardware_rev].irblaster_pwm_disable;

		irremote_timer = irremote_cfg->rev[hardware_rev].irremote_timer;
		irremote_timer_disable = irremote_cfg->rev[hardware_rev].irremote_timer_disable;

		uart3_rx_mux = uart3_cfg->rev[hardware_rev].uart3_rx_mux;
		uart3_tx_mux = uart3_cfg->rev[hardware_rev].uart3_tx_mux;

		gpio_uart3_rx = uart3_cfg->rev[hardware_rev].gpio_uart3_rx;
		gpio_uart3_tx = uart3_cfg->rev[hardware_rev].gpio_uart3_tx;
	
		/* Default mode is UART-3 */
		archos_accessory_set_irmode("default");

		ret = device_create_file(&accessory_device.dev, &dev_attr_ir_mode);
	}
	
	if (vbus_cfg) {
		gpio_5Vusb = vbus_cfg->rev[hardware_rev];
		printk(KERN_DEBUG "archos_accessory_init: gpio %i\n", GPIO_PIN(gpio_5Vusb));

		GPIO_INIT_OUTPUT( gpio_5Vusb );
		omap_set_gpio_dataout( GPIO_PIN( gpio_5Vusb ), 0);
		
		ret = device_create_file(&accessory_device.dev, &dev_attr_5vusb);	
	}

	/* if nothing initialized */
	if (!(irblaster_cfg && irremote_cfg && uart3_cfg) && vbus_cfg == NULL) {
		platform_device_unregister(&accessory_device);
		return -ENODEV;
	}
	
	return 0;
}

late_initcall(archos_accessory_init);
