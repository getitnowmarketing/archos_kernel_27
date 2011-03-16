#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/board-archos.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/mmc.h>

static int wifi_pwron = -1;
static int wifi_irqpin = -1;
static int bt_pwron = -1;
static int bt_power_state;

int mmc2_card_inserted;
EXPORT_SYMBOL(mmc2_card_inserted);

void archos_wifi_set_power(int enable)
{
	printk("archos_wifi_set_power %s (wifi), pwr: %d\n", enable ? "on":"off", wifi_pwron);

	if (wifi_pwron == -1)
		return;

	if (enable) {
		omap_set_gpio_dataout( wifi_pwron, 1);	

		//msleep(200);
		omap_cfg_reg(AE2_3430_MMC2_CLK);
		omap_cfg_reg(AG5_3430_MMC2_CMD);
		omap_cfg_reg(AH5_3430_MMC2_DAT0);
		omap_cfg_reg(AH4_3430_MMC2_DAT1);
		omap_cfg_reg(AG4_3430_MMC2_DAT2);
		omap_cfg_reg(AF4_3430_MMC2_DAT3);

	} else {
		omap_cfg_reg(AE2_3430_SAFE);
		omap_cfg_reg(AG5_3430_SAFE);
		omap_cfg_reg(AH5_3430_SAFE);
		omap_cfg_reg(AH4_3430_SAFE);
		omap_cfg_reg(AG4_3430_SAFE);
		omap_cfg_reg(AF4_3430_SAFE);

		omap_set_gpio_dataout( wifi_pwron, 0);
	}
}

EXPORT_SYMBOL(archos_wifi_set_power);

void archos_wifi_set_carddetect(int detect)
{
	printk(KERN_ERR "%s: %d\n", __FUNCTION__, detect);
	mmc2_card_inserted = detect;
}

EXPORT_SYMBOL(archos_wifi_set_carddetect);

int archos_wifi_get_irqnr(void)
{
	if (wifi_irqpin == -1) {
		printk(KERN_ERR "ERROR: archos_wifi_get_irqnr: %d\n", wifi_irqpin);
		return -1;
	}	
	return gpio_to_irq(wifi_irqpin);
}

EXPORT_SYMBOL(archos_wifi_get_irqnr);

void archos_bt_set_power(int enable)
{
	printk(KERN_DEBUG "%s, %s\n", __FUNCTION__, enable ? "on":"off");
	bt_power_state = enable;
	omap_set_gpio_dataout(bt_pwron, enable ? 1:0);
}

EXPORT_SYMBOL(archos_bt_set_power);


static ssize_t archos_wifi_show_store_enable_status(struct device *class,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;

	int value = simple_strtoul(buf, &endp, 10);

	archos_wifi_set_power(value ? 1:0);
	
	return count;
}

static ssize_t archos_wifi_show_enable_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", omap_get_gpio_datain(wifi_pwron));
}

static DEVICE_ATTR(wifi_enable, S_IRUGO|S_IWUSR, archos_wifi_show_enable_status, archos_wifi_show_store_enable_status);

static ssize_t archos_bt_show_store_enable_status(struct device *class,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;

	int value = simple_strtoul(buf, &endp, 10);

	archos_bt_set_power(value ? 1:0);
	
	return count;
}

static ssize_t archos_bt_show_enable_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bt_power_state);
}

static DEVICE_ATTR(bt_enable, S_IRUGO|S_IWUSR, archos_bt_show_enable_status, archos_bt_show_store_enable_status);


static struct platform_device wifi_bt_power_switching_device = {
	.name = "wifi_bt",
	.id = -1,
};

static struct platform_device wifi_device = {
	.name = "TIWLAN_SDIO",
	.id = 1,
};

/* Wifi/Bt power management driver */

static int wifi_bt_probe(struct platform_device *dev)
{
	return 0;
}

static int wifi_bt_suspend(struct platform_device *dev, pm_message_t pm)
{
	if (bt_power_state)
		omap_cfg_reg(W21_3430_SAFE_PULLUP);
	else
		omap_cfg_reg(W21_3430_SAFE_PULLDOWN);
	return 0;	
}

static int wifi_bt_resume(struct platform_device *dev)
{
	omap_cfg_reg(W21_3430_GPIO162);
	return 0;
}

static struct platform_driver wifi_bt_driver = {
	.driver.name = "wifi_bt",
	.probe = wifi_bt_probe,
#ifdef CONFIG_PM
	.suspend_late = wifi_bt_suspend,
	.resume_early = wifi_bt_resume,
#endif
};

int __init archos_wifi_bt_init(void)
{
	int ret;
	const struct archos_wifi_bt_config *conf = 
		omap_get_config(ARCHOS_TAG_WIFI_BT, struct archos_wifi_bt_config);
	if (!conf)
		return -ENODEV;

	if (hardware_rev < conf->nrev) {
		if (conf->rev[hardware_rev].wifi_power.nb != 0) 
			wifi_pwron = conf->rev[hardware_rev].wifi_power.nb;
		
		if (conf->rev[hardware_rev].wifi_irq.nb != 0) 
			wifi_irqpin = conf->rev[hardware_rev].wifi_irq.nb;				

		if (conf->rev[hardware_rev].bt_power.nb != 0) 
			bt_pwron = conf->rev[hardware_rev].bt_power.nb;				

		if (conf->rev[hardware_rev].wifi_power.mux_cfg != 0)
			omap_cfg_reg( conf->rev[hardware_rev].wifi_power.mux_cfg );
		
		if (conf->rev[hardware_rev].wifi_irq.mux_cfg != 0)
			omap_cfg_reg( conf->rev[hardware_rev].wifi_irq.mux_cfg );

		if (conf->rev[hardware_rev].bt_power.mux_cfg != 0)
			omap_cfg_reg( conf->rev[hardware_rev].bt_power.mux_cfg );

		if (wifi_pwron != -1) {
			if (omap_request_gpio( wifi_pwron ) < 0)
				printk(KERN_ERR "can't get WIFI_PWRON (GPIO%d)\n", wifi_pwron);
		 	omap_set_gpio_dataout(wifi_pwron, 0);	
			omap_set_gpio_direction(wifi_pwron, GPIO_DIR_OUTPUT);
		}

#if 1 // requested in the tiwlan driver
		if (wifi_irqpin != -1) {
			if (omap_request_gpio( wifi_irqpin ) < 0)
				printk(KERN_ERR "can't get WIFI_IRQ (GPIO%d)\n", wifi_irqpin);
			omap_set_gpio_direction(wifi_irqpin, GPIO_DIR_INPUT);
		}
#endif				
		if (bt_pwron != -1) {
			if (omap_request_gpio(bt_pwron) < 0)
				printk(KERN_ERR "can't get BT_PWRON (GPIO%d)\n", bt_pwron);
			omap_set_gpio_dataout(bt_pwron, 0);
			omap_set_gpio_direction(bt_pwron, GPIO_DIR_OUTPUT);
		}

	}
	
	omap_cfg_reg(W8_3430_UART1_CTS);
	omap_cfg_reg(AA9_3430_UART1_RTS);
	omap_cfg_reg(Y8_3430_UART1_RX);
	omap_cfg_reg(AA8_3430_UART1_TX);

	// wifi and bt enable pulse on/off to set chip in low consumption state
	archos_wifi_set_power(1);
	omap_set_gpio_dataout(bt_pwron, 1);
	msleep(10);
	archos_wifi_set_power(0);
	omap_set_gpio_dataout(bt_pwron, 0);

	ret = platform_device_register(&wifi_device);

	if (ret < 0)
		return -1;
	
	ret = platform_device_register(&wifi_bt_power_switching_device);
	if (ret == 0) {
		ret = device_create_file(&wifi_bt_power_switching_device.dev, &dev_attr_wifi_enable);	
		ret = device_create_file(&wifi_bt_power_switching_device.dev, &dev_attr_bt_enable);	
	}

	return platform_driver_register(&wifi_bt_driver);;
}
