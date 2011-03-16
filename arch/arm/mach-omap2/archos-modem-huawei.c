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
#include <mach/mcspi.h>
#include <linux/device.h>

static struct archos_modem_huawei_struct {
	struct archos_modem_huawei_conf modem_conf;
	int modem_reset_cycle;
	int modem_power_low_pulse;
	struct platform_device *pdev;
//	struct work_struct work;
} archos_modem_huawei_obj;

static void _do_reset_cycle( struct archos_gpio pin )
{
	if (GPIO_EXISTS( pin )) {
		pr_info("MODEM resetting\n");
		omap_set_gpio_dataout( GPIO_PIN( pin ), 1 );
		msleep(100);
		omap_set_gpio_dataout( GPIO_PIN( pin ), 0 );
		msleep(200);
	}
}

static void _do_power_on_pulse( struct archos_gpio pin )
{
	if (GPIO_EXISTS( pin )) {
		pr_info("MODEM power on low pulse\n");
		omap_cfg_reg(P26_3430_GPIO127);
		omap_set_gpio_dataout( GPIO_PIN( pin ), 0 );
		omap_set_gpio_dataout( GPIO_PIN( pin ), 1 );
		msleep(1000);
		omap_set_gpio_dataout( GPIO_PIN( pin ), 0 );
		omap_cfg_reg(P26_3430_SAFE); // pwron pin disconected
	}
}

static ssize_t archos_modem_store_pwron(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	pr_info("%s: (%d)\n", __FUNCTION__, on_off);
	if ( on_off ) {
		modem_data->modem_power_low_pulse = 1;
		_do_power_on_pulse(conf->pwron);
		modem_data->modem_power_low_pulse = 0;
	} else {
		modem_data->modem_power_low_pulse = 0;
	}

	return count;
}

static ssize_t archos_modem_show_pwron(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", 
			modem_data->modem_power_low_pulse);
}

static DEVICE_ATTR(pwron, S_IRUGO|S_IWUSR, archos_modem_show_pwron, archos_modem_store_pwron);


static ssize_t archos_modem_store_wwan_reset(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	pr_info("%s: (%d)\n", __FUNCTION__, on_off);

	omap_set_gpio_dataout(GPIO_PIN(conf->wwan_reset), on_off);	
	return count;
}

static ssize_t archos_modem_show_wwan_reset(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", 
			omap_get_gpio_datain(GPIO_PIN(conf->wwan_reset)));
}

static DEVICE_ATTR(wwan_reset, S_IRUGO|S_IWUSR, archos_modem_show_wwan_reset, archos_modem_store_wwan_reset);

static ssize_t archos_modem_store_wwan_disable(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	pr_info("%s: (%d)\n", __FUNCTION__, on_off);

	omap_set_gpio_dataout(GPIO_PIN(conf->wwan_disable), on_off);	
	return count;
}

static ssize_t archos_modem_show_wwan_disable(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", 
			omap_get_gpio_datain(GPIO_PIN(conf->wwan_disable)));
}

static DEVICE_ATTR(wwan_disable, S_IRUGO|S_IWUSR, archos_modem_show_wwan_disable, archos_modem_store_wwan_disable);


static ssize_t archos_modem_store_wake_module(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	pr_info("%s: (%d)\n", __FUNCTION__, on_off);

	omap_set_gpio_dataout(GPIO_PIN(conf->wake_module), on_off);	
	return count;
}

static ssize_t archos_modem_show_wake_module(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", 
			omap_get_gpio_datain(GPIO_PIN(conf->wake_module)));
}

static DEVICE_ATTR(wake_module, S_IRUGO|S_IWUSR, archos_modem_show_wake_module, archos_modem_store_wake_module);

static ssize_t archos_modem_store_enable_usb2(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	pr_info("%s: (%d)\n", __FUNCTION__, on_off);

	omap_set_gpio_dataout(GPIO_PIN(conf->enable_usb2), on_off);	
	return count;
}

static ssize_t archos_modem_show_enable_usb2(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", 
			omap_get_gpio_datain(GPIO_PIN(conf->enable_usb2)));
}

static DEVICE_ATTR(enable_usb2, S_IRUGO|S_IWUSR, archos_modem_show_enable_usb2, archos_modem_store_enable_usb2);

static ssize_t archos_modem_store_gps_enable(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	pr_info("%s: (%d)\n", __FUNCTION__, on_off);

	omap_set_gpio_dataout(GPIO_PIN(conf->gps_enable), on_off);	
	return count;
}

static ssize_t archos_modem_show_gps_enable(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", 
			omap_get_gpio_datain(GPIO_PIN(conf->gps_enable)));
}

static DEVICE_ATTR(gps_enable, S_IRUGO|S_IWUSR, archos_modem_show_gps_enable, archos_modem_store_gps_enable);

static ssize_t archos_modem_store_reset_cycle(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;
	
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	pr_info("%s: (%d)\n", __FUNCTION__, on_off);

	if ( on_off ) {
		modem_data->modem_reset_cycle = 1;
		_do_reset_cycle(conf->wwan_reset);
		modem_data->modem_reset_cycle = 0;
	} else {
		modem_data->modem_reset_cycle = 0;
	}	
	return count;
}

static ssize_t archos_modem_show_reset_cycle(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct archos_modem_huawei_struct *modem_data = dev->platform_data;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", 
			modem_data->modem_reset_cycle);
}

static DEVICE_ATTR(reset_cycle, S_IRUGO|S_IWUSR, archos_modem_show_reset_cycle, archos_modem_store_reset_cycle);

static struct platform_device modem_huawei_device = {
	.name = "modemhuawei",
	.id = -1,
	.dev.platform_data = &archos_modem_huawei_obj,
};


/*
 * Modem device driver
 */

static int modem_huawei_probe(struct platform_device *pdev)
{
	int ret;
	struct archos_modem_huawei_struct *modem_data = pdev->dev.platform_data;
	struct archos_modem_huawei_conf *conf = &modem_data->modem_conf;

	pr_info("modem_huawei_probe\n");

	GPIO_INIT_OUTPUT(conf->wwan_reset);
	GPIO_INIT_OUTPUT(conf->wwan_disable);
	GPIO_INIT_OUTPUT(conf->wake_module);
	GPIO_INIT_OUTPUT(conf->enable_usb2);
	GPIO_INIT_OUTPUT(conf->gps_enable);

	GPIO_INIT_INPUT(conf->wake_omap);

	omap_cfg_reg(P26_3430_SAFE); // pwron pin disconected

	ret = device_create_file(&pdev->dev, &dev_attr_pwron);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot add pwron attr\n");
	ret = device_create_file(&pdev->dev, &dev_attr_wwan_reset);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot add wwan_reset attr\n");
	ret = device_create_file(&pdev->dev, &dev_attr_wwan_disable);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot add wwan_disable attr\n");
	ret = device_create_file(&pdev->dev, &dev_attr_wake_module);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot add wake_module attr\n");	
	ret = device_create_file(&pdev->dev, &dev_attr_enable_usb2);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot add enable_usb2 attr\n");	
	ret = device_create_file(&pdev->dev, &dev_attr_gps_enable);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot add gps_enable attr\n");	
	ret = device_create_file(&pdev->dev, &dev_attr_reset_cycle);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot add reset_cycle attr\n");
	modem_data->pdev = pdev;

	modem_data->modem_reset_cycle = 0;
	modem_data->modem_power_low_pulse = 0;

	/* reset the PHY */
	omap_set_gpio_dataout(GPIO_PIN(conf->enable_usb2), 0);
	udelay(2);
	omap_set_gpio_dataout(GPIO_PIN(conf->enable_usb2), 1);

	/* pin mux for EHCI Port 2 */
	omap_cfg_reg(AE7_3430_USB2HS_PHY_CLK);
	omap_cfg_reg(AF7_3430_USB2HS_PHY_STP);
	omap_cfg_reg(AG7_3430_USB2HS_PHY_DIR);
	omap_cfg_reg(AH7_3430_USB2HS_PHY_NXT);
	omap_cfg_reg(AG8_3430_USB2HS_PHY_DATA0);
	omap_cfg_reg(AH8_3430_USB2HS_PHY_DATA1);
	omap_cfg_reg(AB2_3430_USB2HS_PHY_DATA2);
	omap_cfg_reg(V3_3430_USB2HS_PHY_DATA3);
	omap_cfg_reg(Y2_3430_USB2HS_PHY_DATA4);
	omap_cfg_reg(Y3_3430_USB2HS_PHY_DATA5);
	omap_cfg_reg(Y4_3430_USB2HS_PHY_DATA6);
	omap_cfg_reg(AA3_3430_USB2HS_PHY_DATA7);

	// gps uart
	omap_cfg_reg(AD25_3430_UART2_RX);
	omap_cfg_reg(AA25_3430_UART2_TX);

	omap_set_gpio_dataout(GPIO_PIN(conf->wwan_disable), 0);
	omap_set_gpio_dataout(GPIO_PIN(conf->wwan_reset), 0);
	omap_set_gpio_dataout(GPIO_PIN(conf->gps_enable), 1);

	_do_reset_cycle(conf->wwan_reset);
	GPIO_INIT_OUTPUT(conf->pwron);
	_do_power_on_pulse(conf->pwron);
	//omap_cfg_reg(P26_3430_SAFE); 


	return 0;
}

int __init archos_modem_huawei_init(void)
{
	int ret = 0;

	const struct archos_modem_huawei_config *modem_config = 
		omap_get_config(ARCHOS_TAG_MODEM_HUAWEI, struct archos_modem_huawei_config);

	if (modem_config == NULL) {
		pr_info("archos_modem_huawei_init: no modem configuration\n");
		ret = -ENODEV;
		goto failed;
	}

	pr_info("MODEM HUAWEI start init\n");	

	if (hardware_rev >= modem_config->nrev) {
		pr_err("archos_modem_huawei_init: no configuration for hardware_rev %d\n", 
				hardware_rev);
		goto failed;
	}
	
	archos_modem_huawei_obj.modem_conf = modem_config->rev[hardware_rev];
	
	ret = platform_device_register(&modem_huawei_device);
	if (ret < 0) {
		pr_err("archos_modem_huawei_init: failed to register platform device\n");
		goto failed;
	}
	
	if ((ret = modem_huawei_probe(&modem_huawei_device)) < 0)
		goto failed;

	return 0;
failed:
	omap_cfg_reg(AF26_3430_SAFE); /* set hsdpa_wakup_omap in safe mode */
	return ret;
}

device_initcall(archos_modem_huawei_init);
