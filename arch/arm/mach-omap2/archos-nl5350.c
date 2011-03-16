/*
 * archos-nl5350.c
 *
 *  Created on: Feb 20, 2009
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 */
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <mach/archos-gpio.h>
#include <mach/board-archos.h>

static struct nl5350_struct {
	struct archos_gps_conf gps_conf;
	struct platform_device *pdev;
	struct work_struct work;
	struct wake_lock wake_lock;
} nl5350_obj;

/*
 * NL5350 device and attributes
 */
static ssize_t show_nl5350_enable(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct nl5350_struct *nl5350 = dev->platform_data;
	struct archos_gps_conf *conf = &nl5350->gps_conf;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", 
			omap_get_gpio_datain(GPIO_PIN(conf->gps_enable)));
};

static ssize_t store_nl5350_enable(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct nl5350_struct *nl5350 = dev->platform_data;
	struct archos_gps_conf *conf = &nl5350->gps_conf;
	
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	omap_set_gpio_dataout(GPIO_PIN(conf->gps_enable), on_off);	
	return count;
}
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, show_nl5350_enable, store_nl5350_enable);

static ssize_t show_nl5350_reset(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct nl5350_struct *nl5350 = dev->platform_data;
	struct archos_gps_conf *conf = &nl5350->gps_conf;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", 
			omap_get_gpio_datain(GPIO_PIN(conf->gps_reset)));
};

static ssize_t store_nl5350_reset(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct nl5350_struct *nl5350 = dev->platform_data;
	struct archos_gps_conf *conf = &nl5350->gps_conf;
	
	int on_off = simple_strtol(buf, NULL, 10);
	
	if( on_off > 1 || on_off < -1 )
		return -EINVAL;

	omap_set_gpio_dataout(GPIO_PIN(conf->gps_reset), on_off);
	
	if (on_off) {
		if (!wake_lock_active(&nl5350->wake_lock)) {
			wake_lock(&nl5350->wake_lock);
		}
	} else {
		if (wake_lock_active(&nl5350->wake_lock)) {
			wake_unlock(&nl5350->wake_lock);
		}
	}
	
	return count;
}
static DEVICE_ATTR(reset, S_IRUGO|S_IWUSR, show_nl5350_reset, store_nl5350_reset);

static ssize_t show_nl5350_intr(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct nl5350_struct *nl5350 = dev->platform_data;
	struct archos_gps_conf *conf = &nl5350->gps_conf;
	
	return snprintf(buf, PAGE_SIZE, "%d\n", 
			omap_get_gpio_datain(GPIO_PIN(conf->gps_int)));
};
static DEVICE_ATTR(intr, S_IRUGO, show_nl5350_intr, NULL);

static struct platform_device nl5350_device = {
	.name = "nl5350",
	.id = -1,
	.dev.platform_data = &nl5350_obj,
};

/*
 * NL5350 device driver
 */
static void nl5350_irq_worker(struct work_struct *work)
{
	struct nl5350_struct *nl5350 = container_of(work, struct nl5350_struct, work);
	struct platform_device *pdev = nl5350->pdev;

	static const char *envp[] = {
		"INTERRUPT",
		NULL,
	};
	dev_dbg(&pdev->dev, "nl5350_irq_worker: sending uevent\n");
	kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, (char**)envp);
}

static irqreturn_t nl5350_isr(int irq, void *dev_id)
{
	struct nl5350_struct *nl5350 = dev_id;
	struct platform_device *pdev = nl5350->pdev;
	
	dev_dbg(&pdev->dev, "nl5350_isr: irq %i called\n", irq);
	schedule_work(&nl5350->work);
	return IRQ_HANDLED;
}

static int nl5350_probe(struct platform_device *pdev)
{
	int ret;
	struct nl5350_struct *nl5350 = pdev->dev.platform_data;
	struct archos_gps_conf *conf = &nl5350->gps_conf;
	
	GPIO_INIT_OUTPUT(conf->gps_enable);
	GPIO_INIT_OUTPUT(conf->gps_reset);
	GPIO_INIT_INPUT(conf->gps_int);
	
	omap_cfg_reg(AD25_3430_UART2_RX);
	omap_cfg_reg(AA25_3430_UART2_TX);

	INIT_WORK(&nl5350->work, nl5350_irq_worker);

	ret = request_irq(OMAP_GPIO_IRQ(GPIO_PIN(conf->gps_int)), 
			nl5350_isr, IRQF_TRIGGER_RISING, "nl5350", nl5350);
	if (ret < 0) {
		dev_err(&pdev->dev, "nl5350_probe: cannot register irq %d\n", 
				OMAP_GPIO_IRQ(GPIO_PIN(conf->gps_int)));
		return ret;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_enable);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot add enable attr\n");
	ret = device_create_file(&pdev->dev, &dev_attr_reset);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot add reset attr\n");
	ret = device_create_file(&pdev->dev, &dev_attr_intr);
	if (ret < 0)
		dev_dbg(&pdev->dev, "cannot add intr attr\n");
	
	wake_lock_init(&nl5350->wake_lock, WAKE_LOCK_SUSPEND, "nl5350");

	nl5350->pdev = pdev;
	return 0;
}

static struct platform_driver nl5350_driver = {
	.probe = nl5350_probe,
#ifdef CONFIG_PM
	//.suspend = nl5350_suspend,
	//.resume = nl5350_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name  = "nl5350",
	},
};

static int __init archos_nl5350_init(void)
{
	int ret;
	
	const struct archos_gps_config *gps_config = 
			omap_get_config(ARCHOS_TAG_GPS, struct archos_gps_config);
	
	pr_debug("archos_nl5350_init\n");
	
	if (gps_config == NULL) {
		pr_debug("archos_nl5350_init: no gps configuration\n");
		return -ENODEV;
	}
	
	if (hardware_rev >= gps_config->nrev) {
		pr_err("archos_nl5350_init: no configuration for hardware_rev %d\n", 
				hardware_rev);
		return -ENODEV;
	}
		
	nl5350_obj.gps_conf = gps_config->rev[hardware_rev];
	
	ret = platform_device_register(&nl5350_device);
	if (ret < 0) {
		pr_err("archos_nl5350_init: failed to register platform device\n");
		return -ENODEV;
	}
	return platform_driver_register(&nl5350_driver);
}

device_initcall(archos_nl5350_init);
