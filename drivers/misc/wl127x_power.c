/*
 * Bluetooth[WL1271] on OMAP3430 Zoom2 boards
 *
 * Copyright (C) 2008 Texas Instruments
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <mach/gpio.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>

static int wl127x_probe(struct platform_device *dev);
static int wl127x_remove(struct platform_device *dev);
static int wl127x_suspend(struct platform_device *device, pm_message_t state);
static int wl127x_resume(struct platform_device *device);
static ssize_t fm_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count);
static ssize_t fm_show(struct kobject *kobj, struct kobj_attribute *attr,
		       char *buf);

static struct platform_driver wl127x_platform_driver = {
	.driver = {
		   .name = "wl127x",
		   .owner = THIS_MODULE,
		   },
	.probe = wl127x_probe,
	.remove = wl127x_remove,
	.suspend = wl127x_suspend,
	.resume = wl127x_resume,
};

static int nshut_down_gpio;
static int fm_enable_gpio;
static int bt_active_led;

struct rfkill *btrfkill;
static int nshut_down_gpio_status;
static int fm_gpio_status;
static int nshut_down_state;
static struct kobject *fm_kobj;

/* Create attributes for /sys/wl127x/fm_enable
 */
static struct kobj_attribute fm_attr =
__ATTR(fm_enable, 0666, fm_show, fm_store);

static struct attribute *attrs[] = {
	&fm_attr.attr,
	NULL,
};

static struct attribute_group attr_grp = {
	.attrs = attrs,
};

/* FM functions */
static ssize_t fm_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t count)
{
	int fm_state;
	sscanf(buf, "%du", &fm_state);
	if (fm_gpio_status != -1) {
		/* DIR_OUTPUT = 0 */
		omap_set_gpio_direction(fm_enable_gpio, 0);
		omap_set_gpio_dataout(fm_enable_gpio, fm_state);
		printk(KERN_INFO "fm_state changed to %d\n", fm_state);
	}
	return count;
}

static ssize_t fm_show(struct kobject *kobj, struct kobj_attribute *attr,
		       char *buf)
{
	return sprintf(buf, " %d \n", omap_get_gpio_datain(fm_enable_gpio));
}

/* BT functions */

static int btrfkill_get_state(void *data, enum rfkill_state *state)
{
	nshut_down_state = omap_get_gpio_datain(nshut_down_gpio);
	*state = nshut_down_state;
	printk(KERN_INFO "Current RF State is %d\n", *state);
	return 0;
}

static int btrfkill_toggle_radio(void *data, enum rfkill_state state)
{
	omap_set_gpio_direction(nshut_down_gpio, 0);
	omap_set_gpio_dataout(nshut_down_gpio, state);

	printk(KERN_INFO "Changed RF State to %d\n", state);

	nshut_down_state = state;
	return 0;
}

static int wl127x_suspend(struct platform_device *device, pm_message_t state)
{
/* TODO:Nothing as of yet..
 */
	return 0;
}

static int wl127x_resume(struct platform_device *device)
{
	/* Just to let know of the function
	 */
	return 0;
}

static int wl127x_probe(struct platform_device *dev)
{
	int *gpios = dev->dev.platform_data;

	printk(KERN_INFO "GPIOs are %d, %d\n", gpios[0], gpios[1]);
	nshut_down_gpio = gpios[0];
	fm_enable_gpio = gpios[1];
	bt_active_led = gpios[2];

#ifdef DEBUG_FM
	fm_enable_gpio = 159;
	/* This is LED-D6, gpio led1 on debug board */
#endif
	/* Creating the sysfs entry
	 * sys/class/rfkill/rfkill%d/...
	 */
	if (omap_request_gpio(nshut_down_gpio) != 0)
		nshut_down_gpio_status = -1;

	/*      TODO:
	 *      omap_set_gpio_direction(bt_active_led, 0);
	 *      omap_set_gpio_dataout(bt_active_led, 1);
	 */

	btrfkill = rfkill_allocate(NULL, RFKILL_TYPE_BLUETOOTH);
	btrfkill->name = "Bluetooth on OMA3430 Zoom2";
	btrfkill->state = RFKILL_STATE_OFF;
	/* Could also be RFKILL_STATE_SOFT_BLOCKED
	 * Deprecated
	 */
	btrfkill->toggle_radio = btrfkill_toggle_radio;
	btrfkill->get_state = btrfkill_get_state;
	btrfkill->user_claim = 1;

	rfkill_register(btrfkill);

	/* FM sysfs entry, chose sysfs instead of rfkill, to
	 * reduce number of files to edit
	 */
	/* Create an entry /sys/wl127x/fm_enable
	 */
	fm_kobj = kobject_create_and_add("wl127x", NULL);
	if (!fm_kobj)
		return -ENOMEM;

	if (sysfs_create_group(fm_kobj, &attr_grp))
		kobject_put(fm_kobj);

	if (omap_request_gpio(fm_enable_gpio) != 0) {
		printk(KERN_ERR "Error: Cannot request FM GPIO: %d\n",
		       fm_enable_gpio);
		fm_gpio_status = -1;
	}

	return 0;
}

static int wl127x_remove(struct platform_device *dev)
{
	if (nshut_down_gpio_status != -1)
		omap_free_gpio(nshut_down_gpio);
	if (btrfkill)
		rfkill_unregister(btrfkill);

	/* FIXME:
	 * Should rfkill_free be called
	 */
	kobject_put(fm_kobj);
	if (fm_gpio_status != -1)
		omap_free_gpio(fm_enable_gpio);

	return 0;
}

static int wl127x_init(void)
{
	return platform_driver_register(&wl127x_platform_driver);
}

static void wl127x_exit(void)
{
	platform_driver_unregister(&wl127x_platform_driver);
}

module_init(wl127x_init);
module_exit(wl127x_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Pavan Savoy <pavan.savoy@gmail.com>");
