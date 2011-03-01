/* File: linux/arch/arm/mach-omap2/archos-usb-android.c
 *
 *  This file contains Archos platform-specific data for the Android USB
 * gadget driver.
 *
 * Copyright © 2009 Chidambar Zinnoury - Archos S.A.
 */

#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/usb/android.h>

#include <mach/board-archos.h>
#include <mach/archos-dieid.h>

#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id		= 0x0e79,
	.product_id		= 0x1360,
	.adb_product_id		= 0x1361,
	.version		= 0x0100,
	.product_name		= "Archos5",
	.manufacturer_name	= "Archos",
	.serial_number		= "JFBG-PAD5-DPE5-3C",
	.nluns			= 2,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

static int __init usb_android_init(void)
{
#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_ANDROID_MODULE)

	u32 prod_id[4];

	get_dieid(prod_id);

	if (machine_is_archos_a5s() || machine_is_archos_a5st()) {
		sprintf(android_usb_pdata.serial_number, "A5S-%08X-%08X-%08X-%08X", prod_id[0],prod_id[1],prod_id[2],prod_id[3]);
	} else if ( machine_is_archos_a5h() ) {
		sprintf(android_usb_pdata.serial_number, "A5H-%08X-%08X-%08X-%08X", prod_id[0],prod_id[1],prod_id[2],prod_id[3]);
	} else {
		sprintf(android_usb_pdata.serial_number, "A5x-%08X-%08X-%08X-%08X", prod_id[0],prod_id[1],prod_id[2],prod_id[3]);
	}

	printk(KERN_ERR "registering Android USB device (%s)\n", android_usb_pdata.serial_number);

	if (platform_device_register(&android_usb_device) < 0) {
		printk(KERN_ERR "Unable to register Android USB device\n");
		return -ENODEV;
	}
#endif

	return 0;
}

late_initcall(usb_android_init);
