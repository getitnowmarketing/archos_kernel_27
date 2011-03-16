/* Special functions for Archos Gen6 builtin hard disk
 *
 *   (c) 2008 Matthias Welwarsky (welwarsky@archos.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hdd_dpm.h>
#include <linux/buffer_head.h>
#include <linux/suspend.h>

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/sd.h>

#include <mach/board-archos.h>

#include "usb.h"
#include "transport.h"
#include "protocol.h"
#include "scsiglue.h"
#include "debug.h"

#include "../core/usb.h"
#include "../core/hcd.h"

#define USB_HOST_NAME	"ehci-omap"
    
struct archos_hdd {
	struct us_data *us;
	struct usb_device *pusb_parent;
	struct hdd_dpm_ops dpm_ops;
};

#define to_archos_hdd(p) container_of(p, struct archos_hdd, dpm_ops)

extern int archos_enable_ehci( int enable );

static void archos_flashdisk_off( void )
{
	usbhdd_power(0);
	msleep(100);
}

static int archos_flashdisk_dpm_suspend(struct hdd_dpm_ops *dpm_ops)
{
	struct archos_hdd *hdd = to_archos_hdd(dpm_ops);
	struct us_data *us = hdd->us;
	int rc = 0;

	mutex_lock(&us->dev_mutex);
	dpm_ops->pm_state = -1;

	archos_flashdisk_off();

	hcd_bus_suspend(hdd->pusb_parent);
	msleep(10);
	archos_enable_ehci(0);
	dpm_ops->pm_state = PM_SUSPEND_STANDBY;
	mutex_unlock(&us->dev_mutex);

	return rc;
}

static void archos_flashdisk_wait_on( struct us_data *us )
{
	int wait_time;

	msleep(100);
	/* power to the drive ... */
	usbhdd_power(1);

	/* wait until the usb device comes back */
	wait_time = 300;
	do {
		msleep(100);
	} while (!us->associated && --wait_time);
}

static int archos_flashdisk_dpm_resume(struct hdd_dpm_ops *dpm_ops)
{
	struct archos_hdd *hdd = to_archos_hdd(dpm_ops);
	struct us_data *us = hdd->us;

	//printk("archos_flashdisk_dpm_resume\n");
	dpm_ops->pm_state = -1;

	archos_enable_ehci( 1 );
	hcd_bus_resume(hdd->pusb_parent);

	archos_flashdisk_wait_on( us );

	if (us->associated) {
		msleep(100);
		dpm_ops->pm_state = PM_SUSPEND_ON;
	} else
		dpm_ops->pm_state = PM_SUSPEND_STANDBY;
	
	return 0;
}

static int archos_flashdisk_dpm_sync(struct hdd_dpm_ops *dpm_ops)
{
	struct archos_hdd *hdd = to_archos_hdd(dpm_ops);
	struct us_data *us = hdd->us;
	struct scsi_device *sdev = us->sdev;
	
	struct scsi_disk *sdkp;
	struct gendisk *gd;
	int i;

	if (sdev == NULL)
		return -ENODEV;

	sdkp = dev_get_drvdata(&sdev->sdev_gendev);
	if (sdkp == NULL)
		return -ENODEV;

	gd = sdkp->disk;
	for (i = gd->first_minor; i < gd->first_minor + gd->minors; i++) {
		struct block_device *bdev = bdget_disk(sdkp->disk, i);
		if (bdev) {
			sync_blockdev(bdev);
		}
	}

	return 0;
}

static unsigned long archos_flashdisk_dpm_iocount(struct hdd_dpm_ops *dpm_ops)
{
	struct archos_hdd *hdd = to_archos_hdd(dpm_ops);	

	/* return 0 if there is still a request pending - otherwise return the io_count,
	   which should increase with every transaction */
	return hdd->us->srb == NULL ? hdd->us->io_count : 0;
}

static void archos_flashdisk_pm_hook(struct us_data *us, int state)
{
	struct archos_hdd *hdd = us->extra;
	struct hdd_dpm_ops *dpm_ops = &hdd->dpm_ops;

	if (likely(state == US_RESUME)) {
		/* maybe the drive is in transition already,
		 * wait until it's complete */
		while (unlikely(hdd->dpm_ops.pm_state == -1))
			msleep(100);
		
		if (unlikely(hdd->dpm_ops.pm_state == PM_SUSPEND_STANDBY))
			archos_flashdisk_dpm_resume(dpm_ops);

	} else {
#if 0
		if (hdd->dpm_ops.pm_state == PM_SUSPEND_ON) {			
			dpm_ops->pm_state = -1;
			
			/* ... and switch off power */
			usbhdd_power(0);
			printk("suspend transition\n");

			msleep(100);

			//archos_enable_ehci( 0 );

			dpm_ops->pm_state = PM_SUSPEND_STANDBY;
		}
#endif
	}
}

static int archos_flashdisk_reset( struct us_data *us )
{
	archos_flashdisk_off();
	archos_flashdisk_wait_on( us );
	if (us->associated) {
		msleep(100);
		return 0;
	}
	return -ENODEV;
}

static void archos_flashdisk_exit(void* __hdd)
{
	struct archos_hdd *hdd = __hdd;
	
	hdd_dpm_unregister_dev(&hdd->dpm_ops);
}

int archos_flashdisk_init(struct us_data *us)
{
	struct archos_hdd *hdd;
	
	US_DEBUGP("%s called, bus_name: %s\n", __FUNCTION__, us->pusb_dev->bus->bus_name);
	
	/* register only devices on omap ehci usb bus */
	if (strncmp(us->pusb_dev->bus->bus_name, USB_HOST_NAME, strlen(USB_HOST_NAME)) != 0)
		return 0;
	
	hdd = kzalloc(sizeof (struct archos_hdd), GFP_KERNEL);
	if (hdd == NULL)
		return -ENOMEM;

	hdd->us               = us;
	hdd->dpm_ops.suspend  = archos_flashdisk_dpm_suspend;
	hdd->dpm_ops.resume   = archos_flashdisk_dpm_resume;
	hdd->dpm_ops.sync     = archos_flashdisk_dpm_sync;
	hdd->dpm_ops.get_iocount = archos_flashdisk_dpm_iocount;
	hdd->dpm_ops.pm_state = PM_SUSPEND_ON;
	hdd->dpm_ops.shutdown = 1;
	hdd->pusb_parent      = us->pusb_dev->parent;
	strlcpy(hdd->dpm_ops.name, "sda", sizeof(hdd->dpm_ops.name));

	us->extra_destructor  = archos_flashdisk_exit;
	us->extra             = hdd;
	us->suspend_resume_hook = archos_flashdisk_pm_hook;
	us->device_reattach   = archos_flashdisk_reset;

	return hdd_dpm_register_dev(&hdd->dpm_ops);
}
