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

struct archos_hdd {
	struct us_data *us;
	struct hdd_dpm_ops dpm_ops;
};

#define to_archos_hdd(p) container_of(p, struct archos_hdd, dpm_ops)

extern int bulk_cmd(struct us_data *us, u8 *cdb, int cdb_len, int is_write);

static void usb_stor_suspend_drive(struct us_data *us)
{
	int rc;

	u8 cdb[1] = { 0xE0 };

	rc = bulk_cmd(us, cdb, 1, 0);
	if (rc)
		printk("usb_stor_suspend_drive: rc=%i\n", rc);
}

static int archosg6gp_dpm_suspend(struct hdd_dpm_ops *dpm_ops)
{
	struct archos_hdd *hdd = to_archos_hdd(dpm_ops);
	struct us_data *us = hdd->us;
	int rc;
	
	usb_lock_device(us->pusb_dev);
	rc = usb_external_suspend_device(us->pusb_dev, PMSG_SUSPEND);
	usb_unlock_device(us->pusb_dev);

	return rc;
}

static int archosg6gp_dpm_resume(struct hdd_dpm_ops *dpm_ops)
{
	struct archos_hdd *hdd = to_archos_hdd(dpm_ops);
	struct us_data *us = hdd->us;
	int wait_time;

	dpm_ops->pm_state = -1;

	/* power to the drive ... */
	usbhdd_power(1);

	/* wait until the usb device comes back */
	wait_time = 20;
	do {
		msleep(500);
	} while (!us->associated && --wait_time);

	if (us->associated)
		dpm_ops->pm_state = PM_SUSPEND_ON;
	else
		dpm_ops->pm_state = PM_SUSPEND_STANDBY;
	
	return 0;
}

static int archosg6gp_dpm_sync(struct hdd_dpm_ops *dpm_ops)
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
		//printk("archosg6_dpm_sync: syncing %i:%i\n", gd->major, i);
		if (bdev)
			sync_blockdev(bdev);
		else
			printk("archosg6_dpm_sync: minor %i not found\n", i);
	}

	return 0;
}

static unsigned long archosg6gp_dpm_iocount(struct hdd_dpm_ops *dpm_ops)
{
	struct archos_hdd *hdd = to_archos_hdd(dpm_ops);
	
	/* return 0 if there is still a request pending - otherwise return the io_count,
	   which should increase with every transaction */
	return hdd->us->srb == NULL ? hdd->us->io_count : 0;
}

static void archosg6gp_hdd_pm_hook(struct us_data *us, int state)
{
	struct archos_hdd *hdd = us->extra;
	struct hdd_dpm_ops *dpm_ops = &hdd->dpm_ops;

	if (likely(state == US_RESUME)) {
		/* maybe the drive is in transition already,
		 * wait until it's complete */
		while (unlikely(hdd->dpm_ops.pm_state == -1))
			msleep(100);
		
		if (unlikely(hdd->dpm_ops.pm_state == PM_SUSPEND_STANDBY))
			archosg6gp_dpm_resume(dpm_ops);

	} else {
		if (hdd->dpm_ops.pm_state == PM_SUSPEND_ON) {			
			dpm_ops->pm_state = -1;
			
			/* send drive to sleep... */
			usb_stor_suspend_drive(us);

			/* ... and switch off power */
			usbhdd_power(0);

			dpm_ops->pm_state = PM_SUSPEND_STANDBY;
		}
	}
}

static void archosg6gp_hdd_exit(void* __hdd)
{
	struct archos_hdd *hdd = __hdd;
	
	hdd_dpm_unregister_dev(&hdd->dpm_ops);
}

int archosg6gp_hdd_init(struct us_data *us)
{
	struct archos_hdd *hdd;
	
	US_DEBUGP("%s called\n", __FUNCTION__);
	
	hdd = kzalloc(sizeof (struct archos_hdd), GFP_KERNEL);
	if (hdd == NULL)
		return -ENOMEM;

	hdd->us = us;
	hdd->dpm_ops.suspend = archosg6gp_dpm_suspend;
	hdd->dpm_ops.resume  = archosg6gp_dpm_resume;
	hdd->dpm_ops.sync    = archosg6gp_dpm_sync;
	hdd->dpm_ops.get_iocount = archosg6gp_dpm_iocount;
	hdd->dpm_ops.pm_state = PM_SUSPEND_ON;
	hdd->dpm_ops.shutdown = 1;
	strlcpy(hdd->dpm_ops.name, "sda", sizeof(hdd->dpm_ops.name));

	us->extra_destructor = archosg6gp_hdd_exit;
	us->extra = hdd;
	us->suspend_resume_hook = archosg6gp_hdd_pm_hook;

	return hdd_dpm_register_dev(&hdd->dpm_ops);
}
