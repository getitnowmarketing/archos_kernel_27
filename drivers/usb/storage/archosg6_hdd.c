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

#define ATA_PASSTHROUGH_CMD_SIZE 12
#define JMICRON_ATA_PASSTHROUGH  0xDF

//#define ATA_AAM_TEST

struct archos_hdd {
	struct us_data *us;
	struct usb_device *pusb_parent;
	struct hdd_dpm_ops dpm_ops;
};

extern int archos_enable_ehci( int enable );
extern void usbsata_controller_power( int on_off);

#define to_archos_hdd(p) container_of(p, struct archos_hdd, dpm_ops)

static int bulk_cmd_data(struct us_data *us, u8 *cdb, int cdb_len, int is_write, u8 *buf, unsigned int data_len)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	struct bulk_cs_wrap *bcs = (struct bulk_cs_wrap *) us->iobuf;
	unsigned int residue;
	unsigned int cswlen;
	unsigned int cbwlen = US_BULK_CB_WRAP_LEN;
	int result;
	int i;
	
	/* Take care of BULK32 devices; set extra byte to 0 */
	if ( unlikely(us->fflags & US_FL_BULK32)) {
		cbwlen = 32;
		us->iobuf[31] = 0;
	}

	/* set up the command wrapper */
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = cpu_to_le32(data_len);
	bcb->Flags = is_write ? 0 : 1 << 7;
	bcb->Tag = ++us->tag;
	bcb->Lun = 0; /* we don't know better */
	bcb->Length = cdb_len;

	/* copy the command payload */
	memset(bcb->CDB, 0, sizeof(bcb->CDB));
	memcpy(bcb->CDB, cdb, bcb->Length);

	/* send it to out endpoint */
	US_DEBUGP("Bulk Command S 0x%x T 0x%x L %d F %d Trg %d LUN %d CL %d\n",
			le32_to_cpu(bcb->Signature), bcb->Tag,
			le32_to_cpu(bcb->DataTransferLength), bcb->Flags,
			(bcb->Lun >> 4), (bcb->Lun & 0x0F), 
			bcb->Length);
			
	for (i = 0; i < bcb->Length; i++) {
		US_DEBUGPX(" %02x", bcb->CDB[i]);
	}
	US_DEBUGPX("\n");
	
	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe,
				bcb, cbwlen, NULL);
	US_DEBUGP("Bulk command transfer result=%d\n", result);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	/* DATA STAGE */
	/* send/receive data payload, if there is any */

	if (data_len) {
		int act_len;
		unsigned int pipe = is_write ? us->send_bulk_pipe : us->recv_bulk_pipe;
		udelay(125);
		result = usb_stor_bulk_transfer_buf(us, pipe,
					buf, data_len, &act_len);
		US_DEBUGP("Bulk data transfer result 0x%x act_len %d\n", result, act_len);
		if (result == USB_STOR_XFER_ERROR || result == USB_STOR_XFER_LONG)
			return USB_STOR_TRANSPORT_ERROR;
	}

	/* get CSW for device status */
	US_DEBUGP("Attempting to get CSW...\n");
	result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe,
				bcs, US_BULK_CS_WRAP_LEN, &cswlen);

	/* Some broken devices add unnecessary zero-length packets to the
	 * end of their data transfers.  Such packets show up as 0-length
	 * CSWs.  If we encounter such a thing, try to read the CSW again.
	 */
	if (result == USB_STOR_XFER_SHORT && cswlen == 0) {
		US_DEBUGP("Received 0-length CSW; retrying...\n");
		result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe,
				bcs, US_BULK_CS_WRAP_LEN, &cswlen);
	}

	/* did the attempt to read the CSW fail? */
	if (result == USB_STOR_XFER_STALLED) {

		/* get the status again */
		US_DEBUGP("Attempting to get CSW (2nd try)...\n");
		result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe,
				bcs, US_BULK_CS_WRAP_LEN, NULL);
	}

	/* if we still have a failure at this point, we're in trouble */
	US_DEBUGP("Bulk status result = %d\n", result);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	/* check bulk status */
	residue = le32_to_cpu(bcs->Residue);
	US_DEBUGP("Bulk Status S 0x%x T 0x%x R %u Stat 0x%x\n",
			le32_to_cpu(bcs->Signature), bcs->Tag, 
			residue, bcs->Status);
	if (bcs->Tag != us->tag || bcs->Status > US_BULK_STAT_PHASE) {
		US_DEBUGP("Bulk logical error\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	/* Some broken devices report odd signatures, so we do not check them
	 * for validity against the spec. We store the first one we see,
	 * and check subsequent transfers for validity against this signature.
	 */
	if (!us->bcs_signature) {
		us->bcs_signature = bcs->Signature;
		if (us->bcs_signature != cpu_to_le32(US_BULK_CS_SIGN))
			US_DEBUGP("Learnt BCS signature 0x%08X\n",
					le32_to_cpu(us->bcs_signature));
	} else if (bcs->Signature != us->bcs_signature) {
		US_DEBUGP("Signature mismatch: got %08X, expecting %08X\n",
			  le32_to_cpu(bcs->Signature),
			  le32_to_cpu(us->bcs_signature));
		return USB_STOR_TRANSPORT_ERROR;
	}

	/* based on the status code, we report good or bad */
	switch (bcs->Status) {
	case US_BULK_STAT_OK:
		/* command good -- note that data could be short */
		return USB_STOR_TRANSPORT_GOOD;

	case US_BULK_STAT_FAIL:
		/* command failed */
		return USB_STOR_TRANSPORT_FAILED;

	case US_BULK_STAT_PHASE:
		/* phase error -- note that a transport reset will be
		 * invoked by the invoke_transport() function
		 */
		return USB_STOR_TRANSPORT_ERROR;
	}

	/* we should never get here, but if we do, we're in trouble */
	return USB_STOR_TRANSPORT_ERROR;
}

int bulk_cmd(struct us_data *us, u8 *cdb, int cdb_len, int is_write )
{
	return bulk_cmd_data( us, cdb, cdb_len, is_write, NULL, 0 );
}

/* Send an ATA command via the passthrough interface */
static int bulk_ata_passthrough_nondata(struct us_data *us, u8 *ata_cmd)
{
	u8 cmd_buf[ATA_PASSTHROUGH_CMD_SIZE];
	u16 data_len = 0;
		
	memset(cmd_buf, 0xFF, ATA_PASSTHROUGH_CMD_SIZE);
	
	cmd_buf[0] = JMICRON_ATA_PASSTHROUGH;
	cmd_buf[1] = 0x10;
	cmd_buf[3] = (data_len & 0xFF00) >> 8;
	cmd_buf[4] =  data_len & 0xFF;
	memcpy(&cmd_buf[5], ata_cmd, 7);

	return bulk_cmd(us, cmd_buf, ATA_PASSTHROUGH_CMD_SIZE, 0);
}

static int bulk_ata_passthrough_data(struct us_data *us, u8 *ata_cmd, 
					int is_write, u8 *buf, u16 data_len )
{
	u8 cmd_buf[ATA_PASSTHROUGH_CMD_SIZE];
		
	memset(cmd_buf, 0xFF, ATA_PASSTHROUGH_CMD_SIZE);
	
	cmd_buf[0] = JMICRON_ATA_PASSTHROUGH;
	cmd_buf[1] = is_write ? 0x0 : 0x10;
	cmd_buf[3] = (data_len & 0xFF00) >> 8;
	cmd_buf[4] =  data_len & 0xFF;
	memcpy(&cmd_buf[5], ata_cmd, 7);

	return bulk_cmd_data(us, cmd_buf, ATA_PASSTHROUGH_CMD_SIZE, is_write, buf, data_len);
}

static int usb_stor_ata_sleep(struct us_data *us)
{
	u8 ata_cmd[7];
	
	US_DEBUGP("%s called\n", __FUNCTION__);

	memset(ata_cmd, 0, sizeof(ata_cmd));
	ata_cmd[6] = 0xE6; /* WIN_SLEEPNOW1 */
	
	return bulk_ata_passthrough_nondata(us, ata_cmd);
}

static int archosg6_dpm_suspend(struct hdd_dpm_ops *dpm_ops)
{
	struct archos_hdd *hdd = to_archos_hdd(dpm_ops);
	struct us_data *us = hdd->us;
	int rc = 0;
	
	printk(KERN_DEBUG "archosg6_dpm_suspend\n");
	mutex_lock(&us->dev_mutex);
	dpm_ops->pm_state = -1;
	usb_stor_ata_sleep(us);
	msleep(500);		// let drive enought time to finish sleep process
	usbsata_power(0);
	msleep(100);
	hcd_bus_suspend(hdd->pusb_parent);
	msleep(10);
	archos_enable_ehci(0);
	dpm_ops->pm_state = PM_SUSPEND_STANDBY;
	mutex_unlock(&us->dev_mutex);
	
	return rc;
}

static int archosg6_dpm_resume(struct hdd_dpm_ops *dpm_ops)
{
	struct archos_hdd *hdd = to_archos_hdd(dpm_ops);
	struct us_data *us = hdd->us;	
	int wait_time;

	printk("archosg6_dpm_resume\n");
	
	dpm_ops->pm_state = -1;

	archos_enable_ehci( 1 );

	hcd_bus_resume(hdd->pusb_parent);
	msleep(100);
	/* power to the drive ... */
	usbsata_power(1);

	/* wait until the usb device comes back */
	wait_time = 200;
	do {
		msleep(100);
	} while (!us->associated && --wait_time);

	if (us->associated) {
		msleep(100);
		dpm_ops->pm_state = PM_SUSPEND_ON;
	} else
		dpm_ops->pm_state = PM_SUSPEND_STANDBY;
	
	return 0;
}

static int archosg6_dpm_sync(struct hdd_dpm_ops *dpm_ops)
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

static unsigned long archosg6_dpm_iocount(struct hdd_dpm_ops *dpm_ops)
{
	struct archos_hdd *hdd = to_archos_hdd(dpm_ops);
	
	/* return 0 if there is still a request pending - otherwise return the io_count,
	   which should increase with every transaction */
	return hdd->us->srb == NULL ? hdd->us->io_count : 0;
}

/*
 * central power management hook. caller holds us->dev_lock
 */
static void archosg6_hdd_pm_hook(struct us_data *us, int state)
{
	struct archos_hdd *hdd = us->extra;
	struct hdd_dpm_ops *dpm_ops = &hdd->dpm_ops;

	//US_DEBUGP("%s called\n", __FUNCTION__);
	
	if (state == US_RESUME) {
		/* maybe the drive is in transition already,
		 * wait until it's complete */
		while (unlikely(hdd->dpm_ops.pm_state == -1))
			msleep(100);

		if (hdd->dpm_ops.pm_state == PM_SUSPEND_STANDBY) {
			printk("hdd_pm_hook: resuming\n");
			archosg6_dpm_resume(dpm_ops);
			
		}

	}
}

static int archosg6_sata_reset( struct us_data *us )
{
	int wait_time;
	
	usbsata_controller_power(0);
	msleep(100);
	printk("power on\n");
	usbsata_controller_power(1);

	wait_time = 200;
	do {
		msleep(100);
	} while (!us->associated && --wait_time);

	printk("done waiting...\n");
	if (us->associated) {
		msleep(100);
		return 0;
	}
	return -ENODEV;
}

static void archosg6_hdd_exit(void* __hdd)
{
	struct archos_hdd *hdd = __hdd;
	
	hdd_dpm_unregister_dev(&hdd->dpm_ops);
}

int archosg6_hdd_init(struct us_data *us)
{
	struct archos_hdd *hdd;
	
	US_DEBUGP("%s called\n", __FUNCTION__);
	
	hdd = kzalloc(sizeof (struct archos_hdd), GFP_KERNEL);
	if (hdd == NULL)
		return -ENOMEM;

	hdd->us = us;
	hdd->dpm_ops.suspend  = archosg6_dpm_suspend;
	hdd->dpm_ops.resume   = archosg6_dpm_resume;
	hdd->dpm_ops.sync     = archosg6_dpm_sync;
	hdd->dpm_ops.get_iocount = archosg6_dpm_iocount;
	hdd->dpm_ops.pm_state = PM_SUSPEND_ON;
	hdd->dpm_ops.shutdown = 1;
	hdd->pusb_parent      = us->pusb_dev->parent;
	strlcpy(hdd->dpm_ops.name, "sda", sizeof(hdd->dpm_ops.name));

	us->extra_destructor  = archosg6_hdd_exit;
	us->extra             = hdd;
	us->suspend_resume_hook = archosg6_hdd_pm_hook;
	us->device_reattach   = archosg6_sata_reset;

	return hdd_dpm_register_dev(&hdd->dpm_ops);
}
