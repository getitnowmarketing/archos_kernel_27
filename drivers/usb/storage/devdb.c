/* Small device database to allow non-removable devices to
 * disconnect and reconnect without the scsi layer noticing
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

#include "usb.h"
#include "transport.h"
#include "protocol.h"
#include "scsiglue.h"
#include "debug.h"
#include "devdb.h"

#define MAX_DEVDB_ENTRIES 32

struct devdb_entry {
	char devpath[32];
	struct us_data *us;
};

static struct devdb_entry devdb[MAX_DEVDB_ENTRIES];

struct us_data *usb_stor_devdb_find(const char *devpath)
{
	int n;
	struct us_data *us = NULL;
	
	for (n = 0; n < MAX_DEVDB_ENTRIES; n++) {
		if (strcmp(devdb[n].devpath, devpath) == 0 ) {
			us = devdb[n].us;
			break;
		}	
	}
	
	if (us == NULL)
		printk(KERN_DEBUG "devdb: devpath %s not found\n", devpath);
	
	return us;
}

int usb_stor_devdb_store(struct us_data *us, const char *devpath)
{
	int n;
	
	for (n = 0; n < MAX_DEVDB_ENTRIES; n++) {
		/* slot is not free, continue */
		if (devdb[n].us != NULL)
			continue;

		strlcpy(devdb[n].devpath, devpath, 32);
		devdb[n].us = us;
		break;
	}
	
	if (n == MAX_DEVDB_ENTRIES)
		printk(KERN_DEBUG "devdb: no free slots\n");

	return 0;
}

int usb_stor_devdb_remove(const struct us_data *us)
{
	int n;
	
	for (n = 0; n < MAX_DEVDB_ENTRIES; n++) {
		if (us == devdb[n].us) {
			devdb[n].us = NULL;
			devdb[n].devpath[0] = 0;
			break;
		}
	}
	
	if (n == MAX_DEVDB_ENTRIES)
		printk(KERN_DEBUG "devdb: entry not found\n");

	return 0;
}
