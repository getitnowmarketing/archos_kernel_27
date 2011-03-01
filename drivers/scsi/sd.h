#ifndef _SCSI_DISK_H
#define _SCSI_DISK_H

#include <scsi/sd.h>

/*
 * A DIF-capable target device can be formatted with different
 * protection schemes.  Currently 0 through 3 are defined:
 *
 * Type 0 is regular (unprotected) I/O
 *
 * Type 1 defines the contents of the guard and reference tags
 *
 * Type 2 defines the contents of the guard and reference tags and
 * uses 32-byte commands to seed the latter
 *
 * Type 3 defines the contents of the guard tag only
 */

enum sd_dif_target_protection_types {
	SD_DIF_TYPE0_PROTECTION = 0x0,
	SD_DIF_TYPE1_PROTECTION = 0x1,
	SD_DIF_TYPE2_PROTECTION = 0x2,
	SD_DIF_TYPE3_PROTECTION = 0x3,
};

/*
 * Data Integrity Field tuple.
 */
struct sd_dif_tuple {
       __be16 guard_tag;	/* Checksum */
       __be16 app_tag;		/* Opaque storage */
       __be32 ref_tag;		/* Target LBA or indirect LBA */
};

#if defined(CONFIG_BLK_DEV_INTEGRITY)

extern void sd_dif_op(struct scsi_cmnd *, unsigned int, unsigned int);
extern void sd_dif_config_host(struct scsi_disk *);
extern int sd_dif_prepare(struct request *rq, sector_t, unsigned int);
extern void sd_dif_complete(struct scsi_cmnd *, unsigned int);

#else /* CONFIG_BLK_DEV_INTEGRITY */

#define sd_dif_op(a, b, c)			do { } while (0)
#define sd_dif_config_host(a)			do { } while (0)
#define sd_dif_prepare(a, b, c)			(0)
#define sd_dif_complete(a, b)			(0)

#endif /* CONFIG_BLK_DEV_INTEGRITY */

#endif /* _SCSI_DISK_H */
