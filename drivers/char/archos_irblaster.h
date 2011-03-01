#ifndef _ARCHOS_IRBLAST_H
#define _ARCHOS_IRBLAST_H

typedef struct {				/* expanded results from a call to expand_ir() */
	unsigned short category;		/* device type*/
	unsigned short  details;		/* some status bits */
	unsigned short  carrier;		/* carrier frequency */
	unsigned short  OnceCtr;		/* non-repeat pulse-pair data counter */
	unsigned short  RepeatCtr;		/* repeated pulse-pair data counter */
	unsigned short  Data[256];		/* data (pulse-pairs) */
} irblast_data_t;

/* ioctls */

#define IRBLAST_SET_PRODUCT_ID		1
#define IRBLAST_SET_REPEATS		2
#define IRBLAST_DUMP_REGS		3
#define IRBLAST_WRITE_SINGLE		4
#define IRBLAST_SET_GIO			5
#define IRBLAST_CLEAR_GIO		6

#endif /* _ARCHOS_IRBLAST_H */
