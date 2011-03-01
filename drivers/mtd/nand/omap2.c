/*
 * drivers/mtd/nand/omap2.c
 *
 * Copyright (c) 2004 Texas Instruments, Jian Zhang <jzhang@ti.com>
 * Copyright (c) 2004 Micron Technology Inc.
 * Copyright (c) 2004 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>

#include <asm/dma.h>

#include <mach/gpmc.h>
#include <mach/nand.h>

#define GPMC_IRQ_STATUS		0x18
#define GPMC_ECC_CONFIG		0x1F4
#define GPMC_ECC_CONTROL	0x1F8
#define GPMC_ECC_SIZE_CONFIG	0x1FC
#define GPMC_ECC1_RESULT	0x200

#define	DRIVER_NAME	"omap2-nand"
#define	NAND_IO_SIZE	SZ_4K

#define	NAND_WP_ON	1
#define	NAND_WP_OFF	0
#define NAND_WP_BIT	0x00000010
#define WR_RD_PIN_MONITORING	0x00600000

#define	GPMC_BUF_FULL	0x00000001
#define	GPMC_BUF_EMPTY	0x00000000

#define NAND_Ecc_P1e		(1 << 0)
#define NAND_Ecc_P2e		(1 << 1)
#define NAND_Ecc_P4e		(1 << 2)
#define NAND_Ecc_P8e		(1 << 3)
#define NAND_Ecc_P16e		(1 << 4)
#define NAND_Ecc_P32e		(1 << 5)
#define NAND_Ecc_P64e		(1 << 6)
#define NAND_Ecc_P128e		(1 << 7)
#define NAND_Ecc_P256e		(1 << 8)
#define NAND_Ecc_P512e		(1 << 9)
#define NAND_Ecc_P1024e		(1 << 10)
#define NAND_Ecc_P2048e		(1 << 11)

#define NAND_Ecc_P1o		(1 << 16)
#define NAND_Ecc_P2o		(1 << 17)
#define NAND_Ecc_P4o		(1 << 18)
#define NAND_Ecc_P8o		(1 << 19)
#define NAND_Ecc_P16o		(1 << 20)
#define NAND_Ecc_P32o		(1 << 21)
#define NAND_Ecc_P64o		(1 << 22)
#define NAND_Ecc_P128o		(1 << 23)
#define NAND_Ecc_P256o		(1 << 24)
#define NAND_Ecc_P512o		(1 << 25)
#define NAND_Ecc_P1024o		(1 << 26)
#define NAND_Ecc_P2048o		(1 << 27)

#define TF(value)	(value ? 1 : 0)

#define P2048e(a)	(TF(a & NAND_Ecc_P2048e)	<< 0)
#define P2048o(a)	(TF(a & NAND_Ecc_P2048o)	<< 1)
#define P1e(a)		(TF(a & NAND_Ecc_P1e)		<< 2)
#define P1o(a)		(TF(a & NAND_Ecc_P1o)		<< 3)
#define P2e(a)		(TF(a & NAND_Ecc_P2e)		<< 4)
#define P2o(a)		(TF(a & NAND_Ecc_P2o)		<< 5)
#define P4e(a)		(TF(a & NAND_Ecc_P4e)		<< 6)
#define P4o(a)		(TF(a & NAND_Ecc_P4o)		<< 7)

#define P8e(a)		(TF(a & NAND_Ecc_P8e)		<< 0)
#define P8o(a)		(TF(a & NAND_Ecc_P8o)		<< 1)
#define P16e(a)		(TF(a & NAND_Ecc_P16e)		<< 2)
#define P16o(a)		(TF(a & NAND_Ecc_P16o)		<< 3)
#define P32e(a)		(TF(a & NAND_Ecc_P32e)		<< 4)
#define P32o(a)		(TF(a & NAND_Ecc_P32o)		<< 5)
#define P64e(a)		(TF(a & NAND_Ecc_P64e)		<< 6)
#define P64o(a)		(TF(a & NAND_Ecc_P64o)		<< 7)

#define P128e(a)	(TF(a & NAND_Ecc_P128e)		<< 0)
#define P128o(a)	(TF(a & NAND_Ecc_P128o)		<< 1)
#define P256e(a)	(TF(a & NAND_Ecc_P256e)		<< 2)
#define P256o(a)	(TF(a & NAND_Ecc_P256o)		<< 3)
#define P512e(a)	(TF(a & NAND_Ecc_P512e)		<< 4)
#define P512o(a)	(TF(a & NAND_Ecc_P512o)		<< 5)
#define P1024e(a)	(TF(a & NAND_Ecc_P1024e)	<< 6)
#define P1024o(a)	(TF(a & NAND_Ecc_P1024o)	<< 7)

#define P8e_s(a)	(TF(a & NAND_Ecc_P8e)		<< 0)
#define P8o_s(a)	(TF(a & NAND_Ecc_P8o)		<< 1)
#define P16e_s(a)	(TF(a & NAND_Ecc_P16e)		<< 2)
#define P16o_s(a)	(TF(a & NAND_Ecc_P16o)		<< 3)
#define P1e_s(a)	(TF(a & NAND_Ecc_P1e)		<< 4)
#define P1o_s(a)	(TF(a & NAND_Ecc_P1o)		<< 5)
#define P2e_s(a)	(TF(a & NAND_Ecc_P2e)		<< 6)
#define P2o_s(a)	(TF(a & NAND_Ecc_P2o)		<< 7)

#define P4e_s(a)	(TF(a & NAND_Ecc_P4e)		<< 0)
#define P4o_s(a)	(TF(a & NAND_Ecc_P4o)		<< 1)

static void omap_read_buf8(struct mtd_info *mtd, u_char *buf, int len);
static void omap_write_buf8(struct mtd_info *mtd, const u_char *buf, int len);
static void omap_read_buf16(struct mtd_info *mtd, u_char *buf, int len);
static void omap_write_buf16(struct mtd_info *mtd, const u_char *buf, int len);

#ifdef CONFIG_MACH_ARCHOS
/* Large Page x16 NAND device Layout */
static struct nand_ecclayout hw_nand_oob = {
	.eccbytes = 12,
	.eccpos = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13},
	.oobfree = {
		{.offset = 14,
		 .length = 50 
		} 
	} 
};
#endif

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL };
#endif

#ifdef CONFIG_MTD_NAND_OMAP_PREFETCH
static int use_prefetch = 1;

/* "modprobe ... use_prefetch=0" etc */
module_param(use_prefetch, bool, 0);
MODULE_PARM_DESC(use_prefetch, "enable/disable use of PREFETCH");

#ifdef CONFIG_MTD_NAND_OMAP_PREFETCH_DMA
static int use_dma = 1;

/* "modprobe ... use_dma=0" etc */
module_param(use_dma, bool, 0);
MODULE_PARM_DESC(use_dma, "enable/disable use of DMA");
#else
const int use_dma;
#endif
#else
const int use_prefetch;
const int use_dma;
#endif

#ifdef CONFIG_MTD_NAND_OMAP_OOB_ECC
	static int oob_swecc = 1;

	/* "modprobe ... use_hwecc=0" etc */
	module_param(oob_swecc, bool, 0);
	MODULE_PARM_DESC(oob_swecc, "enable/disable use of HW ECC");
#else
	const int oob_swecc = 0;
#endif

struct omap_nand_info {
	struct nand_hw_control		controller;
	struct omap_nand_platform_data	*pdata;
	struct mtd_info			mtd;
	struct mtd_partition		*parts;
	struct nand_chip		nand;
	struct platform_device		*pdev;

	int				gpmc_cs;
	unsigned long			phys_base;
	void __iomem			*gpmc_cs_baseaddr;
	void __iomem			*gpmc_baseaddr;
	void __iomem			*nand_pref_fifo_add;
	struct completion		comp;
	int				dma_ch;
};

#ifdef CONFIG_MTD_NAND_OMAP_OOB_ECC
/* this array has indices for OOB FREE bytes in OOB area */
unsigned int oob_free_index[40] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
				   15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
				   26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
				   37
				  };

/* this array has indices where OOB ECC bytes will be stored in OOB area */
unsigned int oob_ecc_index[12] = {52, 53, 54, 55, 56, 57, 58, 59};

/*
 * Pre-calculated 256-way 1 byte column parity
 */
static const u_char nand_ecc_precalc_table[] = {
	0x00, 0x55, 0x56, 0x03, 0x59, 0x0c, 0x0f, 0x5a, 0x5a, 0x0f, 0x0c, 0x59, 0x03, 0x56, 0x55, 0x00,
	0x65, 0x30, 0x33, 0x66, 0x3c, 0x69, 0x6a, 0x3f, 0x3f, 0x6a, 0x69, 0x3c, 0x66, 0x33, 0x30, 0x65,
	0x66, 0x33, 0x30, 0x65, 0x3f, 0x6a, 0x69, 0x3c, 0x3c, 0x69, 0x6a, 0x3f, 0x65, 0x30, 0x33, 0x66,
	0x03, 0x56, 0x55, 0x00, 0x5a, 0x0f, 0x0c, 0x59, 0x59, 0x0c, 0x0f, 0x5a, 0x00, 0x55, 0x56, 0x03,
	0x69, 0x3c, 0x3f, 0x6a, 0x30, 0x65, 0x66, 0x33, 0x33, 0x66, 0x65, 0x30, 0x6a, 0x3f, 0x3c, 0x69,
	0x0c, 0x59, 0x5a, 0x0f, 0x55, 0x00, 0x03, 0x56, 0x56, 0x03, 0x00, 0x55, 0x0f, 0x5a, 0x59, 0x0c,
	0x0f, 0x5a, 0x59, 0x0c, 0x56, 0x03, 0x00, 0x55, 0x55, 0x00, 0x03, 0x56, 0x0c, 0x59, 0x5a, 0x0f,
	0x6a, 0x3f, 0x3c, 0x69, 0x33, 0x66, 0x65, 0x30, 0x30, 0x65, 0x66, 0x33, 0x69, 0x3c, 0x3f, 0x6a,
	0x6a, 0x3f, 0x3c, 0x69, 0x33, 0x66, 0x65, 0x30, 0x30, 0x65, 0x66, 0x33, 0x69, 0x3c, 0x3f, 0x6a,
	0x0f, 0x5a, 0x59, 0x0c, 0x56, 0x03, 0x00, 0x55, 0x55, 0x00, 0x03, 0x56, 0x0c, 0x59, 0x5a, 0x0f,
	0x0c, 0x59, 0x5a, 0x0f, 0x55, 0x00, 0x03, 0x56, 0x56, 0x03, 0x00, 0x55, 0x0f, 0x5a, 0x59, 0x0c,
	0x69, 0x3c, 0x3f, 0x6a, 0x30, 0x65, 0x66, 0x33, 0x33, 0x66, 0x65, 0x30, 0x6a, 0x3f, 0x3c, 0x69,
	0x03, 0x56, 0x55, 0x00, 0x5a, 0x0f, 0x0c, 0x59, 0x59, 0x0c, 0x0f, 0x5a, 0x00, 0x55, 0x56, 0x03,
	0x66, 0x33, 0x30, 0x65, 0x3f, 0x6a, 0x69, 0x3c, 0x3c, 0x69, 0x6a, 0x3f, 0x65, 0x30, 0x33, 0x66,
	0x65, 0x30, 0x33, 0x66, 0x3c, 0x69, 0x6a, 0x3f, 0x3f, 0x6a, 0x69, 0x3c, 0x66, 0x33, 0x30, 0x65,
	0x00, 0x55, 0x56, 0x03, 0x59, 0x0c, 0x0f, 0x5a, 0x5a, 0x0f, 0x0c, 0x59, 0x03, 0x56, 0x55, 0x00
};

/*
 * cntbits - Counts number of 1 bits
 * byte:	data byte for which number of 1's has to be counted
 * return:	number of 1's in byte
 */
static inline int cntbits(uint32_t byte)
{
	int count = 0;

	for ( ; byte; count++)
		byte &= byte - 1;
	return count;
}

/*
 * nand_calculate_oob_ecc - Calculate 2-byte ECC for each of the 4 segments
 * in a 36-byte area of the OOB.
 * @ecc_code:   buffer for ECC
 * @dat:        data pointer
 */
void nand_calculate_oob_ecc(const unsigned char *dat, unsigned char *ecc_code)
{
	unsigned char idx, reg1, reg2, reg3, tmp;
	int i = 0, j = 0, b_count = 0;
	u16 eccbytes = 2, ecc_index = 0;

	/* calculate ecc for all 36 bye of oob, in segment of 9 each time */
	for (i = 0; i < 4; i++, ecc_index += eccbytes) {

		/* Initialize variables */
		reg1 = reg2 = reg3 = 0;

		/* Build up column parity */
		for (j = 0; j < 9; j++) {

			/* Get CP0 - CP5 from table */
			idx = nand_ecc_precalc_table[dat[oob_free_index[b_count]]];
			reg1 ^= (idx & 0x3f);

			/* All bit XOR = 1 ? */
			if (idx & 0x40) {
				reg3 ^= (unsigned char) j;
				reg2 ^= ~((unsigned char) j);
			}
			/*increament byte count*/
			b_count++;
		}
		/* Create non-inverted ECC code from line parity */
		tmp  = (reg3 & 0x08) << 4; /* B3 -> B7 */
		tmp |= (reg2 & 0x08) << 3; /* B3 -> B6 */
		tmp |= (reg3 & 0x04) << 3; /* B2 -> B5 */
		tmp |= (reg2 & 0x04) << 2; /* B2 -> B4 */
		tmp |= (reg3 & 0x02) << 2; /* B1 -> B3 */
		tmp |= (reg2 & 0x02) << 1; /* B1 -> B2 */
		tmp |= (reg3 & 0x01) << 1; /* B0 -> B1 */
		tmp |= (reg2 & 0x01) << 0; /* B0 -> B0 */

		/* Copy inverted ECC code to passed array */
		ecc_code[ecc_index] = ~tmp;
		ecc_code[ecc_index + 1] = ((~reg1) << 2) | 0x03;
	}
}

/*
 * nand_correct_oob_ecc_data -   Detect and correct bit error(s)
 * @read_sw_ecc:                ECC from the chip
 * @calc_sw_ecc:                the ECC calculated from raw data
 * @dat:                        data pointer
 * Detect and correct a 1 bit error for 40 byte block
 */
int nand_correct_oob_ecc_data(struct mtd_info *mtd, unsigned char *dat,
			unsigned char *read_ecc, unsigned char *calc_ecc)
{
	unsigned char s0, s1;
	unsigned int i, ret = 0;
	u16 eccbytes = 2, ecc_index = 0;

	/* check if all ECC bytes are same */
	if (memcmp(calc_ecc, read_ecc, 8) == 0)
		return ret;

	/* check ecc for all 36 bye of oob, in segment of 9 each time */
	for (i = 0; i < 4; i++, ecc_index += eccbytes) {

		s0 = calc_ecc[ecc_index] ^ read_ecc[ecc_index];
		s1 = calc_ecc[ecc_index + 1] ^ read_ecc[ecc_index + 1];

		if ((s0 | s1) == 0)
			continue;

		/* Check and correct data byte for a single bit error */
		if (((s0 ^ (s0 >> 1)) & 0x55) == 0x55 &&
		    ((s1 ^ (s1 >> 1)) & 0x54) == 0x54) {

			uint32_t byteoffs, bitnum, bytenum = 0;

			byteoffs = (s0 >> 4) & 0x08;
			byteoffs |= (s0 >> 3) & 0x04;
			byteoffs |= (s0 >> 2) & 0x02;
			byteoffs |= (s0 >> 1) & 0x01;

			bitnum = (s1 >> 5) & 0x04;
			bitnum |= (s1 >> 4) & 0x02;
			bitnum |= (s1 >> 3) & 0x01;

			bytenum = (i*9) + byteoffs;

			dat[oob_free_index[bytenum]] ^= (1 << bitnum);
			ret++;
		}
		/* if there is only one '1' bit present in s0 and s1
		 * collectively, it indicates there was a single bit
		 * error in read ECC
		 */
		else if (cntbits(s0 | ((uint32_t)s1 << 8)) == 1)
			ret++;
		else {
			printk(KERN_INFO "ECC UNCORRECTED_ERROR IN OOB AREA\n");
			return -1;
		}
	}
	return ret;
}
#endif

/*
 * omap_nand_wp - This function enable or disable the Write Protect feature on
 * NAND device
 * @mtd: MTD device structure
 * @mode: WP ON/OFF
 */
static void omap_nand_wp(struct mtd_info *mtd, int mode)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);

	unsigned long config = __raw_readl(info->gpmc_baseaddr + GPMC_CONFIG);

	if (mode)
		config &= ~(NAND_WP_BIT);	/* WP is ON */
	else
		config |= (NAND_WP_BIT);	/* WP is OFF */

	__raw_writel(config, (info->gpmc_baseaddr + GPMC_CONFIG));
}

/*
 * hardware specific access to control-lines
 * NOTE: boards may use different bits for these!!
 *
 * ctrl:
 * NAND_NCE: bit 0 - don't care
 * NAND_CLE: bit 1 -> Command Latch
 * NAND_ALE: bit 2 -> Address Latch
 */
static void omap_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct omap_nand_info *info = container_of(mtd,
					struct omap_nand_info, mtd);
	switch (ctrl) {
	case NAND_CTRL_CHANGE | NAND_CTRL_CLE:
		info->nand.IO_ADDR_W = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_COMMAND;
		info->nand.IO_ADDR_R = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		break;

	case NAND_CTRL_CHANGE | NAND_CTRL_ALE:
		info->nand.IO_ADDR_W = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_ADDRESS;
		info->nand.IO_ADDR_R = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		break;

	case NAND_CTRL_CHANGE | NAND_NCE:
		info->nand.IO_ADDR_W = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		info->nand.IO_ADDR_R = info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_DATA;
		break;
	}

	if (cmd != NAND_CMD_NONE)
		__raw_writeb(cmd, info->nand.IO_ADDR_W);
}

#ifdef CONFIG_MTD_NAND_OMAP_PREFETCH_DMA
/*
 * omap_nand_dma_cb: callback on the completion of dma transfer
 * @lch: logical channel
 * @ch_satuts: channel status
 * @data: pointer to completion data structure
 */
static void omap_nand_dma_cb(int lch, u16 ch_status, void *data)
{
	complete((struct completion *) data);
}

/*
 * omap_nand_dma_transfer: configer and start dma transfer
 * @mtd: MTD device structure
 * @addr: virtual address in RAM of source/destination
 * @len: number of data bytes to be transferred
 * @is_write: flag for read/write operation
 */
static inline int omap_nand_dma_transfer(struct mtd_info *mtd, void *addr,
					unsigned int len, int is_write)
{
	struct omap_nand_info *info = container_of(mtd,
					struct omap_nand_info, mtd);
	uint32_t prefetch_status = 0;
	enum dma_data_direction dir = is_write ? DMA_TO_DEVICE :
							DMA_FROM_DEVICE;
	dma_addr_t dma_addr;

	/* The fifo depth is 64 bytes. We have a sync at each frame and frame
	 * length is 64 bytes.
	 */
	int buf_len = len/64;

	if (addr >= high_memory) {
		struct page *p1;

		if (((size_t)addr & PAGE_MASK) !=
			((size_t)(addr + len - 1) & PAGE_MASK))
			goto out_copy;
		p1 = vmalloc_to_page(addr);
		if (!p1)
			goto out_copy;
		addr = page_address(p1) + ((size_t)addr & ~PAGE_MASK);
	}

	dma_addr = dma_map_single(&info->pdev->dev, addr, len, dir);
	if (dma_mapping_error(&info->pdev->dev, dma_addr)) {
		dev_err(&info->pdev->dev,
			"Couldn't DMA map a %d byte buffer\n", len);
		goto out_copy;
	}

	if (is_write) {
	    omap_set_dma_dest_params(info->dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
						info->phys_base, 0, 0);
	    omap_set_dma_dest_burst_mode(info->dma_ch, OMAP_DMA_DATA_BURST_16);
	    omap_set_dma_src_params(info->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
							dma_addr, 0, 0);
	    omap_set_dma_src_burst_mode(info->dma_ch, OMAP_DMA_DATA_BURST_16);
	    omap_set_dma_transfer_params(info->dma_ch, OMAP_DMA_DATA_TYPE_S32,
					0x10, buf_len, OMAP_DMA_SYNC_FRAME,
					OMAP24XX_DMA_GPMC, OMAP_DMA_DST_SYNC);
	} else {
	    omap_set_dma_src_params(info->dma_ch, 0, OMAP_DMA_AMODE_CONSTANT,
						info->phys_base, 0, 0);
	    omap_set_dma_src_burst_mode(info->dma_ch, OMAP_DMA_DATA_BURST_16);
	    omap_set_dma_dest_params(info->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
							dma_addr, 0, 0);
	    omap_set_dma_dest_burst_mode(info->dma_ch, OMAP_DMA_DATA_BURST_16);
	    omap_set_dma_transfer_params(info->dma_ch, OMAP_DMA_DATA_TYPE_S32,
					0x10, buf_len, OMAP_DMA_SYNC_FRAME,
					OMAP24XX_DMA_GPMC, OMAP_DMA_SRC_SYNC);
	}
	/*  configure and start prefetch transfer */
	gpmc_prefetch_start(info->gpmc_cs, 0x1, len, is_write);
	init_completion(&info->comp);

	omap_start_dma(info->dma_ch);

	/* setup and start DMA using dma_addr */
	wait_for_completion(&info->comp);

	while (0x3fff & (prefetch_status = gpmc_prefetch_status()))
		;
	/* disable and stop the PFPW engine */
	gpmc_prefetch_stop();

	dma_unmap_single(&info->pdev->dev, dma_addr, len, dir);
	return 0;

out_copy:
	if (info->nand.options & NAND_BUSWIDTH_16)
		is_write == 0 ? omap_read_buf16(mtd, (u_char *) addr, len)
			: omap_write_buf16(mtd, (u_char *) addr, len);
	else
		is_write == 0 ? omap_read_buf8(mtd, (u_char *) addr, len)
			: omap_write_buf8(mtd, (u_char *) addr, len);
	return 0;
}
#else
static void omap_nand_dma_cb(int lch, u16 ch_status, void *data) {}
static inline int omap_nand_dma_transfer(struct mtd_info *mtd, void *addr,
					unsigned int len, int is_write)
{
	return 0;
}
#endif

#ifdef CONFIG_MTD_NAND_OMAP_OOB_ECC
static void omap_do_read_oob_ecc(struct mtd_info *mtd, u_char *buf)
{
	struct omap_nand_info *info = container_of(mtd,
					struct omap_nand_info, mtd);
	int i = 0, stat = 0;
	uint8_t *ecc_calc = info->nand.buffers->ecccalc;
	uint8_t *ecc_code = info->nand.buffers->ecccode;

	/* if read size is equal to mtd->oobsize, this is OOB area
	 * read, calculate ECC for this and save it in ecc_calc array
	 */
	nand_calculate_oob_ecc(buf, &ecc_calc[info->nand.ecc.total]);

	/* copy ECC bytes for OOB area to ecc_code array */
	for (i = 0; i < 8; i++)
		ecc_code[info->nand.ecc.total+i] = buf[oob_ecc_index[i]];

	/* check read ECC (ecc_code) and calculated ECC (ecc_calc),
	 * and correct the data for 1 bit error in each segment
	 * (4 segments) of 9 bytes
	 */
	stat = nand_correct_oob_ecc_data(mtd, buf,
					&ecc_code[info->nand.ecc.total],
					&ecc_calc[info->nand.ecc.total]);
	if (stat == -1)
		mtd->ecc_stats.failed++;
	else
		mtd->ecc_stats.corrected += stat;
}

static void omap_do_write_oob_ecc(struct mtd_info *mtd, const u_char * buf)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	struct nand_chip *nand = mtd->priv;
	int i = 0;
	uint8_t *ecc_calc = info->nand.buffers->ecccalc;

	/* if read size is equal to mtd->oobsize, this is oob
	 * area read, calculate ECC for this and save it in
	 * ecc_calc array
	 */
	nand_calculate_oob_ecc(buf, &ecc_calc[info->nand.ecc.total]);

	/* copy calculated ECC bytes for OOB area to
	 * info->nand.oob_poi at correct byte positions
	 * indicated by oob_ecc_index array, before write
	 * operation
	 */
	for (i = 0; i < 8; i++)
		nand->oob_poi[oob_ecc_index[i]] = ecc_calc[info->nand.ecc.total+i];
}
#else
static void omap_do_read_oob_ecc(struct mtd_info *mtd, u_char *buf)
{
}
static void omap_do_write_oob_ecc(struct mtd_info *mtd, const u_char * buf)
{
}
#endif

/*
 * omap_read_buf8 - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf8(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *nand = mtd->priv;
	u_char *p = (u_char *)buf;
	int size = len;

	while (len--)
		*p++ = __raw_readb(nand->IO_ADDR_R);

	if (oob_swecc && mtd->oobsize == size)
		 omap_do_read_oob_ecc(mtd, buf);
}

/*
 * omap_write_buf8 - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf8(struct mtd_info *mtd, const u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	u_char *p = (u_char *)buf;

	if (oob_swecc && mtd->oobsize == len)
		omap_do_write_oob_ecc(mtd, buf);

	while (len--) {
		writeb(*p++, info->nand.IO_ADDR_W);

		while (GPMC_BUF_EMPTY == (readl(info->gpmc_baseaddr +
						GPMC_STATUS) & GPMC_BUF_FULL));
	}
}

/*
 * omap_read_buf16 - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf16(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *nand = mtd->priv;
	int size = len;

	__raw_readsw(nand->IO_ADDR_R, buf, len / 2);

	if (oob_swecc && mtd->oobsize == size)
		 omap_do_read_oob_ecc(mtd, buf);
}

/*
 * omap_write_buf16 - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf16(struct mtd_info *mtd, const u_char * buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	u16 *p = (u16 *) buf;

	if (oob_swecc && mtd->oobsize == len)
		omap_do_write_oob_ecc(mtd, buf);

	/* FIXME try bursts of writesw() or DMA ... */
	len >>= 1;

	while (len--) {
		writew(*p++, info->nand.IO_ADDR_W);

		while (GPMC_BUF_EMPTY == (readl(info->gpmc_baseaddr +
						GPMC_STATUS) & GPMC_BUF_FULL));
	}
}

/*
 * omap_read_buf_pref - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: buffer to store date
 * @len: number of bytes to read
 */
static void omap_read_buf_pref(struct mtd_info *mtd, u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	uint32_t prefetch_status = 0, read_count = 0;
	u32 *p = (u32 *)buf;
	int size = len;

	if ((use_dma && len <= mtd->oobsize) || (!use_dma)) {

		/* take care of subpage reads */
		for (; len % 4 != 0; ) {
			*buf++ = __raw_readb(info->nand.IO_ADDR_R);
			len--;
		}
		p = (u32 *) buf;

		/* configure and start prefetch transfer */
		gpmc_prefetch_start(info->gpmc_cs, 0x0, len, 0x0);

		do {
			prefetch_status = gpmc_prefetch_status();
			read_count = ((prefetch_status >> 24) & 0x7F) >> 2;
			__raw_readsl(info->nand_pref_fifo_add, p,
							read_count);
			p += read_count;
			len -= read_count << 2;
		} while (len);

		/* disable and stop the PFPW engine */
		gpmc_prefetch_stop();
	} else if (use_dma) {
		if (info->dma_ch >= 0)
			/* start transfer in DMA mode */
			omap_nand_dma_transfer(mtd, p, len, 0x0);
	}
	if (oob_swecc && mtd->oobsize == size)
		 omap_do_read_oob_ecc(mtd, buf);
}

/*
 * omap_write_buf_pref - write buffer to NAND controller
 * @mtd: MTD device structure
 * @buf: data buffer
 * @len: number of bytes to write
 */
static void omap_write_buf_pref(struct mtd_info *mtd,
					const u_char *buf, int len)
{
	struct omap_nand_info *info = container_of(mtd,
						struct omap_nand_info, mtd);
	uint32_t prefetch_status = 0, write_count = 0;
	int i = 0;
	u16 *p = (u16 *) buf;

	if (oob_swecc && mtd->oobsize == len)
		omap_do_write_oob_ecc(mtd, buf);

	if ((use_dma && len <= mtd->oobsize) || (!use_dma)) {

		/* take care of subpage writes */
		if (len % 2 != 0) {
			writeb(*buf, info->nand.IO_ADDR_R);
			p = (u16 *)(buf + 1);
			len--;
		}

		/*  configure and start prefetch transfer */
		gpmc_prefetch_start(info->gpmc_cs, 0x0, len, 0x1);

		prefetch_status = gpmc_prefetch_status();
		while (prefetch_status & 0x3FFF) {
			write_count = ((prefetch_status >> 24) & 0x7F) >> 1;
			for (i = 0; (i < write_count) && len; i++, len -= 2)
				__raw_writew(*p++, info->nand_pref_fifo_add);
				prefetch_status = gpmc_prefetch_status();
		}

		/* disable and stop the PFPW engine */
		gpmc_prefetch_stop();
	} else if (use_dma) {
		if (info->dma_ch >= 0)
			/* start transfer in DMA mode */
			omap_nand_dma_transfer(mtd, p, len, 0x1);
	}
}

/*
 * omap_verify_buf - Verify chip data against buffer
 * @mtd: MTD device structure
 * @buf: buffer containing the data to compare
 * @len: number of bytes to compare
 */
static int omap_verify_buf(struct mtd_info *mtd, const u_char * buf, int len)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	u16 *p = (u16 *) buf;

	len >>= 1;

	while (len--) {

		if (*p++ != cpu_to_le16(readw(info->nand.IO_ADDR_R)))
			return -EFAULT;
	}

	return 0;
}

#ifdef CONFIG_MTD_NAND_OMAP_HWECC
/*
 * omap_hwecc_init-Initialize the Hardware ECC for NAND flash 
 * in GPMC controller
 * @mtd: MTD device structure
 */
static void omap_hwecc_init(struct mtd_info *mtd)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	register struct nand_chip *chip = mtd->priv;
	unsigned long val = 0x0;

	/* Read from ECC Control Register */
	val = __raw_readl(info->gpmc_baseaddr + GPMC_ECC_CONTROL);
	/* Clear all ECC | Enable Reg1 */
	val = ((0x00000001<<8) | 0x00000001);
	__raw_writel(val, info->gpmc_baseaddr + GPMC_ECC_CONTROL);

	/* Read from ECC Size Config Register */
	val = __raw_readl(info->gpmc_baseaddr + GPMC_ECC_SIZE_CONFIG);
	/* ECCSIZE1=512 | Select eccResultsize[0-3] */
	val = ((((chip->ecc.size >> 1) - 1) << 22) | (0x0000000F));
	__raw_writel(val, info->gpmc_baseaddr + GPMC_ECC_SIZE_CONFIG);
}

/*
 * gen_true_ecc - This function will generate true ECC value, which can be used
 * when correcting data read from NAND flash memory core
 * @ecc_buf: buffer to store ecc code
 */
#ifndef CONFIG_MACH_ARCHOS
static void gen_true_ecc(u8 *ecc_buf)
{
	u32 tmp = ecc_buf[0] | (ecc_buf[1] << 16) |
		((ecc_buf[2] & 0xF0) << 20) | ((ecc_buf[2] & 0x0F) << 8);

#ifdef CONFIG_MTD_NAND_ECC_SMC
	ecc_buf[0] = ~(P64o(tmp) | P64e(tmp) | P32o(tmp) | P32e(tmp) |
			P16o(tmp) | P16e(tmp) | P8o(tmp) | P8e(tmp));
	ecc_buf[1] = ~(P1024o(tmp) | P1024e(tmp) | P512o(tmp) | P512e(tmp) |
			P256o(tmp) | P256e(tmp) | P128o(tmp) | P128e(tmp));
#else
	ecc_buf[0] = ~(P1024o(tmp) | P1024e(tmp) | P512o(tmp) | P512e(tmp) |
			P256o(tmp) | P256e(tmp) | P128o(tmp) | P128e(tmp));
	ecc_buf[1] = ~(P64o(tmp) | P64e(tmp) | P32o(tmp) | P32e(tmp) |
			P16o(tmp) | P16e(tmp) | P8o(tmp) | P8e(tmp));
#endif
	ecc_buf[2] = ~(P4o(tmp) | P4e(tmp) | P2o(tmp) | P2e(tmp) | P1o(tmp) |
			P1e(tmp) | P2048o(tmp) | P2048e(tmp));
}

#else

static uint32_t gen_true_ecc(uint8_t *ecc_buf)
{
	return ecc_buf[0] | (ecc_buf[1] << 16) | ((ecc_buf[2] & 0xF0) << 20) |
		((ecc_buf[2] & 0x0F) << 8);
}
#endif
/*
 * omap_calcuate_ecc - Generate non-inverted ECC bytes.
 * Using noninverted ECC can be considered ugly since writing a blank
 * page ie. padding will clear the ECC bytes. This is no problem as long
 * nobody is trying to write data on the seemingly unused page. Reading
 * an erased page will produce an ECC mismatch between generated and read
 * ECC bytes that has to be dealt with separately.
 * @mtd: MTD device structure
 * @dat: The pointer to data on which ecc is computed
 * @ecc_code: The ecc_code buffer
 */
static int omap_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				u_char *ecc_code)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	unsigned long val = 0x0;
	unsigned long reg;

	/* Start Reading from HW ECC1_Result = 0x200 */
	reg = (unsigned long)(info->gpmc_baseaddr + GPMC_ECC1_RESULT);
	val = __raw_readl(reg);
	*ecc_code++ = val;          /* P128e, ..., P1e */
	*ecc_code++ = val >> 16;    /* P128o, ..., P1o */
	/* P2048o, P1024o, P512o, P256o, P2048e, P1024e, P512e, P256e */
	*ecc_code++ = ((val >> 8) & 0x0f) | ((val >> 20) & 0xf0);
	reg += 4;
#ifndef CONFIG_MACH_ARCHOS
	gen_true_ecc(ecc_code-3);
#endif

	return 0;
}

/*
 * omap_enable_hwecc - This function enables the hardware ecc functionality
 * @mtd: MTD device structure
 * @mode: Read/Write mode
 */
static void omap_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	register struct nand_chip *chip = mtd->priv;
	unsigned int dev_width = (chip->options & NAND_BUSWIDTH_16) ? 1 : 0;
	unsigned long val = __raw_readl(info->gpmc_baseaddr + GPMC_ECC_CONFIG);

	switch (mode) {
	case NAND_ECC_READ    :
		__raw_writel(0x101, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
		/* (ECC 16 or 8 bit col) | ( CS  )  | ECC Enable */
		val = (dev_width << 7) | (info->gpmc_cs << 1) | (0x1);
		break;
	case NAND_ECC_READSYN :
		 __raw_writel(0x100, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
		/* (ECC 16 or 8 bit col) | ( CS  )  | ECC Enable */
		val = (dev_width << 7) | (info->gpmc_cs << 1) | (0x1);
		break;
	case NAND_ECC_WRITE   :
		__raw_writel(0x101, info->gpmc_baseaddr + GPMC_ECC_CONTROL);
		/* (ECC 16 or 8 bit col) | ( CS  )  | ECC Enable */
		val = (dev_width << 7) | (info->gpmc_cs << 1) | (0x1);
		break;
	default:
		DEBUG(MTD_DEBUG_LEVEL0, "Error: Unrecognized Mode[%d]!\n",
					mode);
		break;
	}

	__raw_writel(val, info->gpmc_baseaddr + GPMC_ECC_CONFIG);
}
#endif

/*
 * omap_wait - Wait function is called during Program and erase
 * operations and the way it is called from MTD layer, we should wait
 * till the NAND chip is ready after the programming/erase operation
 * has completed.
 * @mtd: MTD device structure
 * @chip: NAND Chip structure
 */
static int omap_wait(struct mtd_info *mtd, struct nand_chip *chip)
{
	register struct nand_chip *this = mtd->priv;
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	int status = 0;

	this->IO_ADDR_W = (void *) info->gpmc_cs_baseaddr +
						GPMC_CS_NAND_COMMAND;
	this->IO_ADDR_R = (void *) info->gpmc_cs_baseaddr + GPMC_CS_NAND_DATA;

	while (!(status & 0x40)) {
		 __raw_writeb(NAND_CMD_STATUS & 0xFF, this->IO_ADDR_W);
		status = __raw_readb(this->IO_ADDR_R);
	}
	return status;
}

/*
 * omap_dev_ready - calls the platform specific dev_ready function
 * @mtd: MTD device structure
 */
static int omap_dev_ready(struct mtd_info *mtd)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info,
							mtd);
	unsigned int val = __raw_readl(info->gpmc_baseaddr + GPMC_IRQ_STATUS);

	if ((val & 0x100) == 0x100) {
		/* Clear IRQ Interrupt */
		val |= 0x100;
		val &= ~(0x0);
		__raw_writel(val, info->gpmc_baseaddr + GPMC_IRQ_STATUS);
	} else {
		unsigned int cnt = 0;
		while (cnt++ < 0x1FF) {
			if  ((val & 0x100) == 0x100)
				return 0;
			val = __raw_readl(info->gpmc_baseaddr +
							GPMC_IRQ_STATUS);
		}
	}

	return 1;
}

#ifdef CONFIG_MACH_ARCHOS
/*
 * nand_correct_data - Compares the ecc read from nand spare area with ECC
 * registers values and corrects one bit error if it has occured
 * Further details can be had from OMAP TRM and the following selected links:
 * http://en.wikipedia.org/wiki/Hamming_code
 * http://www.cs.utexas.edu/users/plaxton/c/337/05f/slides/ErrorCorrection-4.pdf
 *
 * @dat:		 page data
 * @read_ecc:		 ecc read from nand flash
 * @calc_ecc:		 ecc read from ECC registers
 *
 * @return 0 if data is OK or corrected, else returns -1
 */
static int omap_nand_correct_data( struct mtd_info *mtd,
				uint8_t *dat, uint8_t *read_ecc, uint8_t *calc_ecc)
{
	uint32_t orig_ecc, new_ecc, res, hm;
	uint16_t parity_bits, byte;
	uint8_t bit;

	/* Regenerate the orginal ECC */
	orig_ecc = gen_true_ecc(read_ecc);
	new_ecc = gen_true_ecc(calc_ecc);
#if 0
printk("read %02x %02x %02x  calc %02x %02x %02x  orig_ecc %x new_ecc %x\n", read_ecc[0], read_ecc[1], read_ecc[2],
 calc_ecc[0], calc_ecc[1], calc_ecc[2], orig_ecc, new_ecc);
#endif
	/* Get the XOR of real ecc */
	res = orig_ecc ^ new_ecc;
	if (res) {
		/* Get the hamming width */
		hm = hweight32(res);
		/* Single bit errors can be corrected! */
		if (hm == 12) {
			/* Correctable data! */
			parity_bits = res >> 16;
			bit = (parity_bits & 0x7);
			byte = (parity_bits >> 3) & 0x1FF;
			/* Flip the bit to correct */
			dat[byte] ^= (0x1 << bit);
		} else if (hm == 1) {
			printk("Error: Ecc is wrong\n");
			/* ECC itself is corrupted */
			return 2;
		} else {
			/*
			 * hm distance != parity pairs OR one, could mean 2 bit
			 * error OR potentially be on a blank page..
			 * orig_ecc: contains spare area data from nand flash.
			 * new_ecc: generated ecc while reading data area.
			 * Note: if the ecc = 0, all data bits from which it was
			 * generated are 0xFF.
			 * The 3 byte(24 bits) ecc is generated per 512byte
			 * chunk of a page. If orig_ecc(from spare area)
			 * is 0xFF && new_ecc(computed now from data area)=0x0,
			 * this means that data area is 0xFF and spare area is
			 * 0xFF. A sure sign of a erased page!
			 */
			if ((orig_ecc == 0x0FFF0FFF) && (new_ecc == 0x00000000))
				return 0;
			printk("Error: Bad compare! failed\n");
			/* detected 2 bit error */
			return -1;
		}
	}
	return 0;
}
#endif

static int __devinit omap_nand_probe(struct platform_device *pdev)
{
	struct omap_nand_info		*info;
	struct omap_nand_platform_data	*pdata;
	int				err;
	unsigned long 			val;


	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct omap_nand_info), GFP_KERNEL);
	if (!info) return -ENOMEM;

	platform_set_drvdata(pdev, info);

	spin_lock_init(&info->controller.lock);
	init_waitqueue_head(&info->controller.wq);

	info->pdev = pdev;

	info->gpmc_cs		= pdata->cs;
	info->gpmc_baseaddr	= pdata->gpmc_baseaddr;
	info->gpmc_cs_baseaddr	= pdata->gpmc_cs_baseaddr;

	info->mtd.priv		= &info->nand;
	info->mtd.name		= pdev->dev.bus_id;
	info->mtd.owner		= THIS_MODULE;

	err = gpmc_cs_request(info->gpmc_cs, NAND_IO_SIZE, &info->phys_base);
	if (err < 0) {
		dev_err(&pdev->dev, "Cannot request GPMC CS\n");
		goto out_free_info;
	}


	/* Enable RD PIN Monitoring Reg */
	if (pdata->dev_ready) {
		val  = gpmc_cs_read_reg(info->gpmc_cs, GPMC_CS_CONFIG1);
		val |= WR_RD_PIN_MONITORING;
		gpmc_cs_write_reg(info->gpmc_cs, GPMC_CS_CONFIG1, val);
	}

	val  = gpmc_cs_read_reg(info->gpmc_cs, GPMC_CS_CONFIG7);
	val &= ~(0xf << 8);
	val |=  (0xc & 0xf) << 8;
	gpmc_cs_write_reg(info->gpmc_cs, GPMC_CS_CONFIG7, val);

	/* NAND write protect off */
	omap_nand_wp(&info->mtd, NAND_WP_OFF);

	if (!request_mem_region(info->phys_base, NAND_IO_SIZE,
				pdev->dev.driver->name)) {
		err = -EBUSY;
		goto out_free_cs;
	}

	info->nand.IO_ADDR_R = ioremap(info->phys_base, NAND_IO_SIZE);
	if (!info->nand.IO_ADDR_R) {
		err = -ENOMEM;
		goto out_release_mem_region;
	}
	if (use_prefetch) {
		/* copy the virtual address of nand base for fifo access */
		info->nand_pref_fifo_add = info->nand.IO_ADDR_R;
		if (use_dma) {
			err = omap_request_dma(OMAP24XX_DMA_GPMC, "NAND",
				omap_nand_dma_cb, &info->comp, &info->dma_ch);
			if (err < 0) {
				info->dma_ch = -1;
				printk(KERN_WARNING "DMA request failed."
					" Non-dma data transfer mode\n");
			}
		}
	}
	info->nand.controller = &info->controller;

	info->nand.IO_ADDR_W = info->nand.IO_ADDR_R;
	info->nand.cmd_ctrl  = omap_hwcontrol;

	/*
	* If RDY/BSY line is connected to OMAP then use the omap ready funcrtion
	* and the generic nand_wait function which reads the status register
	* after monitoring the RDY/BSY line.Otherwise use a standard chip delay
	* which is slightly more than tR (AC Timing) of the NAND device and read
	* status register until you get a failure or success
	*/
	if (pdata->dev_ready) {
		info->nand.dev_ready = omap_dev_ready;
		info->nand.chip_delay = 0;
	} else {
		info->nand.waitfunc = omap_wait;
		info->nand.chip_delay = 50;
	}

#ifdef CONFIG_MACH_ARCHOS
	/* to me it looks like a bug to skip bad block scan!
	 * How are we supposed to get the factory set bad blocks then??!? 
	 */
#else
	info->nand.options  |= NAND_SKIP_BBTSCAN;
#endif
	if ((gpmc_cs_read_reg(info->gpmc_cs, GPMC_CS_CONFIG1) & 0x3000)
								== 0x1000)
		info->nand.options  |= NAND_BUSWIDTH_16;

	if (use_prefetch) {
		info->nand.read_buf   = omap_read_buf_pref;
		info->nand.write_buf  = omap_write_buf_pref;
	} else {
		if (info->nand.options & NAND_BUSWIDTH_16) {
			info->nand.read_buf   = omap_read_buf16;
			info->nand.write_buf  = omap_write_buf16;
		} else {
			info->nand.read_buf   = omap_read_buf8;
			info->nand.write_buf  = omap_write_buf8;
		}
	}
	info->nand.verify_buf = omap_verify_buf;

#ifdef CONFIG_MTD_NAND_OMAP_HWECC
	info->nand.ecc.bytes		= 3;
	info->nand.ecc.size		= 512;
	info->nand.ecc.calculate	= omap_calculate_ecc;
	info->nand.ecc.hwctl		= omap_enable_hwecc;
#ifdef CONFIG_MACH_ARCHOS
	info->nand.ecc.correct		= omap_nand_correct_data;
	info->nand.ecc.layout 		= &hw_nand_oob;
#else
	info->nand.ecc.correct		= nand_correct_data;
#endif
	info->nand.ecc.mode		= NAND_ECC_HW;

	/* init HW ECC */
	omap_hwecc_init(&info->mtd);
#else
	info->nand.ecc.mode = NAND_ECC_SOFT;
	info->nand.ecc.size		= 512;
#endif

	/* DIP switches on some boards change between 8 and 16 bit
	 * bus widths for flash.  Try the other width if the first try fails.
	 */
	if (nand_scan(&info->mtd, 1)) {
		info->nand.options ^= NAND_BUSWIDTH_16;
		if (nand_scan(&info->mtd, 1)) {
			err = -ENXIO;
			goto out_release_mem_region;
		}
	}
	/* If the board has an unlock function, use it */
	info->mtd.unlock	= pdata->unlock;

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(&info->mtd, part_probes, &info->parts, 0);
	if (err > 0)
		add_mtd_partitions(&info->mtd, info->parts, err);
	else if (pdata->parts)
		add_mtd_partitions(&info->mtd, pdata->parts, pdata->nr_parts);
	else
#endif
		add_mtd_device(&info->mtd);

	platform_set_drvdata(pdev, &info->mtd);

	return 0;

out_release_mem_region:
	release_mem_region(info->phys_base, NAND_IO_SIZE);
out_free_cs:
	gpmc_cs_free(info->gpmc_cs);
out_free_info:
	kfree(info);

	return err;
}

static int omap_nand_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct omap_nand_info *info = mtd->priv;

	platform_set_drvdata(pdev, NULL);
	if (use_dma)
		omap_free_dma(info->dma_ch);

	/* Release NAND device, its internal structures and partitions */
	nand_release(&info->mtd);
	iounmap(info->nand_pref_fifo_add);
	kfree(&info->mtd);
	return 0;
}

static struct platform_driver omap_nand_driver = {
	.probe		= omap_nand_probe,
	.remove		= omap_nand_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};
MODULE_ALIAS(DRIVER_NAME);

static int __init omap_nand_init(void)
{
	printk(KERN_INFO "%s driver initializing\n", DRIVER_NAME);

	/* This check is required if driver is being
	 * loaded run time as a module
	 */
	if ((1 == use_dma) && (0 == use_prefetch)) {
		printk(KERN_INFO"Wrong parameters: 'use_dma' can not be 1 "
				"without use_prefetch'. Prefetch will not be"
				" used in either mode (mpu or dma)\n");
	}
	return platform_driver_register(&omap_nand_driver);
}

static void __exit omap_nand_exit(void)
{
	platform_driver_unregister(&omap_nand_driver);
}

module_init(omap_nand_init);
module_exit(omap_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Glue layer for NAND flash on TI OMAP boards");
