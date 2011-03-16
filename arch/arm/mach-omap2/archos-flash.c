#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <mach/nand.h>
#include <mach/io.h>
#include <mach/gpmc.h>
#include <asm/mach/flash.h>
#include <mach/board-archos.h>

#define GPMC_CS_SIZE   0x30

#define NOR_BOOT_BLOCK_SIZE	SZ_8K
#define NOR_BLOCK_SIZE		SZ_64K
#define NAND_BLOCK_SIZE		SZ_128K

#define FLASH_PART_NAME_STAGE1 		"stage1"
#define FLASH_PART_NAME_STAGE2 		"stage2"
#define FLASH_PART_NAME_STORE5 		"store5"
#define FLASH_PART_NAME_STOREX 		"storex"
#define FLASH_PART_NAME_STOREXFLAT	"storexflat"
#define FLASH_PART_NAME_LOGO 		"logo"
#define FLASH_PART_NAME_INIT 		"init"
#define FLASH_PART_NAME_ENV 		"env"
#define FLASH_PART_NAME_RECOVERY 	"recovery"
#define FLASH_PART_NAME_SYSTEM		"system"
#define FLASH_PART_NAME_DATA		"data"

#ifdef CONFIG_MACH_ARCHOS_PRODTEST_MTD
        #define MTD0_MASK_FLAGS 0
#else
        #define MTD0_MASK_FLAGS MTD_WRITEABLE	/* force read-only */
#endif

static struct mtd_partition archos_nor_partitions[] = {
	/* All the partition sizes are listed in terms of Nor block size */
	{
		.name		= FLASH_PART_NAME_STAGE1,
		.offset		= 0,
		.size		= 1 * NOR_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= FLASH_PART_NAME_STORE5,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x010000 */
		.size		= 2 * NOR_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= FLASH_PART_NAME_STAGE2,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x030000 */
		.size		= 3 * NOR_BLOCK_SIZE,
	},
	{
		.name		= FLASH_PART_NAME_LOGO,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x060000 */
		.size		= 2 * NOR_BLOCK_SIZE,
	},
	{
		.name		= FLASH_PART_NAME_RECOVERY,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x080000 */
		.size		= 25 * NOR_BLOCK_SIZE,
	},
	{
		.name		= FLASH_PART_NAME_INIT,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x210000 */
		.size		= 30 * NOR_BLOCK_SIZE,
	},
/*
	{
		.name		= FLASH_PART_NAME_ENV,
		.offset		= MTDPART_OFS_APPEND,	// Offset = 0x3F0000 
		.size		= 1 * NOR_BOOT_BLOCK_SIZE,
	},
*/
	{
		.name		= FLASH_PART_NAME_STOREXFLAT, /* + AVBoot Evironment */
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x3F2000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data archos_nor_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= archos_nor_partitions,
	.nr_parts	= ARRAY_SIZE(archos_nor_partitions),
};

static struct resource archos_nor_resource = {
	.start		= 0,
	.end		= 0,
	.flags		= IORESOURCE_MEM,
};

static struct mtd_partition archos_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= FLASH_PART_NAME_STAGE1,
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD0_MASK_FLAGS,
	},
	{
		.name		= FLASH_PART_NAME_STAGE2,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= FLASH_PART_NAME_RECOVERY,
		.offset		= 8 * NAND_BLOCK_SIZE,	/* Offset = 0x100000 */
		.size		= 24 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= FLASH_PART_NAME_INIT,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x400000 */
		.size		= 20 * NAND_BLOCK_SIZE,
	},
	{
		.name		= FLASH_PART_NAME_STOREX,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= 5 * NAND_BLOCK_SIZE,
	},
	{
		.name		= FLASH_PART_NAME_SYSTEM,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x720000 */
		.size		= 720 * NAND_BLOCK_SIZE,
	},
	{
		.name		= FLASH_PART_NAME_DATA,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x6120000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data archos_nand_data = {
	.options	= NAND_BUSWIDTH_16,
	.parts		= archos_nand_partitions,
	.nr_parts	= ARRAY_SIZE(archos_nand_partitions),
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.nand_setup	= NULL,
	.dev_ready	= NULL,
};

static struct resource archos_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device archos_nor_device = {
	.name		= "omapflash",
	.id		= -1,
	.dev		= {
		.platform_data	= &archos_nor_data,
	},
	.num_resources	= 1,
	.resource	= &archos_nor_resource,
};

static struct platform_device archos_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &archos_nand_data,
	},
	.num_resources	= 1,
	.resource	= &archos_nand_resource,
};

void __init archos_flash_init(void)
{
	u8 cs = 0;

	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	u32 ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);
	if ((ret & 0xC00) == GPMC_CONFIG1_DEVICETYPE_NOR) {
		printk(KERN_INFO "Found NOR on CS%d\n", cs);

		archos_nor_resource.start  = FLASH_BASE_NOR_ARCHOS;
		archos_nor_resource.end    = FLASH_BASE_NOR_ARCHOS
						+ FLASH_SIZE_NOR_ARCHOS - 1;
		printk(KERN_INFO "Registering NOR on CS%d\n", cs);
		if (platform_device_register(&archos_nor_device) < 0)
				printk(KERN_ERR "Unable to register NOR device\n");
	}
	else if ((ret & 0xC00) == GPMC_CONFIG1_DEVICETYPE_NAND) {
		printk(KERN_INFO "Found NAND on CS%d\n", cs);

		archos_nand_data.cs = cs;
		archos_nand_data.gpmc_cs_baseaddr = (void *)
				(gpmc_base_add + GPMC_CS0_BASE + cs * GPMC_CS_SIZE);
		archos_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

		printk(KERN_INFO "Registering NAND on CS%d\n", cs);
		if (platform_device_register(&archos_nand_device) < 0)
				printk(KERN_ERR "Unable to register NAND device\n");

	} else {
		printk(KERN_ERR "No FLASH device found on CS%d\n", cs);
		return;
	}

}
