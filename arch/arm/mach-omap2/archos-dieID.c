#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <asm/io.h>
#include <mach/archos-dieid.h>

static struct sysdev_class die_id_sysclass = {
	.name		= "dieID",
};

void get_dieid(u32 *die_id)
{
	die_id[3]=omap_readl(CONTROL_DIE_ID_REG);
	die_id[2]=omap_readl(CONTROL_DIE_ID_REG+4);
	die_id[1]=omap_readl(CONTROL_DIE_ID_REG+8);
	die_id[0]=omap_readl(CONTROL_DIE_ID_REG+12);
}

static ssize_t show_dieid(struct sysdev_class* cls, char* buf)
{
	u32 die_id[4];
	get_dieid(die_id);
	return sprintf(buf, "%08x%08x%08x%08x\n", die_id[0],die_id[1],die_id[2],die_id[3]); 
}
static SYSDEV_CLASS_ATTR(dieid, S_IRUGO, show_dieid, NULL);

void get_prodid(u32 *prod_id)
{
	prod_id[3]=omap_readl(CONTROL_PROD_ID_REG);
	prod_id[2]=omap_readl(CONTROL_PROD_ID_REG+4);
	prod_id[1]=omap_readl(CONTROL_PROD_ID_REG+8);
	prod_id[0]=omap_readl(CONTROL_PROD_ID_REG+12);
}

static ssize_t show_prodid(struct sysdev_class *cls, char* buf)
{
	u32 prod_id[4];
	get_prodid(prod_id);
	return sprintf(buf, "%08x%08x%08x%08x\n", prod_id[0],prod_id[1],prod_id[2],prod_id[3]); 
}
static SYSDEV_CLASS_ATTR(prodid, S_IRUGO, show_prodid, NULL);

static ssize_t show_idcode(struct sysdev_class *cls, char* buf)
{
	u32 idcode = omap_readl(CONTROL_IDCODE_REG);
	return sprintf(buf, "%08x\n", idcode);
}
static SYSDEV_CLASS_ATTR(idcode, S_IRUGO, show_idcode, NULL);

static int __init archos_get_dieid_init(void)
{
	int ret;
	
	ret = sysdev_class_register(&die_id_sysclass);
	if (ret >= 0) {
		sysdev_class_create_file(&die_id_sysclass, &attr_dieid);	
		sysdev_class_create_file(&die_id_sysclass, &attr_prodid);
		sysdev_class_create_file(&die_id_sysclass, &attr_idcode);
	}
	return ret;
}

late_initcall(archos_get_dieid_init);
