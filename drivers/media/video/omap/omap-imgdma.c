/*
 * omap-imgdma.c
 *
 * Copyright (c) 2006 Archos
 * Author: Matthias Welwarsky
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/rwsem.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>        /* copy_from_user */
#include <asm/io.h>
#include <mach/memory.h>
#include <mach/hardware.h>
#include <mach/display.h>
#include <mach/dma.h>
#include <asm/cacheflush.h>

#include <media/omap-imgdma.h>

#define DBG	if (0)

#define IMGDMA_NAME	"imgdma"

typedef struct {
	unsigned long	src_addr;
	unsigned long	src_linestep;
	unsigned long	dst_addr; // phys
	unsigned long	dst_linestep;
	unsigned long	width;
	unsigned long	height;
	unsigned long	src_ei;
	unsigned long	dst_ei;
} dma_regdata_t;

typedef struct {
	struct completion 	complete;
	struct list_head	node;
	int			err;
	int			cmd_tag;
	int			waiting;
	int			vbl_sync;
	dma_regdata_t		reg;
	unsigned long		dst_addr; // virt
} dma_command_t;

static DEFINE_SPINLOCK(dma_cmd_lock);
static dma_command_t *current_cmd;
static int dma_ch = -1;

static DECLARE_RWSEM(dma_queue_lock);
static LIST_HEAD(dma_queue);
static int dma_current_tag;
static struct workqueue_struct *dma_workqueue;

static void *isr_handle;

static void dma_complete(int lch, unsigned short ch_status, void *data);
static void dma_start_next(void);

static void _dump_regs(dma_regdata_t* reg)
{
	printk("src_addr:	0x%08lX\n", reg->src_addr);
	printk("src_linestep:	%ld\n", reg->src_linestep);
	printk("dst_addr:	0x%08lX\n", reg->dst_addr);
	printk("dst_linestep:	%ld\n", reg->dst_linestep);
	printk("width:		%ld\n", reg->width);
	printk("height:		%ld\n", reg->height);
	printk("src ei:		%ld\n", reg->src_ei);
	printk("dst ei:		%ld\n", reg->dst_ei);
}

static unsigned long to_phys_addr(unsigned long virtp)
{	
	pgd_t *pgd = pgd_offset( current->mm, virtp );
	pmd_t *pmd = pmd_offset( pgd, virtp );
	pte_t *pte = pte_offset_kernel( pmd, virtp );                  
DBG	printk( "imgdma: %s\n", __FUNCTION__ );
	return __pa( page_address( pte_page( *pte ) ) + (virtp & ~PAGE_MASK) );
}

static inline u32 uservirt_to_phys(u32 virtp)
{
	unsigned long physp = 0;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;

	/* For kernel direct-mapped memory, take the easy way */
	if (virtp >= PAGE_OFFSET) {
		physp = virt_to_phys((void *)virtp);
	} else if ((vma = find_vma(mm, virtp)) && (vma->vm_flags & VM_IO)
		   && (vma->vm_pgoff)) {
		/* this will catch, kernel-allocated,
		mmaped-to-usermode addresses */
		physp = (vma->vm_pgoff << PAGE_SHIFT) + (virtp - vma->vm_start);
	} else
		physp = 0;

        return physp;
}

static int prepare_command(dma_command_t* cmd, dma_transfer_t* cmd_data)
{
	int src_area_len;
	int dst_area_len;
	unsigned long src_addr = cmd_data->src_addr;
	unsigned long dst_addr = cmd_data->dst_addr;
	dma_regdata_t* reg = &cmd->reg;
	u32 phys_addr;

DBG	printk( "imgdma: %s\n", __FUNCTION__ );

	/* first, check if we're allowed to read ... */
	src_area_len = cmd_data->src_linestep * (cmd_data->height - 1) + cmd_data->width;
	if (!access_ok(VERIFY_READ, (void*)src_addr, src_area_len)) {
		printk(KERN_DEBUG "imgdma: not allowed to read %u byte from %p\n", src_area_len, (void*)src_addr);
		return -EFAULT;
	}
DBG	printk("imgdma: read access ok\n");
		
	/* ... then if we're allowed to write. */
	dst_area_len = cmd_data->dst_linestep * (cmd_data->height - 1) + cmd_data->width;
	if (!access_ok(VERIFY_WRITE, (void*)dst_addr, dst_area_len)) {
		printk(KERN_DEBUG "imgdma: not allowed to write %u byte to %p\n", dst_area_len, (void*)dst_addr);
		return -EFAULT;
	}
DBG	printk("imgdma: write access ok\n");

	phys_addr = uservirt_to_phys(src_addr);
	if (phys_addr == 0) {
		printk(KERN_ERR "imgdma: src buffer not kernel-allocated\n");
		return -EFAULT;
	}
	reg->src_addr     = phys_addr;

	phys_addr = uservirt_to_phys(dst_addr);
	if (phys_addr == 0) {
		printk(KERN_ERR "imgdma: dst buffer not kernel-allocated\n");
		return -EFAULT;
	}
	reg->dst_addr     = phys_addr;
	
	reg->src_linestep = cmd_data->src_linestep;
	reg->dst_linestep = cmd_data->dst_linestep;
	reg->width        = cmd_data->width;
	reg->height       = cmd_data->height;
	reg->src_ei	  = cmd_data->src_ei;
	reg->dst_ei	  = cmd_data->dst_ei;
	cmd->vbl_sync     = cmd_data->vblsync;
	cmd->dst_addr	  = cmd_data->dst_addr;

	if(src_area_len > 262144) {
		// if we have more to flush than what can fit in the cache,
		// just flush the whole cache! 
		__cpuc_flush_kern_all();
	} else {
		// Source      should be cleaned     (written back)
		// Destination should be invalidated
		dmac_clean_range((void*)src_addr, (void*)(src_addr + src_area_len));
		dmac_inv_range(  (void*)src_addr, (void*)(src_addr + src_area_len));

		dmac_clean_range((void*)dst_addr, (void*)(dst_addr + dst_area_len));
		dmac_inv_range(  (void*)dst_addr, (void*)(dst_addr + dst_area_len));
	}
	
DBG	printk("imgdma: before dump regs\n");
DBG	_dump_regs(reg);
DBG	printk("imgdma: after dump regs\n");

	return 0;
}

unsigned long start_jiffies = 0;

static void dma_kick( dma_regdata_t* regs )
{
	int ret;
	struct omap_dma_channel_params dma_params;

///* DBG */	printk( "imgdma: %s\n", __FUNCTION__ );

	if (dma_ch != -1)
		printk(KERN_ERR "imgdma_kick: dma channel already allocated\n");

	ret = omap_request_dma( 0, IMGDMA_NAME, dma_complete, NULL, &dma_ch);
	if (ret < 0) {
		printk(KERN_ERR "imgdma_kick: no dma channel available!\n");
		current_cmd->err = ret;
		complete(&current_cmd->complete);
		dma_start_next();
		return;
	}

	memset(&dma_params, 0, sizeof(dma_params));

	dma_params.data_type   = OMAP_DMA_DATA_TYPE_S8;
	dma_params.elem_count  = regs->width;
	dma_params.frame_count = regs->height;

	dma_params.src_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
	dma_params.src_start = regs->src_addr;
	dma_params.src_ei = regs->src_ei;
	dma_params.src_fi = regs->src_linestep - regs->src_ei * (regs->width - 1);

	dma_params.dst_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
	dma_params.dst_start = regs->dst_addr;
	dma_params.dst_ei = regs->dst_ei;
	dma_params.dst_fi = regs->dst_linestep - regs->dst_ei * (regs->width - 1);

	dma_params.read_prio = DMA_CH_PRIO_HIGH;
	dma_params.write_prio = DMA_CH_PRIO_HIGH;

	omap_set_dma_params(dma_ch, &dma_params);

	omap_set_dma_src_data_pack(dma_ch, 1);
	omap_set_dma_dest_data_pack(dma_ch, 1);

	omap_set_dma_src_burst_mode(dma_ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_dest_burst_mode(dma_ch, OMAP_DMA_DATA_BURST_16);

	omap_start_dma(dma_ch);

	start_jiffies = jiffies;
}

static dma_command_t *dma_new_command( void )
{
	dma_command_t *new_cmd;
DBG	printk( "imgdma: %s\n", __FUNCTION__ );
	new_cmd = kmalloc( sizeof (dma_command_t), GFP_KERNEL);
	if (new_cmd == NULL)
		return NULL;
		
	new_cmd->waiting = 0;
	new_cmd->err = 0;
	init_completion(&new_cmd->complete);
	
	return new_cmd;
}

static inline void dma_delete_command( dma_command_t *cmd )
{
DBG	printk( "imgdma: %s\n", __FUNCTION__ );
	kfree(cmd);
}

static int dma_queue_cmd( dma_command_t *cmd )
{
	int tag;
	unsigned long flags;
DBG	printk( "imgdma: %s\n", __FUNCTION__ );
	down_write(&dma_queue_lock);
	
	/* generate a new command tag */
	tag = ++dma_current_tag;
	if (tag <= 0)
		tag = dma_current_tag = 1;
	cmd->cmd_tag = tag;

	spin_lock_irqsave(&dma_cmd_lock, flags);
	if (current_cmd != NULL) {
		/* add job and release the lock */
		list_add_tail(&cmd->node, &dma_queue);
		spin_unlock_irqrestore(&dma_cmd_lock, flags);
DBG		printk("dma_queue_cmd: job nr %i queued\n", tag);
	
	} else {
		/* no command running, release lock and start next one */
		spin_unlock_irqrestore(&dma_cmd_lock, flags);
		
		/* add job to the queue and start it */
		list_add_tail(&cmd->node, &dma_queue);
		current_cmd = cmd;
		
		if (!cmd->vbl_sync)
			dma_kick(&cmd->reg);
		
DBG		printk("dma_queue_cmd: job nr %i queued & running\n", tag);
	} 

	up_write(&dma_queue_lock);

	return tag;
}

static inline void dma_dequeue_cmd( dma_command_t *cmd )
{
	unsigned long flags;

DBG	printk( "imgdma: %s\n", __FUNCTION__ );
	spin_lock_irqsave(&dma_cmd_lock, flags);
	list_del(&cmd->node);
	spin_unlock_irqrestore(&dma_cmd_lock, flags);
}

static void dma_job_reaper(struct work_struct* data)
{
	dma_command_t *cmd, *n;

DBG	printk( "imgdma: %s\n", __FUNCTION__ );
	/* walk the queue, remove all completed asynchronous jobs */
	down_write(&dma_queue_lock);
	list_for_each_entry_safe(cmd, n, &dma_queue, node) {
		if (cmd->complete.done && !cmd->waiting) {
			dma_dequeue_cmd(cmd);
/* do not do this, we have seen the kernel hang a lot with it
			dmac_inv_range(
				(void*)cmd->dst_addr,
				(void*)(cmd->dst_addr
					+ cmd->reg.dst_linestep * (cmd->reg.height - 1)
					+ cmd->reg.width));
*/
			dma_delete_command(cmd);
DBG			printk("dma_job_reaper: removing job %i\n", cmd->cmd_tag);
		}
	}	
	up_write(&dma_queue_lock);
}

static DECLARE_WORK(dma_worker, dma_job_reaper);

static void dma_start_next(void)
{
	current_cmd = NULL;

	/* start the next command if queue's not empty */
	if (!list_empty(&dma_queue)) {
		dma_command_t* cmd;
		list_for_each_entry(cmd, &dma_queue, node) {
			
			/* skip completed commands */
			if (cmd->complete.done)
				continue;

DBG			printk("dma_irq: next job nr %i\n", cmd->cmd_tag);
	
			/* go! */
			current_cmd = cmd;
			if (!current_cmd->vbl_sync)
				dma_kick(&current_cmd->reg);

			/* break out of the loop */
DBG			printk(KERN_DEBUG "dma_irq: starting job %i\n", current_cmd->cmd_tag);
			break;
		}
	}

DBG	printk("dma_irq: starting dma_job_reaper\n");
	/* must not add a work item that's already queued */
	if (list_empty(&dma_worker.entry))
		queue_work(dma_workqueue, &dma_worker);
}

static void dma_complete(int lch, unsigned short ch_status, void *data)
{	
//	printk("total_jiffies = %lu\n", jiffies - start_jiffies);
DBG	printk( "imgdma: %s\n", __FUNCTION__ );
	if (lch != dma_ch)
		return;
#if 0
/* ignore status, we do not handle it anyway! */
	if (ch_status ) {
		printk(KERN_INFO "imgdma: DMA completed with status code %i\n", ch_status);
	}
#endif
	omap_free_dma(dma_ch);
	dma_ch = -1;

	/* satisfy your paranoia once a day */
	if (current_cmd == NULL)
		return;
	
	/* wake up tasks waiting for the last command */
	current_cmd->err = 0;
	complete(&current_cmd->complete);
	
DBG	printk("dma_irq: completed job %i\n", current_cmd->cmd_tag);

	dma_start_next();
}

static void vbl_irq(void *arg, u32 mask)
{
	if (current_cmd && current_cmd->vbl_sync) {
		current_cmd->vbl_sync = 0;
		dma_kick(&current_cmd->reg);
	}
}

static int imgdma_ioctl(struct inode * inode, struct file *filp,
	     unsigned int cmd, unsigned long arg)
{
	dma_transfer_t param_cmd_data;
	dma_command_t *dma_cmd, *n;
	int cmd_tag;
	int ret = -EINVAL;
//	unsigned long jif = jiffies;

//	printk("DR: entering ioctl %lu\n", jiffies - jif);

DBG	printk( "imgdma: %s\n", __FUNCTION__ );
	switch (cmd) {
	case IMGDMA_START_TRANSFER:
		/* copy argument, FAULT if pointer is invalid */
		if (copy_from_user(&param_cmd_data, (void __user *)arg, sizeof(param_cmd_data)))
			return -EFAULT;

		if (param_cmd_data.size != sizeof(param_cmd_data))
			return -EINVAL;

		/* create the new job */
		dma_cmd = dma_new_command();
		if (dma_cmd == NULL)
			return -ENOMEM;
		
DBG		printk("imgdma: enter start transfer\n");
		
		/* verify the command, could be done before allocating it
		 * but failure is a rare case anyway */
		ret = prepare_command(dma_cmd, &param_cmd_data);
		if (ret < 0) {
			printk("imgdma: prepare command failed\n");
			dma_delete_command(dma_cmd);
			break;
		}
		
//		printk("DR: before kick %lu\n", jiffies - jif);

		ret = dma_queue_cmd(dma_cmd);
		if (ret < 0)
		{
			printk("imgdma: queue command failed\n");
			dma_delete_command(dma_cmd);
		}

		/* command completes asynchronously, just return the tag */
		break;

//		printk("DR: after kick %lu\n", jiffies - jif);
		
	case IMGDMA_WAIT_SYNC:
		/*
		 * Walk the queue, try to find the tag.
		 * If the tag is found, wait for completion of the command,
		 * timeout after 2 seconds.
		 * If tag is not found, assume that the job was completed and
		 * return immediately.
		 */
		if (get_user(cmd_tag, (int __user*)arg)) {
			ret = -EFAULT;
			break;
		}
//		cmd_tag = ret;
// DR this was used to be able to comment the break before the case WAIT (and thus the get_user() too)

//		printk("DR: before wait sync %lu\n", jiffies - jif);

DBG		printk("imgdma: enter wait_sync\n");

		down_read(&dma_queue_lock);
		list_for_each_entry_safe(dma_cmd, n, &dma_queue, node) {
			if (dma_cmd->cmd_tag == cmd_tag) {
				/* tell async reaper to skip this one */
				dma_cmd->waiting = 1;
				/* it's safe to release the lock now */
				up_read(&dma_queue_lock);

DBG				printk("imgdma: waiting for job %i\n", cmd_tag);
//				printk("DR: before completion %lu\n", jiffies - jif);
				ret = wait_for_completion_timeout(&dma_cmd->complete, 2*HZ);
//				printk("DR: after completion %lu\n", jiffies - jif);
				down_write(&dma_queue_lock);
				dma_dequeue_cmd(dma_cmd);
				up_write(&dma_queue_lock);

				if (ret == 0 && !dma_cmd->complete.done) {
					ret = -EIO;
				} else {
					ret = dma_cmd->err;
				}
				dmac_inv_range(
					(void*)dma_cmd->dst_addr,
					(void*)(dma_cmd->dst_addr
						+ dma_cmd->reg.dst_linestep * (dma_cmd->reg.height - 1)
						+ dma_cmd->reg.width));
				dma_delete_command(dma_cmd);
DBG				printk("imgdma: wait complete for job %i\n", cmd_tag);
//				printk("DR: after wait sync %lu\n", jiffies - jif);
				return ret;
			}
		}

		/* didn't find the tag, release lock and return successfully */
		up_read(&dma_queue_lock);
		ret = 0;
		break;

	default:
		printk("imgdma: unknown ioctl %x\n", cmd);
		break;
	}

DBG	printk("imgdma: exit ioctl\n");
	return ret;
}

static int imgdma_open(struct inode * inode, struct file * filp)
{
DBG	printk( "imgdma: %s\n", __FUNCTION__ );	
	return 0;
}

static int imgdma_release(struct inode * inode, struct file * filp)
{
DBG	printk( "imgdma: %s\n", __FUNCTION__ );	
	return 0;
}

static struct file_operations imgdma_fops = {
	.owner	=	THIS_MODULE,
	.ioctl	=	imgdma_ioctl,
	.open	=	imgdma_open,
	.release=	imgdma_release,
};

static struct miscdevice imgdma_miscdev = {
	.minor	= IMGDMA_MINOR,
	.name	= IMGDMA_NAME,
	.fops	= &imgdma_fops,
};

/* +++++++++++++ End File operations ++++++++++++++*/

#ifdef CONFIG_PM
static int imgdma_drv_suspend(struct platform_device *pdev, pm_message_t pmsg)
{
#if 0
	if (dma_ch != -1) {
		omap_free_dma(dma_ch);
		dma_ch = -1;
	}
#endif
	return 0;
}

static int imgdma_drv_resume(struct platform_device *pdev)
{
#if 0
	int ret;
	ret = omap_request_dma( 0, IMGDMA_NAME, dma_complete, NULL, &dma_ch);
	if (ret< 0)
		printk(KERN_ERR "imgdma_drv_resume: no dma channel on resume\n");
#endif

	return 0;
}

#endif

/* +++++++++++++ Platform Device ++++++++++++++++++*/

static int __init
imgdma_drv_probe(struct platform_device *dev)
{
	int ret;

DBG	printk( "imgdma: %s\n", __FUNCTION__ );

	isr_handle = omap_dispc_register_isr(vbl_irq, (void*)1, DISPC_IRQ_VSYNC);
	if (isr_handle == NULL) {
		printk(KERN_ERR "imgdma: cannot register dispc isr\n");
		goto out_no_isr;
	}

	/* register our misc device */
	if ((ret = misc_register(&imgdma_miscdev)) < 0) {
		printk(KERN_ERR "imgdma: cannot register device (err=%i)\n", ret);
		goto out_no_miscdev;
	} else {
DBG	printk( "imgdma: got regs %d\n", ret );
	}

	dma_workqueue = create_workqueue("imgdma");

DBG	printk( "imgdma: returning ret %d\n", ret );
	return ret;
	
out_no_miscdev:
 	omap_dispc_unregister_isr(isr_handle);
out_no_isr:
	printk( "imgdma: returning ENODEV\n" );
	return -ENODEV;
}

static int
imgdma_drv_remove(struct platform_device *dev)
{
 	omap_dispc_unregister_isr(isr_handle);
	destroy_workqueue(dma_workqueue);

	misc_deregister(&imgdma_miscdev);
	
	return 0;
}

static void imgdma_dev_release(struct device *dev)
{
}

static struct platform_device imgdma_device = {
	.name = IMGDMA_NAME,
	.id   = 0,
	.dev = {
		.release = imgdma_dev_release,
	},
};

static struct platform_driver imgdma_driver = {
	.probe		= imgdma_drv_probe,
	.remove		= imgdma_drv_remove,
	.driver = {
		.bus = &platform_bus_type,
		.name = IMGDMA_NAME,
	},
#ifdef CONFIG_PM
	.suspend	= imgdma_drv_suspend,
	.resume		= imgdma_drv_resume,
#endif
};


/* ++++++++++++++++++ Module Init +++++++++++++++++++++++*/

static int imgdma_init_module(void)
{
	int ret;
	
	ret = platform_driver_register(&imgdma_driver);
DBG	printk( "imgdma: after driver register (%d)\n", ret );

DBG	printk( "imgdma: before device register\n" );
	ret = platform_device_register(&imgdma_device);
DBG	printk( "imgdma: after device register (%d)\n", ret );
	if (ret )
		platform_driver_unregister(&imgdma_driver);
	
	return ret;
}

static void imgdma_cleanup_module(void)
{
	platform_device_unregister(&imgdma_device);

	platform_driver_unregister(&imgdma_driver);
}

/* Module information */
MODULE_AUTHOR("Matthias Welwarsky, Archos S.A.");
MODULE_DESCRIPTION("Image DMA driver");
MODULE_LICENSE("GPL");

module_init(imgdma_init_module);
module_exit(imgdma_cleanup_module);
