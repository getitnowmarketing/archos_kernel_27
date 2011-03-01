/*
 * omap-resz.c
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
#include <linux/timer.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>        /* copy_from_user */
#include <asm/io.h>
#include <mach/memory.h>
#include <mach/hardware.h>
#include <mach/display.h>
#include <asm/scatterlist.h>

#include <media/omap-resz.h>

#include "../isp/isp.h"
#include "../isp/ispresizer.h"
#include "../isp/ispmmu.h"
#include "../isp/ispreg.h"

#define DBG	if (0)

#define VPSSRSZ_NAME	"resz"

#define OMAP_VPSS_REGS_BASE 0x480BD000

#define RESZ_PID	(OMAP_VPSS_REGS_BASE + 0x00)
#define RESZ_PCR	(OMAP_VPSS_REGS_BASE + 0x04)
#define RESZ_RSZ_CNT	(OMAP_VPSS_REGS_BASE + 0x08)
#define RESZ_OUT_SIZE	(OMAP_VPSS_REGS_BASE + 0x0C)
#define RESZ_IN_START	(OMAP_VPSS_REGS_BASE + 0x10)
#define RESZ_IN_SIZE	(OMAP_VPSS_REGS_BASE + 0x14)
#define RESZ_SDR_INADD	(OMAP_VPSS_REGS_BASE + 0x18)
#define RESZ_SDR_INOFF	(OMAP_VPSS_REGS_BASE + 0x1C)
#define RESZ_SDR_OUTADD	(OMAP_VPSS_REGS_BASE + 0x20)
#define RESZ_SDR_OUTOFF	(OMAP_VPSS_REGS_BASE + 0x24)
#define RESZ_HFILT(n)	(OMAP_VPSS_REGS_BASE + 0x28 + (n)*4)
#define RESZ_VFILT(n)	(OMAP_VPSS_REGS_BASE + 0x68 + (n)*4)

typedef struct {
	unsigned int cnt;
	unsigned int out_size;
	unsigned int in_start;
	unsigned int in_size;
	unsigned int sdr_inadd;
	unsigned int sdr_inoff;
	unsigned int sdr_outadd;
	unsigned int sdr_outoff;
	unsigned int hfilt[16];
	unsigned int vfilt[16];
	unsigned int yenh;

	unsigned int read_delay;
} resz_regdata_t;

typedef struct {
	struct completion 	complete;
	struct list_head	node;
	int			cmd;
	int			err;
	int			cmd_tag;
	int			waiting;
	int			vbl_sync;
	resz_regdata_t		reg;
} resz_command_t;

static void resz_complete_cmd( void );
static void watchdog_timeout( unsigned long data );

// ISP-MMU     0x480BD400 0x480BD474 
// ISP-RESIZER 0x480BD000 0x480BD0AC

static void _dump_cmd(resz_command_t *cmd) 
{
	unsigned long addr;

	printk("===   RESZ  ===\n");
	printk("cnt:        %08x\n", cmd->reg.cnt);
	printk("out_size:   %08x\n", cmd->reg.out_size);
	printk("in_start:   %08x\n", cmd->reg.in_start);
	printk("in_size:    %08x\n", cmd->reg.in_size);
	printk("sdr_inadd:  %08x\n", cmd->reg.sdr_inadd);
	printk("sdr_inoff:  %08x\n", cmd->reg.sdr_inoff);
	printk("sdr_outadd: %08x\n", cmd->reg.sdr_outadd);
	printk("sdr_outoff: %08x\n", cmd->reg.sdr_outoff);
	printk("(h/vflt not shown\n");
	printk("yenh:       %08x\n", cmd->reg.yenh);
	
	for( addr = 0x480BD400; addr < 0x480BD474; addr += 4 ) {
		printk("MMU %08lX: %08X\n", addr, omap_readl( addr ) );
	}

	for( addr = 0x480BD000; addr < 0x480BD0AC; addr+= 4 ) {
		printk("RSZ %08lX: %08X\n", addr, omap_readl( addr ) );
	}
	
}

static DEFINE_SPINLOCK(rsz_cmd_lock);
static resz_command_t *current_cmd;

static DECLARE_RWSEM(rsz_queue_lock);
static LIST_HEAD(resz_queue);
static int resz_current_tag;
static struct workqueue_struct *resz_workqueue;

static DEFINE_SPINLOCK(rsz_register_lock);

static DEFINE_SPINLOCK(rsz_watchdog_lock);
static struct timer_list watchdog_timer = TIMER_INITIALIZER(watchdog_timeout, 0, 0);

static int clock_requested = 0;
static DEFINE_SPINLOCK(rsz_clock_request);

static void* isr_handle;

static void idle_timeout( unsigned long data)
{
DBG	printk("ISP idle_timeout\n");

	/* check if the ISP clocks need to be enabled */
	spin_lock(&rsz_clock_request);
	if ( clock_requested ) {
		isp_put();
		clock_requested = 0;
	}
	spin_unlock(&rsz_clock_request);
}

static struct timer_list idle_timer = TIMER_INITIALIZER( idle_timeout, 0, 0 );

static void request_clocks( void )
{
	unsigned long flags;
	
	/* restart timeout (1s) for the isp resources*/
	mod_timer( &idle_timer, jiffies+HZ);
	
	/* check if the ISP clocks need to be enabled */
	spin_lock_irqsave(&rsz_clock_request, flags);
	if ( !clock_requested ) {
		isp_get();
		clock_requested = 1;
	}
	spin_unlock_irqrestore(&rsz_clock_request, flags);
}

static unsigned long to_phys_addr(unsigned long virtp)
{
	pgd_t *pgd = pgd_offset( current->mm, virtp );
	pmd_t *pmd = pmd_offset( pgd, virtp );
	pte_t *pte = pte_offset_kernel( pmd, virtp );                  
	return __pa( page_address( pte_page( *pte ) ) + (virtp & ~PAGE_MASK) );
}

static int prepare_command(resz_command_t* cmd, vpssrsz_param_cmd_t* cmd_data)
{
	resz_regdata_t* reg = &cmd->reg;
	unsigned long src_addr;
	unsigned long dst_addr;
	unsigned int in_area_len;
	unsigned int out_area_len;
	int h_rsz, v_rsz;
	int sph, spv;		/* horizontal start phase, vertical start phase */
	int hsize, vsize;	/* horizontal input size, vertical input size */
	int h_startpx;
	int cbilin;
	int inptype;
	int i;
	int in_surface, out_surface;

	request_clocks();
	
	if ( cmd_data->dst_rect.width > 1280 )
		return -EINVAL;
	
	dst_addr  = cmd_data->dst_rect.addr;
	src_addr  = cmd_data->src_rect.addr & ~0x1FUL;
	h_startpx = (cmd_data->src_rect.addr & 0x1FUL) / 2;
	
	/* first, check if we're allowed to read ... */
	in_area_len  = cmd_data->src_rect.linestep * (cmd_data->src_rect.height - 1) + cmd_data->src_rect.width * 2;
	if (!access_ok(VERIFY_READ, (void*)src_addr, in_area_len)) {
		printk(KERN_DEBUG "do_resize: not allowed to read %u byte from %p\n", in_area_len, (void*)src_addr);
		return -EFAULT;
	}
		
	/* ... then if we're allowed to write. */
	out_area_len = cmd_data->dst_rect.linestep * (cmd_data->dst_rect.height - 1) + cmd_data->dst_rect.width * 2;
	if (!access_ok(VERIFY_WRITE, (void*)dst_addr, out_area_len)) {
		printk(KERN_DEBUG "do_resize: not allowed to write %u byte to %p\n", out_area_len, (void*)dst_addr);
		return -EFAULT;
	}

	reg->sdr_inoff  = cmd_data->src_rect.linestep;
	reg->sdr_inadd  = ispmmu_map(to_phys_addr(src_addr), 2 * in_area_len);
	if( !reg->sdr_inadd ){
		goto error;
	}

	reg->sdr_outoff = cmd_data->dst_rect.linestep;
	reg->sdr_outadd = ispmmu_map(to_phys_addr(dst_addr), 2 * out_area_len);
	if( !reg->sdr_outadd ){
		goto error_unmap_in;
	}

	h_rsz = cmd_data->h_rsz;
	v_rsz = cmd_data->v_rsz;

	if (h_rsz >= 256)
		cbilin = 0;
	else
		cbilin = 1;
		
	/* data input type, 0: YUV422, 1: 8bit grey */
	if (cmd_data->format == 0)
		inptype = 0;
	else
		inptype = 1;

	if (h_rsz > 512) {
		/* resize factor >= 0.25, < 0.5: 4 phases, 7 taps */
		int cfo = (cmd_data->src_rect.width + 8) * 2 - ((cmd_data->dst_rect.width * h_rsz) >> 7);
		sph     = (cfo + 2) & 3;
		hsize   = ((64 * sph + (cmd_data->dst_rect.width - 1) * h_rsz + 32 ) >> 8) + 7;
	} else {
		/* resize factor >= 0.5, < 4: 8 phase, 4 taps */
		int cfo = (cmd_data->src_rect.width + 8) * 4 - ((cmd_data->dst_rect.width * h_rsz) >> 6);
		sph     = cfo & 7;
		hsize	= ((32 * sph + (cmd_data->dst_rect.width - 1) * h_rsz + 16) >> 8) + 7;
	}

	if (v_rsz > 512) {
		int cfo;
		
		if ( cmd_data->dst_rect.width > 640) {
			/* if vertical downsampling > 1/2, output buffer is 640 pixel max. */
			printk(KERN_DEBUG "do_resize: not allowed to ouput > 640 pixels and downsample > 1/2 at the same time.\n");
			goto error_unmap_out;
		}

		/* resize factor >= 0.25, < 0.5: 4 phases, 7 taps */
		cfo = (cmd_data->src_rect.height + 8) * 2 - ((cmd_data->dst_rect.height * v_rsz) >> 7);
		spv     = (cfo + 2) & 3;
		vsize   = ((64 * spv + (cmd_data->dst_rect.height - 1) * v_rsz + 32 ) >> 8) + 7;
	} else {
		/* resize factor >= 0.5, < 4: 8 phase, 4 taps */
		int cfo = (cmd_data->src_rect.height + 8) * 4 - ((cmd_data->dst_rect.height * v_rsz) >> 6);
		spv     = cfo & 7;
		vsize	= ((32 * spv + (cmd_data->dst_rect.height - 1) * v_rsz + 16) >> 8) + 4;
	}

	if ( (hsize < 16) || (vsize < 16) )
		goto error_unmap_out;

	/* set the registers */
	reg->cnt       = (cbilin << 29) | (1 << 28) | (inptype << 27) | (spv << 23) | ( sph << 20) | ((v_rsz-1) << 10) | ((h_rsz-1) << 0);
	reg->out_size  = (cmd_data->dst_rect.height << 16) | (cmd_data->dst_rect.width << 0);
	reg->in_start  = h_startpx;
	reg->in_size   = (vsize << 16) | (hsize << 0);
	
	in_surface  = cmd_data->src_rect.height * cmd_data->src_rect.width;
	out_surface = cmd_data->dst_rect.height * cmd_data->dst_rect.width;

	if( 1 || ( cmd_data->src_rect.width < 100 ) || ( out_surface > 4 * in_surface ) ){
		// wait 1 clock cycle before each read burst
		reg->read_delay = 0x1 << 10;
	} else {
		reg->read_delay = 0;
	}

	/* program filter coefficients */
	for (i = 0; i < 16; i++) {
		reg->hfilt[i] = (unsigned int)(cmd_data->h_coeff.coeff[i*2]) | ((unsigned int)(cmd_data->h_coeff.coeff[i*2+1]) << 16);
		reg->vfilt[i] = (unsigned int)(cmd_data->v_coeff.coeff[i*2]) | ((unsigned int)(cmd_data->v_coeff.coeff[i*2+1]) << 16);
	}

DBG	printk(KERN_DEBUG "do_param_blt: from %08X, %ix%i (%i) to %08X, %ix%i (%i)\n", 
			reg->sdr_inadd, cmd_data->src_rect.width, cmd_data->src_rect.height, 
			cmd_data->src_rect.linestep,
			reg->sdr_outadd, cmd_data->dst_rect.width, cmd_data->dst_rect.height,
			cmd_data->dst_rect.linestep);
DBG	printk(KERN_DEBUG "do_param_blt: insize:%ix%i, spv:%i, sph:%i, v_rsz:%i h_rsz:%i\n",
			hsize, vsize, spv, sph, v_rsz, h_rsz);

	cmd->vbl_sync = cmd_data->vbl_sync;
	
	return 0;

error_unmap_out:
	ispmmu_unmap(reg->sdr_outadd);
error_unmap_in:
	ispmmu_unmap(reg->sdr_inadd);
error:
	return -EINVAL;
}


static void watchdog_timeout( unsigned long data )
{
	unsigned long flags;

DBG	printk("watchdog_timeout\n");

	spin_lock_irqsave(&rsz_watchdog_lock, flags);
	if (omap_readl(RESZ_PCR) & 2) {
		/* still busy, come back later */
		mod_timer(&watchdog_timer, jiffies + msecs_to_jiffies(10));
	} else
		resz_complete_cmd();
	spin_unlock_irqrestore(&rsz_watchdog_lock, flags);
}

static void resz_kick( resz_regdata_t* regs )
{
	int i;
	unsigned long flags;

	/* diagnostic for resizer hang */
	if (omap_readl(RESZ_PCR) & 2)
		printk("resz_kick: resizer still busy!\n");
	
	spin_lock_irqsave(&rsz_register_lock, flags);
	
	/* for big upscales, the resizer writes much more than what it reads
	   this can cause some overflows in the write buffers so we have to introduce some delay */
	omap_writel(regs->read_delay, ISP_SBL_REQ_EXP);

	/* set the registers */
	omap_writel(regs->cnt,        RESZ_RSZ_CNT);
	omap_writel(regs->out_size,   RESZ_OUT_SIZE);
	omap_writel(regs->in_start,   RESZ_IN_START);
	omap_writel(regs->in_size,    RESZ_IN_SIZE);
	omap_writel(regs->sdr_inadd,  RESZ_SDR_INADD);
	omap_writel(regs->sdr_inoff,  RESZ_SDR_INOFF);
	omap_writel(regs->sdr_outadd, RESZ_SDR_OUTADD);
	omap_writel(regs->sdr_outoff, RESZ_SDR_OUTOFF);

	/* program filter coefficients */
	for (i = 0; i < 16; i++) {
		omap_writel(regs->hfilt[i], RESZ_HFILT(i));
		omap_writel(regs->vfilt[i], RESZ_VFILT(i));
	}

	omap_writel(omap_readl(RESZ_PCR) | 5, RESZ_PCR);

	/* start the watchdog, sometimes completion IRQ does not fire! */
	mod_timer(&watchdog_timer, jiffies + msecs_to_jiffies(10));

	spin_unlock_irqrestore(&rsz_register_lock, flags);
}

static resz_command_t *resz_new_command(int cmd)
{
	resz_command_t *new_cmd;
	
	new_cmd = kmalloc( sizeof (resz_command_t), GFP_KERNEL);
	if (new_cmd == NULL)
		return NULL;
		
	memset(new_cmd, 0, sizeof(resz_command_t));

	new_cmd->waiting = 0;
	new_cmd->cmd = cmd;
	new_cmd->err = 0;
	init_completion(&new_cmd->complete);
	
	return new_cmd;
}

static inline void resz_delete_command( resz_command_t *cmd )
{
	if( cmd->reg.sdr_inadd )
		ispmmu_unmap(cmd->reg.sdr_inadd);

	if( cmd->reg.sdr_outadd )
		ispmmu_unmap(cmd->reg.sdr_outadd);

	kfree(cmd);
}

static int resz_queue_cmd( resz_command_t *cmd )
{
	int tag;
	unsigned long flags;
	
	down_write(&rsz_queue_lock);
	
	/* generate a new command tag */
	tag = ++resz_current_tag;
	if (tag <= 0)
		tag = resz_current_tag = 1;
	cmd->cmd_tag = tag;

	spin_lock_irqsave(&rsz_cmd_lock, flags);
	if (current_cmd != NULL) {
		/* add job and release the lock */
		list_add_tail(&cmd->node, &resz_queue);
		spin_unlock_irqrestore(&rsz_cmd_lock, flags);
DBG		printk("resz_queue_cmd: job nr %i queued\n", tag);
	
	} else {
		/* no command running, release lock and start next one */
		request_clocks();
		
		spin_unlock_irqrestore(&rsz_cmd_lock, flags);
		
		/* add job to the queue and start it */
		list_add_tail(&cmd->node, &resz_queue);
		current_cmd = cmd;
		
		if (!cmd->vbl_sync)
			resz_kick(&cmd->reg);
		
DBG		printk("resz_queue_cmd: job nr %i queued & running\n", tag);
	} 

	up_write(&rsz_queue_lock);

	return tag;
}

static inline void resz_dequeue_cmd( resz_command_t *cmd )
{
	unsigned long flags;

	spin_lock_irqsave(&rsz_cmd_lock, flags);
	list_del(&cmd->node);
	spin_unlock_irqrestore(&rsz_cmd_lock, flags);
}

static void resz_job_reaper(struct work_struct *work)
{
	resz_command_t *cmd, *n;
	
	request_clocks();
	
	/* walk the queue, remove all completed asynchronous jobs */
	down_write(&rsz_queue_lock);
	list_for_each_entry_safe(cmd, n, &resz_queue, node) {
		if (cmd->complete.done && !cmd->waiting) {
DBG			printk("resz_job_reaper: removing job %i\n", cmd->cmd_tag);
			resz_dequeue_cmd(cmd);
			resz_delete_command(cmd);
		}
	}	
	up_write(&rsz_queue_lock);
}

static DECLARE_WORK(resz_worker, resz_job_reaper);

static void resz_complete_cmd( void )
{
	/* satisfy your paranoia once a day */
	if (current_cmd == NULL)
		return;
	
	/* wake up tasks waiting for the last command */
	current_cmd->err = 0;
	complete(&current_cmd->complete);
	
DBG	printk("rsz_complete_cmd: completed job %i (RESZ_PCR %x)\n", current_cmd->cmd_tag, omap_readl(RESZ_PCR));

	current_cmd = NULL;

	/* start the next command if queue's not empty */
	if (!list_empty(&resz_queue)) {
		resz_command_t* cmd;
		list_for_each_entry(cmd, &resz_queue, node) {
			
			/* skip completed commands */
			if (cmd->complete.done)
				continue;

DBG			printk("resz_complete_cmd: next job nr %i is type %i\n", cmd->cmd_tag, cmd->cmd);
	
			/* go! */
			current_cmd = cmd;
			if (!current_cmd->vbl_sync)
				resz_kick(&current_cmd->reg);

			/* break out of the loop */
DBG			printk(KERN_DEBUG "resz_complete_cmd: starting job %i\n", current_cmd->cmd_tag);
			break;
		}
	}

DBG	printk("resz_complete_cmd: starting resz_job_reaper\n");
	/* must not add a work item that's already queued */
	if (list_empty(&resz_worker.entry))
		queue_work(resz_workqueue, &resz_worker);
}

static void rsz_irq(unsigned long status, isp_vbq_callback_ptr arg1, void *arg2 )
{
	unsigned long flags;

DBG	printk("rsz_irq: status %08lX\n", status);

	spin_lock_irqsave(&rsz_watchdog_lock, flags);
	if (omap_readl(RESZ_PCR) & 2) {
		/* might happen if we race with the watchdog */
		printk("rsz_irq: RESZ is still busy!\n");
	} else {
		/* kill the watchdog */
		del_timer_sync(&watchdog_timer);
		/* complete the command */
		resz_complete_cmd();
	}
	spin_unlock_irqrestore(&rsz_watchdog_lock, flags);
}

static void vbl_irq(void *arg, u32 mask)
{
	if (current_cmd && current_cmd->vbl_sync) {
		current_cmd->vbl_sync = 0;
		resz_kick(&current_cmd->reg);
	}
}

static int vpssrsz_ioctl(struct inode * inode, struct file *filp,
	     unsigned int cmd, unsigned long arg)
{
	vpssrsz_param_cmd_t param_cmd_data;
	resz_command_t *rsz_cmd, *n;
	int cmd_tag;
	int ret = -EINVAL;

	request_clocks();
	
	switch (cmd) {
	case VPSSRSZ_CMD_PARAMBLT_ASYNC:
		/* copy argument, FAULT if pointer is invalid */
		if (copy_from_user(&param_cmd_data, (void __user *)arg, sizeof(param_cmd_data)))
			return -EFAULT;

		if (param_cmd_data.size != sizeof(param_cmd_data))
			return -EINVAL;

		/* create and queue the new job */
		rsz_cmd = resz_new_command(cmd);
		if (rsz_cmd == NULL)
			return -ENOMEM;
		
		/* verify the command, could be done before allocating it
		 * but failure is a rare case anyway */
		ret = prepare_command(rsz_cmd, &param_cmd_data);
		if (ret < 0) {
			resz_delete_command(rsz_cmd);
			break;
		}
		
		ret = resz_queue_cmd(rsz_cmd);
		if (ret < 0)
			resz_delete_command(rsz_cmd);

		/* command completes asynchronously, just return the tag */
		break;
		
	case VPSSRSZ_CMD_WAIT_SYNC:
		/*
		 * Walk the queue, try to find the tag.
		 * If the tag is found, wait for completion of the command,
		 * timeout after 2 seconds.
		 * If tag is not found, assume that the job was completed and
		 * return immediately.
		 */
		if (get_user(cmd_tag, (int __user*)arg))
			return -EFAULT;

		down_read(&rsz_queue_lock);
		list_for_each_entry_safe(rsz_cmd, n, &resz_queue, node) {
			if (rsz_cmd->cmd_tag == cmd_tag) {
				/* tell async reaper to skip this one */
				rsz_cmd->waiting = 1;
				/* it's safe to release the lock now */
				up_read(&rsz_queue_lock);

				ret = wait_for_completion_timeout(&rsz_cmd->complete, 2*HZ);

				/* as this might have taken rather long time, we need to request clocks again... */
				request_clocks();
				
				down_write(&rsz_queue_lock);
				resz_dequeue_cmd(rsz_cmd);
				up_write(&rsz_queue_lock);

				if (ret == 0 && !rsz_cmd->complete.done) {
					_dump_cmd(rsz_cmd);
					ret = -EIO;
				} else
					ret = rsz_cmd->err;

DBG				printk("wait_for_completion_timeout: removing job %i\n", rsz_cmd->cmd_tag);
				resz_delete_command(rsz_cmd);
				return ret;
			}
		}

		/* didn't find the tag, release lock and return successfully */
		up_read(&rsz_queue_lock);
		ret = 0;
		break;

	default:
		break;
	}

	return ret;
}

static int vpssrsz_open(struct inode * inode, struct file * filp)
{
	return 0;
}

static int vpssrsz_release(struct inode * inode, struct file * filp)
{
	return 0;
}

static struct file_operations vpssrsz_fops = {
	.owner	=	THIS_MODULE,
	.ioctl	=	vpssrsz_ioctl,
	.open	=	vpssrsz_open,
	.release=	vpssrsz_release,
};

static struct miscdevice vpssrsz_miscdev = {
	.minor	= VPSSRSZ_MINOR,
	.name	= VPSSRSZ_NAME,
	.fops	= &vpssrsz_fops,
};

/* +++++++++++++ End File operations ++++++++++++++*/

/* +++++++++++++ Platform Device ++++++++++++++++++*/

#define RSZ_OFFSET	0x0800
#define RSZ_EXTENT	0x0400

static int __init
vpssrsz_drv_probe(struct platform_device *dev)
{
	int ret;
	unsigned int mask = (DISPC_IRQ_VSYNC | DISPC_IRQ_EVSYNC_ODD | DISPC_IRQ_EVSYNC_EVEN); 
	
	// don't we need omap2_get_dss() here?
	isp_get();
	
	if( ispresizer_request() ){
		printk(KERN_ERR "vpssrsz: ispresizer_request failed\n");
		goto out_noirq;
	}
		

	ret = isp_set_callback(CBK_RESZ_DONE, rsz_irq, (isp_vbq_callback_ptr)NULL, (void *)NULL);
	if (ret < 0) {
		printk(KERN_ERR "%s: can't get IRQ for RESZDONE\n", VPSSRSZ_NAME);
		goto out_noirq;
	}

	isr_handle = omap_dispc_register_isr(vbl_irq, (void*)1, mask);
	if (isr_handle == NULL) {
		printk(KERN_ERR "vpssrsz: cannot register dispc isr\n");
		goto out_no_isr;
	}
	
	/* register our misc device */
	if ((ret = misc_register(&vpssrsz_miscdev)) < 0) {
		printk(KERN_ERR "vpssrsz: cannot register device (err=%i)\n", ret);
		goto out_no_miscdev;
	}

	resz_workqueue = create_workqueue("resz");

	isp_put();

	return ret;

out_no_miscdev:
	omap_dispc_unregister_isr(isr_handle);
out_no_isr:
	isp_unset_callback(CBK_RESZ_DONE);	
out_noirq:
	isp_put();
	return -ENODEV;
}

static int
vpssrsz_drv_remove(struct platform_device *dev)
{
	unsigned long flags;

	destroy_workqueue(resz_workqueue);

	misc_deregister(&vpssrsz_miscdev);
	
	omap_dispc_unregister_isr(isr_handle);

	isp_get();
	isp_unset_callback(CBK_RESZ_DONE);
	ispresizer_free();
	isp_put();

	del_timer_sync(&idle_timer);
	spin_lock_irqsave(&rsz_clock_request, flags);
	if ( clock_requested ) {
		isp_put();
		clock_requested = 0;
	}
	spin_unlock_irqrestore(&rsz_clock_request, flags);
	return 0;
}

static void vpssrsz_dev_release(struct device *dev)
{
}

static struct platform_device vpssrsz_device = {
	.name = VPSSRSZ_NAME,
	.id   = 100,
	.dev = {
		.release = vpssrsz_dev_release,
	},
};

static struct platform_driver vpssrsz_driver = {
	.probe		= vpssrsz_drv_probe,
	.remove		= vpssrsz_drv_remove,
	.driver = {
		   .bus = &platform_bus_type,
		   .name = VPSSRSZ_NAME,
	},
};


/* ++++++++++++++++++ Module Init +++++++++++++++++++++++*/

static int vpssrsz_init_module(void)
{
	int ret = platform_driver_register(&vpssrsz_driver);
	if( ret )
		return ret;

	ret = platform_device_register(&vpssrsz_device);
	if( ret ){
		platform_driver_unregister(&vpssrsz_driver);
		return ret;
	}

	return 0;
}

static void vpssrsz_cleanup_module(void)
{
	platform_device_unregister(&vpssrsz_device);
	platform_driver_unregister(&vpssrsz_driver);
}

/* Module information */
MODULE_AUTHOR("Matthias Welwarsky, Archos S.A.");
MODULE_DESCRIPTION("Image Resizer Driver");
MODULE_LICENSE("GPL");

module_init(vpssrsz_init_module);
module_exit(vpssrsz_cleanup_module);
