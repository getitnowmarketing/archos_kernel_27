/*
 * linux/kernel/irq/proc.c
 *
 * Copyright (C) 1992, 1998-2004 Linus Torvalds, Ingo Molnar
 *
 * This file contains the /proc/irq/ handling code.
 */

#include <linux/irq.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>

#include "internals.h"

static struct proc_dir_entry *root_irq_dir;

#ifdef CONFIG_SMP

static int irq_affinity_proc_show(struct seq_file *m, void *v)
{
	struct irq_desc *desc = irq_desc + (long)m->private;
	cpumask_t *mask = &desc->affinity;

#ifdef CONFIG_GENERIC_PENDING_IRQ
	if (desc->status & IRQ_MOVE_PENDING)
		mask = &desc->pending_mask;
#endif
	seq_cpumask(m, mask);
	seq_putc(m, '\n');
	return 0;
}

#ifndef is_affinity_mask_valid
#define is_affinity_mask_valid(val) 1
#endif

int no_irq_affinity;
static ssize_t irq_affinity_proc_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int irq = (int)(long)PDE(file->f_path.dentry->d_inode)->data;
	cpumask_t new_value;
	int err;

	if (!irq_desc[irq].chip->set_affinity || no_irq_affinity ||
	    irq_balancing_disabled(irq))
		return -EIO;

	err = cpumask_parse_user(buffer, count, new_value);
	if (err)
		return err;

	if (!is_affinity_mask_valid(new_value))
		return -EINVAL;

	/*
	 * Do not allow disabling IRQs completely - it's a too easy
	 * way to make the system unusable accidentally :-) At least
	 * one online CPU still has to be targeted.
	 */
	if (!cpus_intersects(new_value, cpu_online_map))
		/* Special case for empty set - allow the architecture
		   code to set default SMP affinity. */
		return irq_select_affinity(irq) ? -EINVAL : count;

	irq_set_affinity(irq, new_value);

	return count;
}

static int irq_affinity_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, irq_affinity_proc_show, PDE(inode)->data);
}

static const struct file_operations irq_affinity_proc_fops = {
	.open		= irq_affinity_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= irq_affinity_proc_write,
};

static int default_affinity_show(struct seq_file *m, void *v)
{
	seq_cpumask(m, &irq_default_affinity);
	seq_putc(m, '\n');
	return 0;
}

static ssize_t default_affinity_write(struct file *file,
		const char __user *buffer, size_t count, loff_t *ppos)
{
	cpumask_t new_value;
	int err;

	err = cpumask_parse_user(buffer, count, new_value);
	if (err)
		return err;

	if (!is_affinity_mask_valid(new_value))
		return -EINVAL;

	/*
	 * Do not allow disabling IRQs completely - it's a too easy
	 * way to make the system unusable accidentally :-) At least
	 * one online CPU still has to be targeted.
	 */
	if (!cpus_intersects(new_value, cpu_online_map))
		return -EINVAL;

	irq_default_affinity = new_value;

	return count;
}

static int default_affinity_open(struct inode *inode, struct file *file)
{
	return single_open(file, default_affinity_show, NULL);
}

static const struct file_operations default_affinity_proc_fops = {
	.open		= default_affinity_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= default_affinity_write,
};
#endif

static int irq_spurious_read(char *page, char **start, off_t off,
				  int count, int *eof, void *data)
{
	struct irq_desc *d = &irq_desc[(long) data];
	return sprintf(page, "count %u\n"
			     "unhandled %u\n"
			     "last_unhandled %u ms\n",
			d->irq_count,
			d->irqs_unhandled,
			jiffies_to_msecs(d->last_unhandled));
}

#define MAX_NAMELEN 128

static int name_unique(unsigned int irq, struct irqaction *new_action)
{
	struct irq_desc *desc = irq_desc + irq;
	struct irqaction *action;
	unsigned long flags;
	int ret = 1;

	spin_lock_irqsave(&desc->lock, flags);
	for (action = desc->action ; action; action = action->next) {
		if ((action != new_action) && action->name &&
				!strcmp(new_action->name, action->name)) {
			ret = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&desc->lock, flags);
	return ret;
}

void register_handler_proc(unsigned int irq, struct irqaction *action)
{
	char name [MAX_NAMELEN];

	if (!irq_desc[irq].dir || action->dir || !action->name ||
					!name_unique(irq, action))
		return;

	memset(name, 0, MAX_NAMELEN);
	snprintf(name, MAX_NAMELEN, "%s", action->name);

	/* create /proc/irq/1234/handler/ */
	action->dir = proc_mkdir(name, irq_desc[irq].dir);
}

#undef MAX_NAMELEN

#define MAX_NAMELEN 10

void register_irq_proc(unsigned int irq)
{
	char name [MAX_NAMELEN];
	struct proc_dir_entry *entry;

	if (!root_irq_dir ||
		(irq_desc[irq].chip == &no_irq_chip) ||
			irq_desc[irq].dir)
		return;

	memset(name, 0, MAX_NAMELEN);
	sprintf(name, "%d", irq);

	/* create /proc/irq/1234 */
	irq_desc[irq].dir = proc_mkdir(name, root_irq_dir);

#ifdef CONFIG_SMP
	/* create /proc/irq/<irq>/smp_affinity */
	proc_create_data("smp_affinity", 0600, irq_desc[irq].dir,
			 &irq_affinity_proc_fops, (void *)(long)irq);
#endif

	entry = create_proc_entry("spurious", 0444, irq_desc[irq].dir);
	if (entry) {
		entry->data = (void *)(long)irq;
		entry->read_proc = irq_spurious_read;
	}
}

#undef MAX_NAMELEN

void unregister_handler_proc(unsigned int irq, struct irqaction *action)
{
	if (action->dir)
		remove_proc_entry(action->dir->name, irq_desc[irq].dir);
}

void register_default_affinity_proc(void)
{
#ifdef CONFIG_SMP
	proc_create("irq/default_smp_affinity", 0600, NULL,
		    &default_affinity_proc_fops);
#endif
}

void init_irq_proc(void)
{
	int i;

	/* create /proc/irq */
	root_irq_dir = proc_mkdir("irq", NULL);
	if (!root_irq_dir)
		return;

	register_default_affinity_proc();

	/*
	 * Create entries for all existing IRQs.
	 */
	for (i = 0; i < NR_IRQS; i++)
		register_irq_proc(i);
}

