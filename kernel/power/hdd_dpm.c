/*
 * kernel/power/hdd_dpm.c - Dynamic HDD power management
 * Copyright (c) 2008 Matthias Welwarsky <welwarsky@archos.com>
 *
 * This file is released under the GPLv2.
 *
 */

#include <linux/kernel.h>
#include <linux/hdd_dpm.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/wait.h>
#include <linux/freezer.h>
#include <linux/reboot.h>
#include <linux/delay.h>

#include <linux/semaphore.h>
#include <asm/uaccess.h>

#define DBG if(0)
#define MAX_ENTRIES 32

#define PROC_READ_RETURN(page,start,off,count,eof,len) \
{					\
	len -= off;			\
	if (len < count) {		\
		*eof = 1;		\
		if (len <= 0)		\
			return 0;	\
	} else				\
		len = count;		\
	*start = page + off;		\
	return len;			\
}

struct thread_data {
	struct task_struct* handle;
	struct hdd_dpm_ops *dpm_ops;
	unsigned long io_count;
	int idle_timeout;
	int timeout;
	int pm_request;
	wait_queue_head_t wq;
	struct proc_dir_entry* proc;
	struct semaphore lock;
	int last_pm_state;
};

struct hdpwrd_proc_entry {
	const char	*name;
	mode_t		mode;
	read_proc_t	*read_proc;
	write_proc_t	*write_proc;
};

static struct thread_data *thread_instance[MAX_ENTRIES];
static struct proc_dir_entry *proc_hdpwrd_root;

static inline unsigned long get_drive_io(struct hdd_dpm_ops *dpm_ops)
{
	return dpm_ops->get_iocount(dpm_ops);
}

static int pwr_check_thread(void* data)
{
	struct thread_data *this = data;
	long wait_time = HZ;

	set_freezable();
	
	do {
		unsigned long new_stats;

		this->pm_request = -1;
		wait_time = wait_event_freezable_timeout(this->wq, this->pm_request != -1, wait_time);

		// if signalled but not requested to stop, ignore the signal
		if (wait_time < 0) {
			wait_time = HZ;
			continue;
		}

		// woken up by a request
		if (this->pm_request != -1) {
			struct hdd_dpm_ops *dpm_ops = this->dpm_ops;
			int min_wait;
			
			if (dpm_ops->pm_state != -1 && this->pm_request != dpm_ops->pm_state) {
				if (this->pm_request == PM_SUSPEND_ON) {
DBG					printk(KERN_DEBUG "hdpwrd: resuming drive on user request\n");
					dpm_ops->resume(dpm_ops);
					this->timeout = this->idle_timeout;
					wait_time = HZ;
				} else {
DBG					printk(KERN_DEBUG "hdpwrd: suspending drive on user request\n");
					min_wait = dpm_ops->min_idle-(this->idle_timeout-this->timeout);
					if (min_wait < 0)
						this->timeout = 0;
					else
						this->timeout = min_wait;
					wait_time = 0;
DBG					printk(KERN_DEBUG "hdpwrd: min_wait %i\n", min_wait);
				}
				this->pm_request = -1;
			}
		}

		// not yet done waiting
		if (wait_time)
			continue;

		wait_time = HZ;

		// no timeout at all, leave it spinning
		if (this->idle_timeout == 0)
			continue;

		// timed out, run state machine
		new_stats = get_drive_io(this->dpm_ops);
		
		down(&this->lock);
		if (!new_stats || new_stats != this->io_count) {
			/* renew the timeout */
DBG			printk(KERN_DEBUG "hdpwrd: new_stats: %ld old_stats: %ld\n", 
				new_stats, this->io_count);
			this->io_count = new_stats;
			this->timeout = this->idle_timeout;
		} else {
			struct hdd_dpm_ops *dpm_ops = this->dpm_ops;
			if (dpm_ops->pm_state == PM_SUSPEND_ON) {
DBG				printk(KERN_DEBUG "hdpwrd: timeout left %i\n", this->timeout);
				if (this->timeout == 0) {
					if (dpm_ops->sync)
						dpm_ops->sync(dpm_ops);
					if (unlikely(dpm_ops->suspend(dpm_ops)) < 0) {
DBG						printk(KERN_DEBUG "hdpwrd: suspending drive failed\n");
						/* retry after one second */
					} else {
DBG						printk(KERN_DEBUG "hdpwrd: suspending drive on timeout\n");
						this->timeout = this->idle_timeout;
					}
				} else
				if (this->timeout > 0) {
					this->timeout--;
				}
			} 
		}
		up(&this->lock);

	} while (!kthread_should_stop());

	return 0;
}

static int proc_hdpwrd_read_timeout(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct thread_data *this = data;
	int len;
	
	len = sprintf(page, "%i\n", this->idle_timeout);
	PROC_READ_RETURN(page,start,off,count,eof,len);
}

static int proc_hdpwrd_read_state(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	struct thread_data *this = data;
	int len;
	
	len = sprintf(page, "%li\n", this->dpm_ops->pm_state);
	PROC_READ_RETURN(page,start,off,count,eof,len);
}

static int proc_hdpwrd_write_timeout(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	struct thread_data* this = data;
	int idle_timeout;
	char *buf, *s, *q;
	
	if (!capable(CAP_SYS_ADMIN))
		return -EACCES;

	if (count >= PAGE_SIZE)
		return -EINVAL;
	
	s = buf = (char *)__get_free_page(GFP_USER);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		free_page((unsigned long)buf);
		return -EFAULT;
	}

	buf[count] = '\0';

	/*
	 * Skip over leading whitespace
	 */
	while (count && isspace(*s)) {
		--count;
		++s;
	}

	q = s;
	idle_timeout = simple_strtol(s, &q, 10);
	if (idle_timeout < this->dpm_ops->min_idle)
		idle_timeout = this->dpm_ops->min_idle;
	this->idle_timeout = idle_timeout;
	
DBG	printk(KERN_DEBUG "hdpwrd: new idle_timeout %i\n", this->idle_timeout);
	
	free_page((unsigned long)buf);
	return count;
}

static int proc_hdpwrd_write_state(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	struct thread_data* this = data;
	struct hdd_dpm_ops *dpm_ops = this->dpm_ops;
	char *buf, *s, *q;
	long pm_state;
	
	if (!capable(CAP_SYS_ADMIN))
		return -EACCES;

	if (count >= PAGE_SIZE)
		return -EINVAL;
	
	s = buf = (char *)__get_free_page(GFP_USER);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		free_page((unsigned long)buf);
		return -EFAULT;
	}

	buf[count] = '\0';

	/*
	 * Skip over leading whitespace
	 */
	while (count && isspace(*s)) {
		--count;
		++s;
	}

	q = s;
	pm_state = simple_strtol(s, &q, 10);
	/* free the buffer, we won't need it any more */
	free_page((unsigned long)buf);
	
	/* safety check: if the drive's power state is in transition, do nothing */
	if (dpm_ops->pm_state < 0)
		return -EAGAIN;
	
	switch (pm_state) {
	case PM_SUSPEND_ON:
		if (dpm_ops->pm_state != pm_state) {
			this->timeout = this->idle_timeout;
			this->pm_request = pm_state;
			wake_up_interruptible(&this->wq);			
		}
		break;
	case PM_SUSPEND_STANDBY:
		if (dpm_ops->pm_state != pm_state) {
			this->pm_request = pm_state;
			wake_up_interruptible(&this->wq);			
		}
		break;
	
	default:
		return -EINVAL;
	}
	
	return count;
}

struct hdpwrd_proc_entry drive_proc_entries[] = {
	{ "timeout", S_IFREG|S_IRUGO, proc_hdpwrd_read_timeout, proc_hdpwrd_write_timeout },
	{ "state",   S_IFREG|S_IRUGO|S_IWUSR, proc_hdpwrd_read_state,   proc_hdpwrd_write_state },
};

static struct proc_dir_entry *create_drive_entry(struct thread_data *this, struct proc_dir_entry* parent)
{
	struct proc_dir_entry *entry;
	struct proc_dir_entry *dir;
	int n;
	
	dir = proc_mkdir(this->dpm_ops->name, parent);
	if (!dir)
		return 0;
		
	for (n=0; n < ARRAY_SIZE(drive_proc_entries); n++) {
		entry = create_proc_entry(drive_proc_entries[n].name, drive_proc_entries[n].mode, dir);
		if (!entry)
			return 0;
		entry->nlink      = 1;
		entry->data       = this;
		entry->read_proc  = drive_proc_entries[n].read_proc;
		entry->write_proc = drive_proc_entries[n].write_proc;
	}

	return dir;
}

static int hdd_dpm_storetd(struct thread_data* td)
{
	int n;
	
	for (n = 0; n < MAX_ENTRIES; n++) {
		/* slot is not free, continue */
		if (thread_instance[n] != NULL)
			continue;

		thread_instance[n] = td;
		break;
	}
	
	if (n == MAX_ENTRIES) {
DBG		printk(KERN_DEBUG "hdd_dpm: no free slots\n");
		return -ENOMEM;
	}

	return 0;
}

static int hdd_dpm_removetd(const struct thread_data *td)
{
	int n;
	
	for (n = 0; n < MAX_ENTRIES; n++) {
		if (thread_instance[n] == td) {
			thread_instance[n] = NULL;
			break;
		}
	}
	
	if (n == MAX_ENTRIES) {
DBG		printk(KERN_DEBUG "hdd_dpm: entry not found\n");
		return -ENODEV;
	}

	return 0;
}

static struct thread_data *hdd_dpm_findtd(const struct hdd_dpm_ops *dpm_ops)
{
	int n;

	for (n = 0; n < MAX_ENTRIES; n++) {
		if (thread_instance[n] == NULL)
			continue;
			
		if (thread_instance[n]->dpm_ops == dpm_ops)
			break;
	}

	if (n == MAX_ENTRIES) {
DBG		printk(KERN_DEBUG "hdd_dpm: entry not found\n");
		return NULL;
	}

	return thread_instance[n];
}

int hdd_dpm_register_dev(struct hdd_dpm_ops *dpm_ops)
{
	struct thread_data *td;

DBG	printk(KERN_DEBUG "hdd_dpm_register_dev (%s)\n", dpm_ops->name);

	td = kzalloc(sizeof(struct thread_data), GFP_KERNEL);
	if (td == NULL)
		return -ENOMEM;

	if (hdd_dpm_storetd(td)) {
		kfree(td);
		return -ENOMEM;
	}

	td->idle_timeout = 20;
	if (td->idle_timeout < dpm_ops->min_idle)
		td->idle_timeout = dpm_ops->min_idle; 

	td->dpm_ops       = dpm_ops;
	td->timeout       = td->idle_timeout;
	td->io_count      = 0;
	td->last_pm_state = -1;
	td->proc          = create_drive_entry(td, proc_hdpwrd_root);

	init_waitqueue_head(&td->wq);
	init_MUTEX(&td->lock);

	td->handle = kthread_run(pwr_check_thread, td, "hdd-dpm/%s", dpm_ops->name);
	if (IS_ERR(td->handle)) {
		int ret = PTR_ERR(td->handle);
		hdd_dpm_removetd(td);
		kfree(td);
		return ret;
	}

	return 0;
}

int hdd_dpm_unregister_dev(struct hdd_dpm_ops *dpm_ops)
{
	int n;
	struct thread_data *td;

DBG	printk(KERN_DEBUG "hdd_dpm_unregister_dev (%s)\n", dpm_ops->name);

	if ((td = hdd_dpm_findtd(dpm_ops)) == NULL)
		return -ENODEV;

	kthread_stop(td->handle);

	for (n = 0; n < ARRAY_SIZE(drive_proc_entries); n++) {
		remove_proc_entry(drive_proc_entries[n].name, td->proc);
	}
	remove_proc_entry(td->dpm_ops->name, proc_hdpwrd_root);

	hdd_dpm_removetd(td);
	kfree(td);

	return 0;
}

static int hdd_dpm_reboot(struct notifier_block *notifier, unsigned long val,
                       void *v)
{
	int n;

	for (n = 0; n < MAX_ENTRIES; n++) {
		struct hdd_dpm_ops *dpm_ops;
		struct thread_data *this;

		if (thread_instance[n] == NULL)
			continue;
			
		this = thread_instance[n];         
		dpm_ops = this->dpm_ops;
		if (dpm_ops->shutdown && dpm_ops->pm_state == PM_SUSPEND_ON) {
			int min_wait;
			
			min_wait = dpm_ops->min_idle-(this->idle_timeout-this->timeout);
			if (min_wait > 0)
				msleep(min_wait*HZ);
			if (dpm_ops->sync)
				dpm_ops->sync(dpm_ops);
			dpm_ops->suspend(dpm_ops);
		}
	}

	return NOTIFY_OK;
}

static int hdd_dpm_notify_suspend(struct notifier_block *notifier, 
		unsigned long val, void *v)
{
	int n;
	int ret = NOTIFY_OK;

	if (val == PM_POST_SUSPEND) {
		for (n = 0; n < MAX_ENTRIES; n++) {
			struct hdd_dpm_ops *dpm_ops;
			struct thread_data *this;

			if (thread_instance[n] == NULL)
				continue;
		
			this = thread_instance[n];
			down(&this->lock);
			dpm_ops = this->dpm_ops;
			if (dpm_ops->suspend_locked)
				dpm_ops->suspend_locked = 0;
			up(&this->lock);
		}
	} else
	if (val == PM_SUSPEND_PREPARE) {
		for (n = 0; n < MAX_ENTRIES; n++) {
			struct hdd_dpm_ops *dpm_ops;
			struct thread_data *this;
	
			if (thread_instance[n] == NULL)
				continue;
				
			this = thread_instance[n];
			down(&this->lock);
			dpm_ops = this->dpm_ops;
			if (dpm_ops->pm_state == PM_SUSPEND_ON) {
				int min_wait;
				
				min_wait = dpm_ops->min_idle-(this->idle_timeout-this->timeout);
				if (min_wait > 0)
					msleep(min_wait*HZ);
				if (dpm_ops->sync)
					dpm_ops->sync(dpm_ops);
				dpm_ops->suspend_locked = 1;
				dpm_ops->suspend(dpm_ops);
			} else {
				dpm_ops->suspend_locked = 1;
			}
			up(&this->lock);
		}
	} else
	if (val == PM_POST_FREEZE) {
		for (n = 0; n < MAX_ENTRIES; n++) {
			struct hdd_dpm_ops *dpm_ops;
			struct thread_data *this;

			if (thread_instance[n] == NULL)
				continue;

			this = thread_instance[n];
			dpm_ops = this->dpm_ops;
			down(&this->lock);
			if (dpm_ops->suspend_locked == 1 && dpm_ops->pm_state != PM_SUSPEND_STANDBY) {
				ret = NOTIFY_BAD;
				up(&this->lock);
				break;
			}
			up(&this->lock);
		}
	}
	
	return ret;	
}

static struct notifier_block hdd_dpm_reboot_notifier = {
	.notifier_call = hdd_dpm_reboot,
	.priority = 0,
};

static struct notifier_block hdd_dpm_suspend_notifier = {
	.notifier_call = hdd_dpm_notify_suspend,
	.priority = 0,
};

static int __init hdd_dpm_init(void)
{
	proc_hdpwrd_root = proc_mkdir("hdpwrd", NULL);
	register_reboot_notifier(&hdd_dpm_reboot_notifier);
	register_pm_notifier(&hdd_dpm_suspend_notifier);
	return 0;
}

subsys_initcall(hdd_dpm_init);

EXPORT_SYMBOL(hdd_dpm_register_dev);
EXPORT_SYMBOL(hdd_dpm_unregister_dev);
