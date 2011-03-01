#ifndef __LINUX_HDD_DPM_H
#define __LINUX_HDD_DPM_H

struct hdd_dpm_ops;

typedef int (*hdd_dpm_suspend)(struct hdd_dpm_ops *dpm_ops);
typedef int (*hdd_dpm_resume)(struct hdd_dpm_ops *dpm_ops);
typedef int (*hdd_dpm_syncdev)(struct hdd_dpm_ops *dpm_ops);
typedef unsigned long (*hdd_dpm_iocount)(struct hdd_dpm_ops *dpm_ops);

struct hdd_dpm_ops {
	char name[32];
	hdd_dpm_suspend suspend;
	hdd_dpm_resume resume;
	hdd_dpm_syncdev sync;
	hdd_dpm_iocount get_iocount;
	volatile long pm_state;
	unsigned long min_idle;
	unsigned shutdown:1;
	unsigned suspend_locked:1;
};

#ifdef CONFIG_HDD_DPM
extern int hdd_dpm_register_dev(struct hdd_dpm_ops *dpm_ops);
extern int hdd_dpm_unregister_dev(struct hdd_dpm_ops *dpm_ops);
#else
static inline int hdd_dpm_register_dev(struct hdd_dpm_ops *dpm_ops) { return 0; }
static inline int hdd_dpm_unregister_dev(struct hdd_dpm_ops *dpm_ops) { return 0; }
#endif

#endif /* __LINUX_HDD_DPM_H */
