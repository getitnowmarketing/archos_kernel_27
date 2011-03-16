#ifndef __PRCM_DEBUG_H__
#define __PRCM_DEBUG_H__ 1

extern u32 current_vdd1_opp;
extern u32 current_vdd2_opp;
extern struct res_handle  *memret1;
extern struct res_handle  *memret2;
extern struct res_handle  *logret1;
int power_configuration_test(void);
int powerapi_test(void);
int clkapi_test(void);
int dpllapi_test(void);
int voltage_scaling_tst(u32 target_opp_id, u32 current_opp_id);
int voltage_scaling_set_of_test(void);
int frequency_scaling_test(void);

extern unsigned int rnd_rate_vdd1[7];
extern unsigned int rnd_rate_vdd2[3];

int nb_pre_test_func(struct notifier_block *n, unsigned long event, void *ptr)
{
	printk(KERN_INFO"Pre Notifier function called for level %lu\n", event);
	return 0;
}


int nb_post_test_func(struct notifier_block *n, unsigned long event, void *ptr)
{
	printk(KERN_INFO"Post Notifier function called for level %lu\n", event);
	return 0;
}

struct notifier_block nb_test_pre[5] = {{nb_pre_test_func, NULL, },
					{nb_pre_test_func, NULL, },
					{nb_pre_test_func, NULL, },
					{nb_pre_test_func, NULL, },
					{nb_pre_test_func, NULL, }
				};

struct notifier_block nb_test_post[5] = {{nb_post_test_func, NULL, },
					 {nb_post_test_func, NULL, },
					 {nb_post_test_func, NULL, },
					 {nb_post_test_func, NULL, },
					 {nb_post_test_func, NULL, }
				};

struct constraint_param {
	struct constraint_handle *handler_ptr;
	struct notifier_block *nb_test_pre_ptr;
	struct notifier_block *nb_test_post_ptr;
};

#endif /* #ifndef __PRCM_DEBUG_H__ */
