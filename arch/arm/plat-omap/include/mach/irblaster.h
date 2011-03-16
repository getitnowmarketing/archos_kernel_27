#ifndef __MACH_IRBLASTER_H
#define __MACH_IRBLASTER_H

enum {
	IRBLASTER_PWM = 0,
	IRBLASTER_TIMER_CTRL,
	PWM_NUMBER,
};

struct omap_pwm_timer {
	char *name;
	int no;
	int source;
	struct omap_dm_timer *timer;
	int rate;
	unsigned int period;
	void (*init)(void);
	int mux_config;
};

#endif
