#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <mach/board-archos.h>
#include <mach/gpio.h>
#ifdef CONFIG_OMAP3_PM
#include "prcm-regs.h"
#include "ti-compat.h"
#include <mach/prcm_34xx.h>
#endif


#ifdef CONFIG_OMAP3_PM
#define CONTROL_SYSC_SMARTIDLE  	(0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE   	(0x1)

/*
 * Board DDR timings used during frequency changes
 */
struct dvfs_config omap3_vdd2_config[PRCM_NO_VDD2_OPPS] = {
#ifdef CONFIG_OMAP3_CORE_166MHZ
	{
	/* SDRC CS0/CS1 values 83MHZ*/
	/* not optimized at 1/2 speed except for RFR */
	{{0x00025801, 0x629db4c6, 0x00012214},    /* cs 0 */
	 {0x00025801, 0x629db4c6, 0x00012214} },  /* cs 1 */
	},

	/* SDRC CS0/CS1 values 166MHZ*/
	{
	{{0x0004e201, 0xaa9db4c6, 0x00011517},
	 {0x0004e201, 0xaa9db4c6, 0x00011517} },
	},
#elif defined(CONFIG_OMAP3_CORE_133MHZ)
	{
	/* SDRC CS0/CS1 values 66MHZ*/
	{{0x0001ef01, 0x8a99b485, 0x00011412},
	 {0x0001ef01, 0x8a99b485, 0x00011412} },
	},

	/* SDRC CS0/CS1 values 133MHZ*/
	{
	{{0x0003de01, 0x8a99b485, 0x00011412},
	 {0x0003de01, 0x8a99b485, 0x00011412} },
	},
#endif
};
#endif

int __init archosg6_init(void)
{
	return 0;
}
