#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <mach/gpio.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <mach/mux.h>

#include "omap34xxcam.h"

#define SP7658_WDT_TIMEOUT		1500		// TimeOut for Flash mode
#define SP7658_TORCH_MODE_GPIO		55
#define SP7658_ENABLE_GPIO		56
#define SP7658_NAME			"sp7685"

 #define SP7658_FLAG_ENABLED				(1<<0)	// Flag for Enable pin
 #define SP7658_FLAG_MODE_FLASH				(1<<1)	// Flag for Flash mode(1), ~ for Torch mode (0)
 #define SP7658_FLAG_WDT				(1<<2)	// WDT timer is enabled

static struct v4l2_int_device sp7658_int_device;
static struct sp7658_private sp7658_dev;

struct sp7658_private {
	struct v4l2_int_device *v4l2_int_device;
	struct timer_list wdt;
	int flags;
	spinlock_t lock;
};

static struct v4l2_queryctrl sp7658_ctrls[] = {
	{
		.id			= V4L2_CID_FLASH_MODE,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flash mode",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
		.flags		= V4L2_CTRL_FLAG_UPDATE,
	},

	{
		.id			= V4L2_CID_FLASH_ENABLE,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flash enable",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.flags		= V4L2_CTRL_FLAG_UPDATE,
	},
};


static int find_vctrl(int id)
{
       int i;

       if (id < V4L2_CID_BASE)
               return -EDOM;

       for (i = (ARRAY_SIZE(sp7658_ctrls) - 1); i >= 0; i--) {
               if (sp7658_ctrls[i].id == id)
                       return i;
       }
       return -EINVAL;
}

static void sp7658_wdt_expire(unsigned long data)
{
	struct sp7658_private *flash = (struct sp7658_private *)data;

	if ((flash->flags & SP7658_FLAG_ENABLED) &&
	    (flash->flags & SP7658_FLAG_MODE_FLASH)){
		printk(" sp7658: Flash turnded off - WDT timeout of %dms has expired! \n",SP7658_WDT_TIMEOUT);
		omap_set_gpio_dataout(SP7658_ENABLE_GPIO, 0);
		flash->flags |= ~SP7658_FLAG_ENABLED;
	}
}
static int sp7658_ioctl_queryctrl(struct v4l2_int_device *s,
                               struct v4l2_queryctrl *qc)
{
	int i;
	i = find_vctrl(qc->id);

	if (i == -EINVAL) {
		qc->flags = V4L2_CTRL_FLAG_DISABLED;
	}

	if (i < 0) {
		return -EINVAL;
	}

	*qc = sp7658_ctrls[i];

	return 0;
}

static int sp7658_ioctl_g_ctrl(struct v4l2_int_device *s,
			struct v4l2_control *vc)
{
	return 0;
}

static int sp7658_ioctl_s_ctrl(struct v4l2_int_device *s,
				struct v4l2_control *vc)
{
	struct sp7658_private *flash = s->priv;

	switch (vc->id) {
	case V4L2_CID_FLASH_MODE:
		if(vc->value){
			printk("sp7658: Mode Flash. GPIO%d=0\n",SP7658_TORCH_MODE_GPIO);
			flash->flags |= SP7658_FLAG_MODE_FLASH;
			omap_set_gpio_dataout(SP7658_TORCH_MODE_GPIO, 0);
		} else {
			printk("sp7658: Mode Torch. GPIO%d=1\n",SP7658_TORCH_MODE_GPIO);
			flash->flags &= ~SP7658_FLAG_MODE_FLASH;
			omap_set_gpio_dataout(SP7658_TORCH_MODE_GPIO, 1);
		}
		break;

	case V4L2_CID_FLASH_ENABLE:
		if(vc->value){
			printk("sp7658: Turn ON flash. GPIO%d=1\n",SP7658_ENABLE_GPIO);
			flash->flags |= SP7658_FLAG_ENABLED;
			omap_set_gpio_dataout(SP7658_ENABLE_GPIO, 1);

			if(flash->flags & SP7658_FLAG_MODE_FLASH)
				mod_timer(&sp7658_dev.wdt, jiffies + msecs_to_jiffies(SP7658_WDT_TIMEOUT));
		} else {
			printk("sp7658: Turn OFF flash. GPIO%d=0\n",SP7658_ENABLE_GPIO	);
			flash->flags &= ~SP7658_FLAG_ENABLED;
			omap_set_gpio_dataout(SP7658_ENABLE_GPIO, 0);
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int sp7658_ioctl_s_power(struct v4l2_int_device *s,
				 enum v4l2_power state)
{
	return 0;
}

static struct omap34xxcam_hw_config sp7658_omap34xxcam_hw_config = {
	.dev_index	= 0,
	.dev_minor	= 0,
	.dev_type	= OMAP34XXCAM_SLAVE_FLASH,
	.u		= {
		.flash		= {
		 },
	},
};

static int sp7658_ioctl_g_priv(struct v4l2_int_device *s, void *priv)
{

	*(struct omap34xxcam_hw_config *)priv = sp7658_omap34xxcam_hw_config;
	
	return 0;
}

static struct v4l2_int_ioctl_desc sp7658_ioctl_desc[] = {
	{ vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)sp7658_ioctl_queryctrl },
	{ vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)sp7658_ioctl_g_ctrl },
	{ vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)sp7658_ioctl_s_ctrl },
	{ vidioc_int_s_power_num, (v4l2_int_ioctl_func *)sp7658_ioctl_s_power },
	{ vidioc_int_g_priv_num, (v4l2_int_ioctl_func *)sp7658_ioctl_g_priv },
};

static struct v4l2_int_slave sp7658_slave = {
	.ioctls = sp7658_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(sp7658_ioctl_desc),
};


static struct sp7658_private sp7658_dev = {
	.v4l2_int_device = &sp7658_int_device,
	.flags = 0,
};

static struct v4l2_int_device sp7658_int_device = {
	.module = THIS_MODULE,
	.name = SP7658_NAME,
	.priv = &sp7658_dev,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &sp7658_slave,
	},
};

static int __init sp7658_init(void)
{
	int err;

 	omap_set_gpio_direction(SP7658_TORCH_MODE_GPIO, GPIO_DIR_OUTPUT);
 	omap_set_gpio_direction(SP7658_ENABLE_GPIO, GPIO_DIR_OUTPUT);

	init_timer(&sp7658_dev.wdt);
	setup_timer(&sp7658_dev.wdt, sp7658_wdt_expire, (unsigned long)&sp7658_dev );
	
	spin_lock_init(&sp7658_dev.lock);	

	err = v4l2_int_device_register(&sp7658_int_device);
	if (err != 0){
		printk(KERN_ERR "Could not register sp7658 as v4l2_int_device\n");
		goto sp7658_init_exit1;
	}

	return 0;
	
sp7658_init_exit1:
	return -ENODEV;
}

static void __exit sp7658_exit(void)
{
	unsigned long flags;

	v4l2_int_device_unregister(&sp7658_int_device);

	omap_set_gpio_direction(SP7658_TORCH_MODE_GPIO, GPIO_DIR_INPUT);
	omap_free_gpio(SP7658_TORCH_MODE_GPIO);

	omap_set_gpio_direction(SP7658_ENABLE_GPIO, GPIO_DIR_INPUT);
	omap_free_gpio(SP7658_ENABLE_GPIO);

	spin_lock_irqsave(&sp7658_dev.lock, flags);

	if (sp7658_dev.flags & SP7658_FLAG_WDT) {
		del_timer(&sp7658_dev.wdt);
		sp7658_dev.flags &= ~SP7658_FLAG_WDT;
	}

	spin_unlock_irqrestore(&sp7658_dev.lock, flags);
}

/*
 * FIXME: isn't ready (?) at module_init stage, so use
 * late_initcall for now.
 */
late_initcall(sp7658_init);
module_exit(sp7658_exit);

MODULE_AUTHOR("Ivaylo Petrov <ivpetrov@mm-sol.com");
MODULE_DESCRIPTION("SP7658 flash pump - v4l2 device");
MODULE_LICENSE("GPL");
