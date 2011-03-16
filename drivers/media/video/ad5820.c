/*
 * drivers/media/video/ad5820.c
 *
 * AD5820 Coil Motor (LENS) driver
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Contributors:
 * 	Troy Laramy <t-laramy@ti.com>
 * 	Mohit Jalori <mjalori@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>
#include <mach/gpio.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/device.h>

#include "ad5820.h"

#define DRIVER_NAME  "ad5820"

static int
ad5820_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __exit ad5820_remove(struct i2c_client *client);
int position_global;

struct ad5820_device {
	const struct ad5820_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	int opened;
	u16 current_lens_posn;
	u16 saved_lens_posn;
	int state;
	int power_state;

    struct ad5820_micro_steps_list mstep_list;
    struct completion mstep_completion;
};

static const struct i2c_device_id ad5820_id[] = {
	{ AD5820_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad5820_id);

static struct i2c_driver ad5820_i2c_driver = {
	.driver = {
		.name = AD5820_NAME,
		.owner = THIS_MODULE,
	},
	.probe = ad5820_probe,
	.remove = __exit_p(ad5820_remove),
	.id_table = ad5820_id,
};

static struct ad5820_device ad5820 = {
	.state = LENS_NOT_DETECTED,
	.current_lens_posn = DEF_LENS_POSN,
};

static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
	{
		{
			.id = V4L2_CID_FOCUS_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Lens Position",
			.minimum = 0,
			.maximum = MAX_FOCUS_POS,
			.step = LENS_POSN_STEP,
			.default_value = DEF_LENS_POSN,
		},
		.current_value = DEF_LENS_POSN,
	},
	{
		{
			.id = V4L2_CID_FOCUS_SCRIPT,
			.type = V4L2_CTRL_TYPE_INTEGER64,
			.name = "Lens script",
			.default_value = 0,
		},
		.current_value = 0,
	}
};

static struct i2c_driver ad5820_i2c_driver;

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/**
 * camaf_reg_read - Reads a value from a register in AD5820 Coil driver device.
 * @client: Pointer to structure of I2C client.
 * @value: Pointer to u16 for returning value of register to read.
 *
 * Returns zero if successful, or non-zero otherwise.
 **/
static int camaf_reg_read(struct i2c_client *client, u16 *value)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = I2C_M_RD;
	msg->len = 2;
	msg->buf = data;

	data[0] = 0;
	data[1] = 0;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		*value = err = ((data[0] & 0xFF) << 8) | (data[1]);
		return 0;
	}
	return err;
}

/**
 * camaf_reg_write - Writes a value to a register in AD5820 Coil driver device.
 * @client: Pointer to structure of I2C client.
 * @value: Value of register to write.
 *
 * Returns zero if successful, or non-zero otherwise.
 **/
static int camaf_reg_write(struct i2c_client *client, u16 value)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	data[0] = (u8)(value >> 8);
	data[1] = (u8)(value & 0xFF);

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0)
		return 0;

	if (retry <= AD5820_I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retry ... %d", retry);
		retry++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto again;
	}
	return err;
}

/**
 * ad5820_detect - Detects AD5820 Coil driver device.
 * @client: Pointer to structure of I2C client.
 *
 * Returns 0 if successful, -1 if camera is off or if test register value
 * wasn't stored properly, or returned errors from either camaf_reg_write or
 * camaf_reg_read functions.
 **/
static int ad5820_detect(struct i2c_client *client)
{
	int err = 0;
	u16 wposn = 0, rposn = 0;
	u16 posn = 0x05;

	wposn = (CAMAF_AD5820_POWERDN(CAMAF_AD5820_ENABLE) |
						CAMAF_AD5820_DATA(posn));

	err = camaf_reg_write(client, wposn);
	if (err) {
		printk(KERN_ERR "Unable to write AD5820 \n");
		return err;
	}

	err = camaf_reg_read(client, &rposn);
	if (err) {
		printk(KERN_ERR "Unable to read AD5820 \n");
		return err;
	}

	if (wposn != rposn) {
		printk(KERN_ERR "W/R MISMATCH!\n");
		return -1;
	}
	posn = 0;
	wposn = (CAMAF_AD5820_POWERDN(CAMAF_AD5820_ENABLE) |
						CAMAF_AD5820_DATA(posn));
	err = camaf_reg_write(client, wposn);

	return err;
}

/**
 * ad5820_af_setfocus - Sets the desired focus.
 * @posn: Desired focus position, 0 (far) - 100 (close).
 *
 * Returns 0 on success, -EINVAL if camera is off or focus value is out of
 * bounds, or returned errors from either camaf_reg_write or camaf_reg_read
 * functions.
 **/
int ad5820_af_setfocus(u16 posn)
{
	struct ad5820_device *af_dev = &ad5820;
	struct i2c_client *client = af_dev->i2c_client;
	u16 cur_focus_value = 0;
	int ret = -EINVAL;
	
	if (posn > MAX_FOCUS_POS) {
		printk(KERN_ERR "Bad posn params 0x%x \n", posn);
		return ret;
	}

	if ((af_dev->power_state == V4L2_POWER_OFF) ||
		(af_dev->power_state == V4L2_POWER_STANDBY)) {
		af_dev->current_lens_posn = posn;
		return 0;
	}

	ret = camaf_reg_read(client, &cur_focus_value);

	if (ret) {
		printk(KERN_ERR "Read of current Lens position failed\n");
		return ret;
	}

	if (CAMAF_AD5820_DATA_R(cur_focus_value) == posn) {
		printk(KERN_DEBUG "Device already in requested focal point\n");
		return ret;
	}

	ret = camaf_reg_write(client,
				CAMAF_AD5820_POWERDN(CAMAF_AD5820_ENABLE) |
				CAMAF_AD5820_DATA(posn));

	if (ret)
		printk(KERN_ERR "Setfocus register write failed\n");
	ad5820.current_lens_posn = posn;
	position_global = ad5820.current_lens_posn;
	return ret;
}
EXPORT_SYMBOL(ad5820_af_setfocus);

static enum hrtimer_restart ad5820_timer_callback(struct hrtimer *timer)
{
    complete(&ad5820.mstep_completion);

    return HRTIMER_NORESTART;
}

static enum ad5820_drive_mode select_drive_mode(u16 new_pos, u16 cur_pos, u32 time)
{
	u16 offset = abs(new_pos - cur_pos);

	if (AD5820_TIME_50000 * offset > time) {
		return AD5820_DRIVE_MODE_DIRECT_0;
	}else if (AD5820_TIME_50000 * offset < time && AD5820_TIME_100000 * offset > time) {
		return AD5820_DRIVE_MODE_LINEAR_50000;
	} else if (AD5820_TIME_100000 * offset < time && AD5820_TIME_200000 * offset > time) {
		return AD5820_DRIVE_MODE_LINEAR_100000;
	} else if (AD5820_TIME_200000 * offset < time && AD5820_TIME_400000 * offset > time) {
		return AD5820_DRIVE_MODE_LINEAR_200000;
	} else if (AD5820_TIME_400000 * offset < time && AD5820_TIME_800000 * offset > time) {
		return AD5820_DRIVE_MODE_LINEAR_400000;
	} else if (AD5820_TIME_800000 * offset < time && AD5820_TIME_1600000 * offset > time) {
		return AD5820_DRIVE_MODE_LINEAR_800000;
	} else if (AD5820_TIME_1600000 * offset < time && AD5820_TIME_3200000 * offset > time) {
		return AD5820_DRIVE_MODE_LINEAR_1600000;
	} else {
		return AD5820_DRIVE_MODE_LINEAR_3200000;
	}
}

/**
 * ad5820_af_setscript - Sets the desired focus script.
 * @posn: Desired focus position, 0 (far) - 100 (close).
 *
 * Returns 0 on success, -EINVAL if camera is off or focus value is out of
 * bounds, or returned errors from either camaf_reg_write or camaf_reg_read
 * functions.
 **/
int ad5820_af_setscript(s32 script)
{
	int i, ret = -EINVAL;
	u16 cur_pos;
	u16 new_pos;
	struct timespec start_time;
	s32 time_offset;
	s32 time_correction = 0;
	s32 time_to_wait;
	enum ad5820_drive_mode drive_mode;

	struct ad5820_micro_steps_list __user *userp =
		(struct ad5820_micro_steps_list __user*) script;

	struct hrtimer timer;

	//printk(KERN_ERR "V4L2_CID_FOCUS_SCRIPT(%p)\n", userp);

	if( copy_from_user(&ad5820.mstep_list, userp, sizeof(*userp)) != 0 )
	{
		printk(KERN_ERR "V4L2_CID_FOCUS_SCRIPT - copy_from_user failed\n");
		return ret;
	}

	if ((ad5820.power_state == V4L2_POWER_OFF) ||
		(ad5820.power_state == V4L2_POWER_STANDBY)) {
		return 0;
	}

	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer.function = ad5820_timer_callback;

	cur_pos = ad5820.current_lens_posn;

	for (i = 0; i < ad5820.mstep_list.count; i++) {
		new_pos = ad5820.mstep_list.steps[i].lens_pos;

		if (new_pos > MAX_FOCUS_POS) {
			printk(KERN_ERR "Bad posn params 0x%x \n", new_pos);
			return ret;
		}

		time_to_wait = ad5820.mstep_list.steps[i].time_sleep_ns - time_correction;

		if (time_to_wait < 0) {
			time_to_wait = 0;
		}

		drive_mode = select_drive_mode(new_pos, cur_pos, time_to_wait);

		ktime_get_ts(&start_time);

		if (time_to_wait > 1000000) {
			init_completion(&ad5820.mstep_completion);
			hrtimer_start(&timer, ktime_set(0, time_to_wait), HRTIMER_MODE_REL);
		}

		ret = camaf_reg_write(ad5820.i2c_client,
					CAMAF_AD5820_POWERDN(CAMAF_AD5820_ENABLE) |
					CAMAF_AD5820_DATA(new_pos) |
					CAMAF_AD5820_MODE(drive_mode));

		if (ret) {
			hrtimer_cancel(&timer);
			return ret;
		}

		cur_pos = new_pos;

		if (time_to_wait > 1000000) {
			wait_for_completion(&ad5820.mstep_completion);
		} else {
			udelay(time_to_wait / 1000);
		}

		ktime_get_ts(&ad5820.mstep_list.steps[i].exec_time);

		time_offset = ad5820.mstep_list.steps[i].exec_time.tv_nsec - start_time.tv_nsec;
		if (time_offset < 0) {
			time_offset = 1000000000 - start_time.tv_nsec + ad5820.mstep_list.steps[i].exec_time.tv_nsec;
		}

		time_correction = time_offset - ad5820.mstep_list.steps[i].time_sleep_ns;

		if (time_correction < 0) {
			time_correction = 0;
		}
	}

	ad5820.current_lens_posn = cur_pos;

	if (copy_to_user(userp, &ad5820.mstep_list, sizeof(*userp)) != 0)
		printk(KERN_ERR "copy_to_user failed\n");

	return 0;
}
EXPORT_SYMBOL(ad5820_af_setscript);

/**
 * ad5820_af_getfocus - Gets the focus value from device.
 * @value: Pointer to u16 variable which will contain the focus value.
 *
 * Returns 0 if successful, -EINVAL if camera is off, or return value of
 * camaf_reg_read if fails.
 **/
int ad5820_af_getfocus(u16 *value)
{
	int ret = -EINVAL;
	u16 posn = 0;

	struct ad5820_device *af_dev = &ad5820;
	struct i2c_client *client = af_dev->i2c_client;

	if ((af_dev->power_state == V4L2_POWER_OFF) ||
	   (af_dev->power_state == V4L2_POWER_STANDBY))
		return ret;

	ret = camaf_reg_read(client, &posn);

	if (ret) {
		printk(KERN_ERR "Read of current Lens position failed\n");
		return ret;
	}
	*value = CAMAF_AD5820_DATA_R(posn);
	ad5820.current_lens_posn = CAMAF_AD5820_DATA_R(posn);
	return ret;
}
EXPORT_SYMBOL(ad5820_af_getfocus);

/**
 * ioctl_queryctrl - V4L2 lens interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
				struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;
	return 0;
}

/**
 * ioctl_g_ctrl - V4L2 AD5820 lens interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	struct vcontrol *lvc;
	int i;
	u16 curr_posn;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case  V4L2_CID_FOCUS_ABSOLUTE:
		if (ad5820_af_getfocus(&curr_posn))
			return -EFAULT;
		vc->value = curr_posn;
		lvc->current_value = curr_posn;
		break;
	}

	return 0;
}

/**
 * ioctl_s_ctrl - V4L2 AD5820 lens interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	int retval = -EINVAL;
	int i;
	struct vcontrol *lvc;


	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
		retval = ad5820_af_setfocus(vc->value);
		if (!retval)
			lvc->current_value = vc->value;
		break;

    case V4L2_CID_FOCUS_SCRIPT:
		retval = ad5820_af_setscript(vc->value);
		if (!retval)
			lvc->current_value = vc->value;
		break;
 	}

	return retval;
}

/**
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct ad5820_device *lens = s->priv;

	return lens->pdata->priv_data_set(p);

}

/**
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct ad5820_device *lens = s->priv;
	struct i2c_client *c = lens->i2c_client;
	int rval;

	rval = lens->pdata->power_set(on);

	if ((on == V4L2_POWER_ON) && (lens->state == LENS_NOT_DETECTED)) {
		rval = ad5820_detect(c);
		if (rval < 0) {
			dev_err(&c->dev, "Unable to detect "
				DRIVER_NAME " lens HW\n");
			printk(KERN_ERR "Unable to detect "
				DRIVER_NAME " lens HW\n");
			lens->state = LENS_NOT_DETECTED;
			return rval;
		}
		lens->state = LENS_DETECTED;
		pr_info(DRIVER_NAME " lens HW detected\n");
	}

	if ((lens->power_state == V4L2_POWER_STANDBY) && (on == V4L2_POWER_ON)
					&& (lens->state == LENS_DETECTED))
		ad5820_af_setfocus(position_global);

	lens->power_state = on;
	return 0;
}

static struct v4l2_int_ioctl_desc ad5820_ioctl_desc[] = {
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_priv },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_queryctrl },
	{ .num = vidioc_int_g_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_ctrl },
};

static struct v4l2_int_slave ad5820_slave = {
	.ioctls = ad5820_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ad5820_ioctl_desc),
};

static struct v4l2_int_device ad5820_int_device = {
	.module = THIS_MODULE,
	.name = DRIVER_NAME,
	.priv = &ad5820,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ad5820_slave,
	},
};

/**
 * ad5820_probe - Probes the driver for valid I2C attachment.
 * @client: Pointer to structure of I2C client.
 *
 * Returns 0 if successful, or -EBUSY if unable to get client attached data.
 **/
static int
ad5820_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ad5820_device *lens = &ad5820;
	int err;

	dev_info(&client->dev, "ad5820 probe called....\n");

	if (i2c_get_clientdata(client)) {
		printk(KERN_ERR " DTA BUSY %s\n", client->name);
		return -EBUSY;
	}

	lens->pdata = client->dev.platform_data;

	if (!lens->pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	lens->v4l2_int_device = &ad5820_int_device;

	lens->i2c_client = client;
	i2c_set_clientdata(client, lens);

	err = v4l2_int_device_register(lens->v4l2_int_device);
	if (err) {
		printk(KERN_ERR "Failed to Register "
			DRIVER_NAME " as V4L2 device.\n");
		i2c_set_clientdata(client, NULL);
	} else {
		printk(KERN_ERR "Registered "
			DRIVER_NAME " as V4L2 device.\n");
	}

	return 0;
}

/**
 * ad5820_remove - Routine when device its unregistered from I2C
 * @client: Pointer to structure of I2C client.
 *
 * Returns 0 if successful, or -ENODEV if the client isn't attached.
 **/
static int __exit ad5820_remove(struct i2c_client *client)
{
	struct ad5820_device *lens = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(lens->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

/**
 * ad5820_init - Module initialisation.
 *
 * Returns 0 if successful, or -EINVAL if device couldn't be initialized, or
 * added as a character device.
 **/
static int __init ad5820_init(void)
{
	int err = -EINVAL;

	err = i2c_add_driver(&ad5820_i2c_driver);
	if (err)
		goto fail;
	return err;
fail:
	printk(KERN_ERR "Failed to register " DRIVER_NAME ".\n");
	return err;
}
late_initcall(ad5820_init);

/**
 * ad5820_cleanup - Module cleanup.
 **/
static void __exit ad5820_cleanup(void)
{
	i2c_del_driver(&ad5820_i2c_driver);
}
module_exit(ad5820_cleanup);

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AD5820 LENS driver");
