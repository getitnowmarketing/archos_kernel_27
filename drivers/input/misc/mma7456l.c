/*
 *   MMA7456L Accelerometer driver
 *
 *   Copyright (c) by Jean-Christophe Rona <rona@archos.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/mma7456l.h>
#include <linux/earlysuspend.h>

#undef DEBUG_REPORT
#undef DEBUG_IOCTL
#undef DEBUG_IRQ
#undef DEBUG_REG
//#define DEBUG_REPORT
#define DEBUG_IOCTL
//#define DEBUG_IRQ
//#define DEBUG_REG

#define MMA7456L_VERSION		"0.1"
#define MMA7456L_DATE			"25 February 2009"

static inline int mma7456l_write(struct i2c_client *client, int reg, int value);
static inline int mma7456l_read(struct i2c_client *client, int reg);

struct mma7456l_data {
	struct input_dev *input_dev;
	struct delayed_work polling_work;
	struct work_struct irq_work;
	struct mma7456l_pdata *pdata;
	/* Delay in ms between two acceleration reports */
	unsigned long poll_delay;
	/* Do we read the 10 bits output (8g only, 64 LSB/g)
		or the 8 bits one (8g, 4g, 2g) ? */
	int output_length;
	int opencount;
	struct early_suspend early_suspend;
	unsigned standby:1;
};

/* I2C client handle */
struct i2c_client *this_client;


/*
 * MMA7456L stuff
 */

static int mma7456l_set_mode(short mode)
{
	int ret = 0;
	u8 data;
	
	data = mma7456l_read(this_client, REG_MCTL) & ~MMA7456L_MODE_MASK;
	ret = mma7456l_write(this_client, REG_MCTL, data |
				(mode & MMA7456L_MODE_MASK));
	return ret;
}

static short mma7456l_get_mode(void)
{
	u8 data;
	
	data = mma7456l_read(this_client, REG_MCTL) & MMA7456L_MODE_MASK;
	return data;
}

static int mma7456l_set_meas_range(short range)
{
	int ret = 0;
	u8 data;
	
	data = mma7456l_read(this_client, REG_MCTL) & ~MMA7456L_RANGE_MASK;
	ret = mma7456l_write(this_client, REG_MCTL, data |
				(range & MMA7456L_RANGE_MASK));
	return ret;
}

static short mma7456l_get_meas_range(void)
{
	u8 data;
	
	data = mma7456l_read(this_client, REG_MCTL) & MMA7456L_RANGE_MASK;
	return data;
}

static int mma7456l_set_pulse_duration(short time)
{
	int ret = 0;
	u8 data;
	
	data = mma7456l_read(this_client, REG_PW) & ~MMA7456L_PULSE_DURATION_MASK;
	ret = mma7456l_write(this_client, REG_PW, data |
				(time & MMA7456L_PULSE_DURATION_MASK));
	return ret;
}

static short mma7456l_get_pulse_duration(void)
{
	u8 data;
	
	data = mma7456l_read(this_client, REG_PW) & MMA7456L_PULSE_DURATION_MASK;
	return data;
}

static int mma7456l_set_pulse_threshold(short trshld)
{
	int ret = 0;
	u8 data;
	
	data = mma7456l_read(this_client, REG_PDTH) & ~MMA7456L_PULSE_THRESHOLD_MASK;
	ret = mma7456l_write(this_client, REG_PDTH, data |
				(trshld & MMA7456L_PULSE_THRESHOLD_MASK));
	return ret;
}

static short mma7456l_get_pulse_threshold(void)
{
	u8 data;
	
	data = mma7456l_read(this_client, REG_PDTH) & MMA7456L_PULSE_THRESHOLD_MASK;
	return data;
}

static int mma7456l_set_level_threshold(short trshld)
{
	int ret = 0;
	u8 data;
	
	data = mma7456l_read(this_client, REG_LDTH) & ~MMA7456L_LEVEL_THRESHOLD_MASK;
	ret = mma7456l_write(this_client, REG_LDTH, data |
				(trshld & MMA7456L_LEVEL_THRESHOLD_MASK));
	return ret;
}

static short mma7456l_get_level_threshold(void)
{
	u8 data;
	
	data = mma7456l_read(this_client, REG_LDTH) & MMA7456L_LEVEL_THRESHOLD_MASK;
	return data;
}

static int mma7456l_datardy_trigger_int1(int trigger)
{
	int ret = 0;
	u8 data;

	if (!trigger)
		data = mma7456l_read(this_client, REG_MCTL) | MMA7456L_DATA_RDY_INT1;
	else
		data = mma7456l_read(this_client, REG_MCTL) & ~MMA7456L_DATA_RDY_INT1;

	ret = mma7456l_write(this_client, REG_MCTL, data);
	return ret;
}

static void mma7456l_get_values(struct accel_data * values)
{
	struct mma7456l_data *data = i2c_get_clientdata(this_client);

	if (!values) {
		printk("mma7456l_get_values: values = NULL !!");
		return;
	}

	/* Wait for the data to be ready */
	while(!(mma7456l_read(this_client, REG_STATUS) & MMA7456L_STATUS_DRDY));

	/* x, y and z are 10/8 bits signed values */
	if (data->output_length) {
		values->x = (mma7456l_read(this_client, REG_XOUTL) & 0xFF) |
			((mma7456l_read(this_client, REG_XOUTH) & 0x03) << 8);
		if (values->x >= 512) values->x -= 1024;
		values->y = (mma7456l_read(this_client, REG_YOUTL) & 0xFF) |
			((mma7456l_read(this_client, REG_YOUTH) & 0x03) << 8);
		if (values->y >= 512) values->y -= 1024;
		values->z = (mma7456l_read(this_client, REG_ZOUTL) & 0xFF) |
			((mma7456l_read(this_client, REG_ZOUTH) & 0x03) << 8);
		if (values->z >= 512) values->z -= 1024;
	} else {
		values->x = (signed char) mma7456l_read(this_client, REG_XOUT8);
		values->y = (signed char) mma7456l_read(this_client, REG_YOUT8);
		values->z = (signed char) mma7456l_read(this_client, REG_ZOUT8);
	}
}

static void mma7456l_report_values(void)
{
	struct accel_data values;
	struct mma7456l_data *data = i2c_get_clientdata(this_client);

	mma7456l_get_values(&values);

#ifdef DEBUG_REPORT
	printk("mma7456l_report_values: X = %d, Y = %d, Z = %d\n", values.x, values.y, values.z);
#endif
	input_report_abs(data->input_dev, ABS_X, values.x);
	input_report_abs(data->input_dev, ABS_Y, values.y);
	input_report_abs(data->input_dev, ABS_Z, values.z);

	input_sync(data->input_dev);
	return;
}

static void mma7456l_general_purpose_event(int code)
{
	struct mma7456l_data *data = i2c_get_clientdata(this_client);

	input_report_abs(data->input_dev, ABS_TILT_X, code);
	return;
}

static int mma7456l_configure(struct mma7456l_data *data)
{
	int ret = 0;
	
	/* Set Standby mode, Measurement mode will
		be set when the device will be open */
	ret = mma7456l_set_mode(MMA7456L_MODE_STANDBY);
	if (ret) {
		printk(KERN_ERR "mma7456l_configure: mma7456l_set_mode failed\n");
		return ret;
	}
	data->standby = 1;

	/* Set 2G range */
	ret = mma7456l_set_meas_range(MMA7456L_RANGE_2G);
	if (ret) {
		printk(KERN_ERR
			"mma7456l_configure: mma7456l_set_meas_range failed\n");
		return ret;
	}

	/* We don't want data ready to trigger INT1 ! */
	mma7456l_datardy_trigger_int1(0);

	/* Set pulse threshold to 100 */
	ret = mma7456l_set_pulse_threshold(0x64);
	if (ret) {
		printk(KERN_ERR
			"mma7456l_configure: mma7456l_set_pulse_threshold failed\n");
		return ret;
	}

	/* Set max pulse duration to 5 ms */
	ret = mma7456l_set_pulse_duration(0x0A);
	if (ret) {
		printk(KERN_ERR
			"mma7456l_configure: mma7456l_set_pulse_duration failed\n");
		return ret;
	}

	/* Set level threshold to the max (prevents from receiving INT 1) */
	ret = mma7456l_set_level_threshold(0x7f);
	if (ret) {
		printk(KERN_ERR
			"mma7456l_configure: mma7456l_set_level_threshold failed\n");
		return ret;
	}

	return 0;
}

static int mma7456l_open(struct input_dev *dev)
{
	int ret = 0;
	struct mma7456l_data *data = i2c_get_clientdata(this_client);

	data->opencount++;
	if (data->opencount > 1)
		return 0;
	
	/* Set Measurement mode */
	ret = mma7456l_set_mode(MMA7456L_MODE_MEASURE);
	if (ret) {
		printk(KERN_ERR
			"mma7456l_open: mma7456l_set_mode failed\n");
		return ret;
	}
	data->standby = 0;

	/* Everything is OK, enabling polling (if needed) */
	if (data->poll_delay > 0)
		schedule_delayed_work(&data->polling_work,0);
	
	return ret;
}

static void mma7456l_close(struct input_dev *dev)
{
	struct mma7456l_data *data = i2c_get_clientdata(this_client);

	data->opencount--;
	if (data->opencount > 0)
		return;
	
	/* Cancel remaining work */
	cancel_delayed_work_sync(&data->polling_work);
	/* Put the chip in Standby mode */
	mma7456l_set_mode(MMA7456L_MODE_STANDBY);
	data->standby = 1;
	return;
}

static void mma7456l_polling_work_func(struct work_struct *work)
{
	struct mma7456l_data *data = i2c_get_clientdata(this_client);

	/* Schedule the next work in data->poll_delay ms */
	schedule_delayed_work(&data->polling_work, (HZ * data->poll_delay) / 1000);

	mma7456l_report_values();
}

static void mma7456l_irq_work_func(struct work_struct *work)
{
	/* Clear INTs */
	mma7456l_write(this_client, REG_INTRST, 0x03);
	mma7456l_write(this_client, REG_INTRST, 0x00);
}

static irqreturn_t mma7456l_isr(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct mma7456l_data *data = i2c_get_clientdata(this_client);

#ifdef DEBUG_IRQ
	printk("mma7456l_isr: got irq %d\n", irq);
#endif

	if (irq == data->pdata->irq1) {
		/* TODO: Report level event */
	} else if (irq == data->pdata->irq2) {
		input_report_abs(data->input_dev, ABS_MISC, 1);
		input_sync(data->input_dev);
		input_report_abs(data->input_dev, ABS_MISC, 0);
		input_sync(data->input_dev);
	}

	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}

static void mma7456_early_suspend(struct early_suspend *h)
{
	struct mma7456l_data *data = 
		container_of(h, struct mma7456l_data, early_suspend);
	
	if (data->standby)
		return;
	
	if (data->poll_delay > 0)
		cancel_delayed_work(&data->polling_work);
	mma7456l_set_mode(MMA7456L_MODE_STANDBY);
	data->standby = 1;
}

static void mma7456_early_resume(struct early_suspend *h)
{
	struct mma7456l_data *data = 
		container_of(h, struct mma7456l_data, early_suspend);

	if (!data->standby)
		return;
	
	mma7456l_set_mode(MMA7456L_MODE_MEASURE);
	data->standby = 0;
	if (data->poll_delay > 0)
		schedule_delayed_work(&data->polling_work,0);
}

/*
 * Control device stuff
 */

static int mma7456l_ctrl_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int mma7456l_ctrl_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int
mma7456l_ctrl_ioctl(struct inode *inode, struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	struct mma7456l_data *data = i2c_get_clientdata(this_client);
	void __user *argp = (void __user *)arg;
	short short_data;
	struct accel_data accel_values;
	int int_data;

	switch (cmd) {
	case MMA7456L_IOCTL_S_MODE:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
		switch(short_data) {
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Set mode %d\n", short_data);
#endif
		case MMA7456L_MODE_STANDBY:
			/* Disable polling first, then the chip
				can be put in standby mode */
			cancel_delayed_work_sync(&data->polling_work);
			mma7456l_set_mode(MMA7456L_MODE_STANDBY);
			data->standby = 1;
			break;
		case MMA7456L_MODE_MEASURE:
			mma7456l_set_mode(MMA7456L_MODE_MEASURE);
			data->standby = 0;
			/* Enabling polling if needed */
			if (data->poll_delay > 0)
				schedule_delayed_work(&data->polling_work,0);
			break;
		case MMA7456L_MODE_LEVELDET:
			cancel_delayed_work_sync(&data->polling_work);
			mma7456l_set_mode(MMA7456L_MODE_LEVELDET);
			data->standby = 0;
			break;
		case MMA7456L_MODE_PULSEDET:
			cancel_delayed_work_sync(&data->polling_work);
			mma7456l_set_mode(MMA7456L_MODE_PULSEDET);
			data->standby = 0;
			break;
		default:
			return -EINVAL;
		}
		break;
	case MMA7456L_IOCTL_G_MODE:
		short_data = mma7456l_get_mode();
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Get mode %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA7456L_IOCTL_S_RANGE:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Set range %d\n", short_data);
#endif
		switch(short_data) {
		case MMA7456L_RANGE_2G:
			mma7456l_set_meas_range(MMA7456L_RANGE_2G);
			break;
		case MMA7456L_RANGE_4G:
			mma7456l_set_meas_range(MMA7456L_RANGE_4G);
			break;
		case MMA7456L_RANGE_8G:
			mma7456l_set_meas_range(MMA7456L_RANGE_8G);
			break;
		default:
			return -EINVAL;
		}
		break;
	case MMA7456L_IOCTL_G_RANGE:
		short_data = mma7456l_get_meas_range();
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Get range %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA7456L_IOCTL_S_POLL_DELAY:
		if (copy_from_user(&int_data, argp, sizeof(int_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Set polling delay %d\n", int_data);
#endif
		if (int_data < 0) {
			return -EINVAL;
		} else if (int_data == 0) {
			/* Disabling polling */
			data->poll_delay = 0;
			cancel_delayed_work_sync(&data->polling_work);
		} else if (data->poll_delay == 0) {
			/* Set the polling delay and enable polling
				if we are in measurement mode*/
			data->poll_delay = int_data;
			if (mma7456l_get_mode() == MMA7456L_MODE_MEASURE) 
				schedule_delayed_work(&data->polling_work,0);
		} else {
			/* Set the polling delay */
			data->poll_delay = int_data;
		}
		break;
	case MMA7456L_IOCTL_G_POLL_DELAY:
		int_data = (int) data->poll_delay;
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Get polling delay %d\n", int_data);
#endif
		if (copy_to_user(argp, &int_data, sizeof(int_data)))
			return -EFAULT;
		break;
	case MMA7456L_IOCTL_S_OUTPUT_LENGTH:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Set output length %d\n", short_data);
#endif
		switch(short_data) {
		case MMA7456L_OUTPUT_8BITS:
			data->output_length = MMA7456L_OUTPUT_8BITS;
			break;
		case MMA7456L_OUTPUT_10BITS:
			data->output_length = MMA7456L_OUTPUT_10BITS;
			break;
		default:
			return -EINVAL;
		}
		break;
	case MMA7456L_IOCTL_G_OUTPUT_LENGTH:
		short_data = (short) data->output_length;
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Get output length %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA7456L_IOCTL_S_PULSE_DURATION:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Set pulse duration %d\n", short_data);
#endif
		if (short_data < 0 || short_data > 0xFF)
			return -EINVAL;
		mma7456l_set_pulse_duration(short_data);
		break;
	case MMA7456L_IOCTL_G_PULSE_DURATION:
		short_data = mma7456l_get_pulse_duration();
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Get pulse duration %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA7456L_IOCTL_S_PULSE_THRSHOLD:
		if (copy_from_user(&short_data, argp, sizeof(short_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Set pulse threshold %d\n", short_data);
#endif
		if (short_data < 0 || short_data > 0x7F)
			return -EINVAL;
		mma7456l_set_pulse_threshold(short_data);
		break;
	case MMA7456L_IOCTL_G_PULSE_THRSHOLD:
		short_data = mma7456l_get_pulse_threshold();
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Get pulse threshold %d\n", short_data);
#endif
		if (copy_to_user(argp, &short_data, sizeof(short_data)))
			return -EFAULT;
		break;
	case MMA7456L_IOCTL_GP_EVENT:
		if (copy_from_user(&int_data, argp, sizeof(int_data)))
			return -EFAULT;
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: General purpose event %d\n", int_data);
#endif
		if (int_data < 0 || int_data > 65535)
			return -EINVAL;
		mma7456l_general_purpose_event(int_data);
		break;
	case MMA7456L_IOCTL_G_ACCEL_DATA:
		mma7456l_get_values(&accel_values);
#ifdef DEBUG_IOCTL
		printk("mma7456l_ctrl_ioctl: Get accel values x=%d, y=%d, z=%d\n", accel_values.x, accel_values.y, accel_values.z);
#endif
		if (copy_to_user(argp, &accel_values, sizeof(struct accel_data)))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct file_operations mma7456l_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = mma7456l_ctrl_open,
	.release = mma7456l_ctrl_release,
	.ioctl = mma7456l_ctrl_ioctl,
};

static struct miscdevice mma7456l_ctrl_device = {
	.minor = ACCEL_CTRL_MINOR, //MISC_DYNAMIC_MINOR,
	.name = "accel_ctrl",
	.fops = &mma7456l_ctrl_fops,
};


/*
 * I2C Stuff
 */

static inline int mma7456l_write(struct i2c_client *client, int reg, int value)
{
#ifdef DEBUG_REG
	int ret;
	ret = i2c_smbus_write_byte_data(client, reg, value);
	printk("mma7456l_write: Reg = 0x%02X, Value = 0x%02X, Ret = %d\n", reg, value, ret);
	return ret;
#else
	return i2c_smbus_write_byte_data(client, reg, value);
#endif
}

static inline int mma7456l_read(struct i2c_client *client, int reg)
{
#ifdef DEBUG_REG
	int value;
	value = i2c_smbus_read_byte_data(client, reg);;
	printk("mma7456l_read: Reg = 0x%02X, Value = 0x%02X\n", reg, value);
	return value;
#else
	return i2c_smbus_read_byte_data(client, reg);
#endif
}

static int mma7456l_probe(struct i2c_client *client, 
		const struct i2c_device_id *id)
{
	int err;
	struct mma7456l_data *i2c_data;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s functinality check failed\n", id->name);
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	printk(KERN_INFO "MMA7456L : Chip ID = 0x%02X, User Info = 0x%02X\n",
		mma7456l_read(client, REG_WHOAMI),
		mma7456l_read(client, REG_USRINF));

	i2c_data = kzalloc(sizeof(struct mma7456l_data), GFP_KERNEL);
	if (!i2c_data) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_data->pdata = (struct mma7456l_pdata*) client->dev.platform_data;
	if (!i2c_data->pdata) {
		printk(KERN_ERR "mma7456l_probe: No platform data !!\n");
		err = -ENODEV;
		goto exit_plat_data_failed;
	}

	err = request_irq(i2c_data->pdata->irq1, mma7456l_isr, IRQF_TRIGGER_RISING,
			    "mma7456l-int", client);
	if(err) {
		printk(KERN_ERR "mma7456l_probe: Irq request failed (irq %d)!!\n", i2c_data->pdata->irq1);
		goto exit_req_irq1_failed;
	}

	err = request_irq(i2c_data->pdata->irq2, mma7456l_isr, IRQF_TRIGGER_RISING,
			    "mma7456l-int", client);
	if(err) {
		printk(KERN_ERR "mma7456l_probe: Irq request failed (irq %d)!!\n", i2c_data->pdata->irq2);
		goto exit_req_irq2_failed;
	}

	INIT_DELAYED_WORK(&i2c_data->polling_work, mma7456l_polling_work_func);
	INIT_WORK(&i2c_data->irq_work, mma7456l_irq_work_func);
	i2c_set_clientdata(client, i2c_data);
	this_client = client;

	i2c_data->input_dev = input_allocate_device();
	/* Set the polling delay to 0 ms, disabling polling */
	i2c_data->poll_delay = 0;
	/* We will read the 10 bits output */
	i2c_data->output_length = MMA7456L_OUTPUT_10BITS;
	/* initialize reference counting */
	i2c_data->opencount = 0;
	
	if (!i2c_data->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "mma7456l_probe: Failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	set_bit(EV_ABS, i2c_data->input_dev->evbit);
	/* x-axis acceleration */
	input_set_abs_params(i2c_data->input_dev, ABS_X, -512, 512, 4, 0);
	/* y-axis acceleration */
	input_set_abs_params(i2c_data->input_dev, ABS_Y, -512, 512, 4, 0);
	/* z-axis acceleration */
	input_set_abs_params(i2c_data->input_dev, ABS_Z, -512, 512, 4, 0);
	/* pulse event */
	input_set_abs_params(i2c_data->input_dev, ABS_MISC, 0, 1, 0, 0);
	/* GP event */
	input_set_abs_params(i2c_data->input_dev, ABS_TILT_X, 0, 65535, 0, 0);

	i2c_data->input_dev->name = "MMA7456L Accelerometer";
	i2c_data->input_dev->open = mma7456l_open;
	i2c_data->input_dev->close = mma7456l_close;

	err = misc_register(&mma7456l_ctrl_device);
	if (err) {
		printk(KERN_ERR
		       "mma7456l_probe: Unable to register control misc device\n");
		goto exit_misc_device_register_failed;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	/* register early suspend handler */
	i2c_data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	i2c_data->early_suspend.suspend = mma7456_early_suspend;
	i2c_data->early_suspend.resume = mma7456_early_resume;
	register_early_suspend(&i2c_data->early_suspend);
#endif
	/* Configure the chip */
	err = mma7456l_configure(i2c_data);
	if (err) {
		printk(KERN_ERR
		       "mma7456l_probe: mma7456l initialization failed\n");
		goto exit_init_failed;
	}
	
	err = input_register_device(i2c_data->input_dev);
	if (err) {
		printk(KERN_ERR
		       "mma7456l_probe: Unable to register input device: %s\n",
		       i2c_data->input_dev->name);
		goto exit_input_register_device_failed;
	}

	return 0;

exit_input_register_device_failed:
exit_init_failed:
	unregister_early_suspend(&i2c_data->early_suspend);
	misc_deregister(&mma7456l_ctrl_device);
exit_misc_device_register_failed:
	input_free_device(i2c_data->input_dev);
exit_input_dev_alloc_failed:
	free_irq(i2c_data->pdata->irq2, client);
exit_req_irq2_failed:
	free_irq(i2c_data->pdata->irq1, client);
exit_req_irq1_failed:
exit_plat_data_failed:
	kfree(i2c_data);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}
	
static int __exit mma7456l_remove(struct i2c_client *client)
{
	struct mma7456l_data *data = i2c_get_clientdata(client);
	misc_deregister(&mma7456l_ctrl_device);
	input_unregister_device(data->input_dev);
	unregister_early_suspend(&data->early_suspend);
	free_irq(data->pdata->irq2, client);
	free_irq(data->pdata->irq1, client);
	i2c_detach_client(client);
	kfree(data);
	return 0;
}

static int mma7456l_suspend(struct i2c_client *client, pm_message_t mesg)
{
//	struct mma7456l_data *data = i2c_get_clientdata(client);
//	if (data->opencount) {
//		cancel_delayed_work_sync(&data->polling_work);
//		mma7456l_set_mode(MMA7456L_MODE_STANDBY);
//	}
	return 0;
}

static int mma7456l_resume(struct i2c_client *client)
{
//	struct mma7456l_data *data = i2c_get_clientdata(client);
//	if (data->opencount) {
//		mma7456l_set_mode(MMA7456L_MODE_MEASURE);
//		if (data->poll_delay > 0)
//			schedule_delayed_work(&data->polling_work,0);
//	}
	return 0;
}


static const struct i2c_device_id mma7456l_id[] = {
	{"mma7456l", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mma7456l_id);

static struct i2c_driver mma7456l_driver = {

	.driver = {
		.owner	= THIS_MODULE,
		.name	= "mma7456l",
	},
	.probe		= mma7456l_probe,
	.suspend	= mma7456l_suspend,
	.resume		= mma7456l_resume,
	.remove		= __exit_p(mma7456l_remove),
	.id_table	= mma7456l_id,
};

static int __init mma7456l_init(void)
{
	int res;
	
	if ((res = i2c_add_driver(&mma7456l_driver))) {
		printk("mma7456l: Driver registration failed, module not inserted.\n");
		return res;
	}

	printk("MMA7456L driver version %s (%s)\n", MMA7456L_VERSION, MMA7456L_DATE);
	
	return 0;
}

static void __exit mma7456l_exit(void)
{
	i2c_del_driver(&mma7456l_driver);
}

MODULE_AUTHOR("Jean-Christophe Rona <rona@archos.com>");
MODULE_DESCRIPTION("Input device driver for MMA7456L accelerometer");
MODULE_LICENSE("GPL");

module_init(mma7456l_init)
module_exit(mma7456l_exit)
