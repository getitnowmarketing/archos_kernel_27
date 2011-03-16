/*
 * atmega-rtc.c
 *
 * Copyright (c) 2006 Archos
 * Author: Xavier Leclercq
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
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/workqueue.h>

#include <asm/uaccess.h>

#include <mach/atmega.h>
#include <mach/atmega-io.h>


#define ATMEGA_RTC_VERSION	"1.0"

struct atmega_rtc {
	struct rtc_device	*rtcdev;
	struct mutex 		rtc_mutex;
	int 			irq;
	struct work_struct	work;
	struct platform_device *pdev;
};

static long user_time_diff_stored = 0;

long atmega_rtc_get_diff(void)
{
	return user_time_diff_stored;
}

void atmega_rtc_set_diff(long newDiff)
{	
	user_time_diff_stored = newDiff;
}

static int atmega_rtc_gettime(struct device *dev, struct rtc_time *rtc_tm)
{
	dev_info(dev, "gettime\n");
	return atmega_io_get_clockval(rtc_tm, USER_TIME);
}

static int atmega_rtc_settime(struct device *dev, struct rtc_time *rtc_tm)
{
	dev_info(dev, "settime\n");
	return atmega_io_set_clockval(rtc_tm, USER_TIME);
}

static int atmega_rtc_getalarm(struct device *dev, struct rtc_wkalrm *rtc_alr)
{
	struct rtc_time tm;
	unsigned long almtime;
	unsigned short status;
	int ret;
	
	ret = atmega_io_get_clockval(&tm, ALARM_TIME);
	if (ret < 0)
		return ret;
	
	if ( atmega_read_value( ATMEGA_I2C_STATUS_REG,
			(unsigned char*)&status, ATMEGA_I2C_STATUS_SIZE) < 0)
		return -EIO;
	
	rtc_tm_to_time(&tm, &almtime);
	if ( almtime ) {
		rtc_alr->enabled = 1;
		rtc_alr->time = tm;
	} else {
		rtc_alr->enabled = 0;
	}
	
	if (status & 0x40)
		rtc_alr->pending = 1;
	else 
		rtc_alr->pending = 0;
	
	return 0;
}

static int atmega_rtc_setalarm(struct device *dev, struct rtc_wkalrm *rtc_alr)
{
	struct rtc_time *alr_tm = &rtc_alr->time;
	int ret = 0;
	
	if ( rtc_alr->enabled ) {
		dev_dbg(dev, "atmega_rtc_setalarm: SET\n");
		
		ret = atmega_io_set_clockval(alr_tm, ALARM_TIME);
	} else {
		struct atmega_exchange_table table = { 0,0,0,0,0 };

		dev_dbg(dev, "atmega_rtc_setalarm: CLEAR\n");

		table.control_byte = ATMEGA_I2C_CTRL_CMD_RESET_ALARM;
		ret = atmega_write_table( &table );
	}
	
	return ret;
}

static int atmega_rtc_open(struct device *dev)
{
	struct atmega_rtc *rtc = dev_get_drvdata(dev);
	if (mutex_lock_interruptible(&rtc->rtc_mutex))
		return -EINTR;
	
	return 0;
}

static void atmega_rtc_release(struct device *dev)
{
	struct atmega_rtc *rtc = dev_get_drvdata(dev);
	mutex_unlock(&rtc->rtc_mutex);
}

static irqreturn_t atmega_rtc_statusirq(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct atmega_rtc *rtc = platform_get_drvdata(pdev);
	
	schedule_work(&rtc->work);
	return IRQ_HANDLED;
}

static void atmega_rtc_worker(struct work_struct *work)
{
	struct atmega_rtc *rtc = container_of(work, struct atmega_rtc, work);
	struct rtc_task *rtc_task = rtc->rtcdev->irq_task;
	struct platform_device *pdev = rtc->pdev;
	unsigned short status;
	struct atmega_exchange_table alarm_reset = { 
		0, ATMEGA_I2C_CTRL_CMD_RESET_ALARM, 0, 0, 0 
	};

	/* must read status to see if it's alarm interrupt */
	if (atmega_read_value( ATMEGA_I2C_STATUS_REG, 
			(unsigned char*)&status, ATMEGA_I2C_STATUS_SIZE ) < 0) {
		dev_err(&pdev->dev, "cannot read atmega status\n");
		return;
	}
	
	/* only interested in alarm interrupts */
	if ( (status & 0x40) == 0)
		return;
	
	if (atmega_write_table(&alarm_reset) < 0)
		dev_err(&pdev->dev, "cannot clear alarm status\n");
	
	dev_dbg(&pdev->dev, "alarm irq\n");

	if (rtc_task)
		rtc_task->func(rtc_task->private_data);
}

static const struct rtc_class_ops atmega_rtc_ops = {
	.open		= atmega_rtc_open,
	.release	= atmega_rtc_release,
	.read_time	= atmega_rtc_gettime,
	.set_time	= atmega_rtc_settime,
	.read_alarm	= atmega_rtc_getalarm,
	.set_alarm	= atmega_rtc_setalarm,
};

static int __devexit atmega_rtc_remove( struct platform_device *pdev )
{
	struct atmega_rtc *rtc = platform_get_drvdata(pdev);
	rtc_device_unregister(rtc->rtcdev);
	mutex_destroy(&rtc->rtc_mutex);
	free_irq(rtc->irq, pdev);
	platform_set_drvdata(pdev, NULL);
	kfree(rtc);
	return 0;
}

static int __devinit atmega_rtc_probe( struct platform_device  *pdev )
{
	int ret;
	struct atmega_rtc *rtc;
	
	rtc = kzalloc(sizeof(struct atmega_rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	mutex_init(&rtc->rtc_mutex);
	INIT_WORK(&rtc->work, atmega_rtc_worker);

	rtc->irq = platform_get_irq(pdev, 0);
	if (rtc->irq <= 0) {
		ret = -ENOENT;
		goto fail;
	}
	
	if ( request_irq( rtc->irq, atmega_rtc_statusirq, 
			IRQF_SHARED|IRQF_TRIGGER_RISING, "atmega-rtc", pdev) ) {
		dev_err(&pdev->dev, "IRQ%d already in use\n", rtc->irq );
		ret = -EBUSY;
		goto fail;
	}

	rtc->rtcdev = rtc_device_register(pdev->name, &pdev->dev,
				&atmega_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc->rtcdev)) {
		ret = PTR_ERR(rtc->rtcdev);
		goto fail2;
	}

	rtc->pdev = pdev;
	platform_set_drvdata(pdev, rtc);
	return 0;
	
fail2:
	free_irq(rtc->irq, pdev);
fail:
	kfree(rtc);
	return ret;
}

static struct platform_driver atmega_rtc_driver = {
	.probe		= atmega_rtc_probe,
	.remove		= __devexit_p(atmega_rtc_remove),
	.driver		= {
		.name	= "atmega-rtc",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device *pdev;

/* ++++++++++++++++++ Module Init +++++++++++++++++++++++*/

static int __init atmega_rtc_init( void )
{
	printk( "ATMEGA RTC Driver v%s for G7X\n", ATMEGA_RTC_VERSION );

	return platform_driver_register( &atmega_rtc_driver );
}

static void __exit atmega_rtc_exit( void )
{
	platform_driver_unregister( &atmega_rtc_driver );
	platform_device_unregister( pdev );
}

module_init( atmega_rtc_init );
module_exit( atmega_rtc_exit );

/* Module information */
MODULE_AUTHOR("Archos S.A.");
MODULE_DESCRIPTION("ATMEGA RTC Driver for G7X");
MODULE_LICENSE("GPL");

