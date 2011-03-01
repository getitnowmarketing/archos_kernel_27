/*
 *  omap2_irblast.c 
 *
 * Copyright 2009 Archos
 * Author: Paul EBEYAN
 *         Jean-Christophe RONA
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
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/uaccess.h>        /* get_user,copy_to_user */
#include <asm/setup.h>
#include <mach/clock.h>
#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <mach/dmtimer.h>
#include <mach/irblaster.h>
#include <mach/board-archos.h>
#include <mach/archos-accessory.h>


#include "archos_irblaster.h"

#define IRBLAST_NAME	"irblast"

#define CONSTANT_CARRIER 0x100	/* from vg.h in the irblaster library */
#define NUM_OF_REPEATS	3

#define IRBLAST_TIMEOUT (2000) // 2 seconds timeout
#define IRBLAST_CHECK_TIME (4) // check every 4 milliseconds

#define IRB_DEV_NAME	"archos-irblaster"

//TODO : If needed, we can set a GPIO for N8 to test the IR stuff
//#define GIO_TX_PIN AD25_3430_GPIO147
//#define GIO_TX	147

typedef struct irblast_t {
	int numRepeats;
} irblast_t; 

static irblast_t irblast;
static struct omap_pwm_timer *gpt_pwm_list;

//#define DEBUG_ON_GIO_166

#define OMAP_TIMER_OCP_CFG_REG		0x10
#define OMAP_TIMER_OCP_WAKEUP_LINE	(1<<2)
#define OMAP_TIMER_OCP_NO_IDLE		(1<<3)
#define OMAP_TIMER_OCP_SMART_IDLE	(2<<3)
#define OMAP_TIMER_OCP_FREE_RUNNING	(1<<5)

#define OMAP_TIMER_OCP_CFG_REG_DEFAULT \
	(OMAP_TIMER_OCP_WAKEUP_LINE | \
		OMAP_TIMER_OCP_NO_IDLE | \
		OMAP_TIMER_OCP_FREE_RUNNING)

#define OMAP_TIMER_STAT_REG		0x18
#define OMAP_TIMER_CTRL_REG		0x24
#define OMAP_TIMER_CTRL_PT		(1 << 12)
#define OMAP_TIMER_CTRL_AR		(1 << 1)	/* auto-reload enable */
#define OMAP_TIMER_CTRL_ST		(1 << 0)	/* start timer */
#define OMAP_TIMER_LOAD_REG		0x2c
#define OMAP_TIMER_IF_CTRL_REG		0x40


#define IRBLASTER_TIMER_CTRL_IT_TYPE (OMAP_TIMER_INT_MATCH)

/* register offsets */
#define OMAP_TIMER_COUNTER_REG                 0x28
#define OMAP_TIMER_WRITE_PEND_REG              0x34

#if defined (DEBUG_ON_GIO_166)
	#define GIO_TX	166	/* gpio 166 */
	#define GIO_TX_PIN	V21_3430_GPIO166

	#define _gio_debug_set() omap_set_gpio_dataout(GIO_TX, 1)
	#define _gio_debug_clear() omap_set_gpio_dataout(GIO_TX, 0)
	#define _gio_debug_init() {\
		omap_request_gpio(GIO_TX);\
		if ( omap_cfg_reg(GIO_TX_PIN) < 0) {\
			printk(KERN_INFO "cannot request uart3 pin\n");\
		}\
		omap_set_gpio_direction(GIO_TX, GPIO_DIR_OUTPUT);\
		omap_set_gpio_dataout(GIO_TX, 1);\
	}

#else
	#define _gio_debug_set() do {} while(0)
	#define _gio_debug_clear() do {} while(0)
	#define _gio_debug_init() do {} while(0)
#endif

static u32 irq_irblaster_pwm_ctrl_reg;
unsigned short *irq_irblaster_cycles;

static wait_queue_head_t irb_wait;
static int irb_over = 0;

static irqreturn_t (*_callback_irblaster_timer_interrupt)(void);

static inline int _check_id(int id)
{
	if (id >= PWM_NUMBER) {
		printk(KERN_DEBUG "invalid pwm id %i\n", id);
		return 0;
	}
	return 1;
}

static inline int _check_timer(int id)
{
	if ( !gpt_pwm_list[id].timer) {
		printk(KERN_DEBUG "no gpt associated with: %s\n", gpt_pwm_list[id].name);
		return 0;
	}

	return 1;
}

void pwm_gpt_dump(int id)
{
	int i;

	if ( !_check_id(id) ) {
		return;
	}

	if ( !_check_timer(id) ) {
		return;
	}

	for (i=0;i<(17*4);i+=4)
		printk("0x%02x: 0x%08x\n", i, omap_dm_timer_read_reg(gpt_pwm_list[id].timer,i));
}

void pwm_gpt_start(int id)
{
	if ( !_check_id(id) ) {
		return;
	}

	if ( !_check_timer(id) ) {
		return;
	}

	/* enable the timer, start the clock */
	omap_dm_timer_enable(gpt_pwm_list[id].timer);

/*	do not wait for ack, just set up the reg and leave */
/*	this is better if you setup 2 correlated timers */
	omap_dm_timer_start(gpt_pwm_list[id].timer);
}


void pwm_gpt_stop(int id)
{
	if ( !_check_id(id) ) {
		return;
	}

	if ( !_check_timer(id) ) {
		return;
	}

	omap_dm_timer_stop(gpt_pwm_list[id].timer);
	omap_dm_timer_disable(gpt_pwm_list[id].timer);
}

static inline void gpt_set_new_cycle (struct omap_dm_timer *timer, 
		const unsigned int period, const int nb_cycle) 
{
	u32 val;

	val = 0xFFFFFFFF+1-(period*nb_cycle);
 	omap_dm_timer_write_reg(timer, OMAP_TIMER_LOAD_REG, val);
}

static void _select_irb_config(void)
{
#ifdef CONFIG_BOTH_UART_IR
	archos_accessory_set_irmode("both");
#else
	/* Disabling UART-3, enabling IR */
	archos_accessory_set_irmode("ir");
#endif
}

static void _select_default_config(void)
{
	/* Default mode */
	archos_accessory_set_irmode("default");
}


static irqreturn_t omap2_irblaster_timer_interrupt_no_carrier(void)
{
	int next_cycle;
	struct omap_dm_timer *timer_ctrl = gpt_pwm_list[ IRBLASTER_TIMER_CTRL].timer;
	struct omap_dm_timer *timer_pwm = gpt_pwm_list[ IRBLASTER_PWM ].timer;
	unsigned int timer_ctrl_period = gpt_pwm_list[ IRBLASTER_TIMER_CTRL].period;
	unsigned int timer_pwm_period = gpt_pwm_list[ IRBLASTER_PWM].period;

	omap_dm_timer_write_status(timer_ctrl, OMAP_TIMER_INT_MATCH);
	_gio_debug_clear();
	next_cycle = *irq_irblaster_cycles;
	if ( next_cycle ) {
		irq_irblaster_cycles++;
		gpt_set_new_cycle(timer_ctrl,timer_ctrl_period, next_cycle);
		gpt_set_new_cycle(timer_pwm,timer_pwm_period, next_cycle);
	}
	else {
		/* no trigger */
		omap_dm_timer_set_pwm(timer_pwm,0,1,0);
		/* stop timers */
		omap_dm_timer_stop(timer_pwm);
		omap_dm_timer_stop(timer_ctrl);
		irb_over = 1;
		wake_up_interruptible(&irb_wait);
	}

	_gio_debug_set();
	return IRQ_HANDLED;

}

static irqreturn_t omap2_irblaster_timer_interrupt_carrier(void)
{
	static int toggle=0;
	static int next_cycle=0;
	struct omap_dm_timer *timer_ctrl = gpt_pwm_list[ IRBLASTER_TIMER_CTRL].timer;
	struct omap_dm_timer *timer_pwm = gpt_pwm_list[ IRBLASTER_PWM ].timer;
	unsigned int timer_ctrl_period = gpt_pwm_list[ IRBLASTER_TIMER_CTRL].period;
	const unsigned int counter_lim = 0xFFFFFFFF+1-(timer_ctrl_period/2)+1;

	_gio_debug_clear();

	omap_dm_timer_write_status(timer_ctrl, OMAP_TIMER_INT_MATCH);
	next_cycle = *irq_irblaster_cycles;

	if ( next_cycle ) {
		irq_irblaster_cycles++;
		gpt_set_new_cycle(timer_ctrl,timer_ctrl_period, next_cycle);
	}

	if (toggle) {
		/* no trigger */
		irq_irblaster_pwm_ctrl_reg &= ~(3<<10);
	} else {
		/* trigger on overflow and match */
		/* output carrier */
		irq_irblaster_pwm_ctrl_reg |= (2<<10);
	}

	toggle = (toggle+1)&0x1;

	if ( !next_cycle ) {
		/* stop timers */
		omap_dm_timer_stop(timer_ctrl);
		omap_dm_timer_stop(timer_pwm);
		/* reset toogle */
		toggle=0;
		next_cycle=0;
	
		irb_over = 1;
		wake_up_interruptible(&irb_wait);
		goto intr_end;
	}

	/* wait for overflow */
	/* FIXME: try to find something that leaves cpu some ressources*/
	while ( omap_dm_timer_read_counter(timer_ctrl) < counter_lim)
		cpu_relax();

	/* active/desactive pwm */
	omap_dm_timer_write_reg(timer_pwm, OMAP_TIMER_CTRL_REG, irq_irblaster_pwm_ctrl_reg);

 intr_end:
	_gio_debug_set();


	return IRQ_HANDLED;
}


static irqreturn_t omap2_irblaster_timer_interrupt(int irq, void *dev_id) 
{
	return (*_callback_irblaster_timer_interrupt)();
}

#define BEST_MATCH_INT_DELAY 220 /* in microsec */
/* for irblaster that output the carrier */
/* a match interrupt will be set at best_match period before overflow */
/* the interrupt will loop to wait the overflow (actually overflow - period/2 */
/* so the best_match should be set as lowest as possible */
/* and we consider that 200 us before the overflow is good enough */
/* except if (min_cycle-2) is less than that ... */
/* the int will loop during: */
/* 200us-time_for_int_to_come(= about 50us) - (1/carrier_freq)/2 */
/* so it is a question of 120us */
static int _get_best_match_timing_int(int min_cycle, int carrier_period, int timer_freq)
{
	int  best_period;
	/* default match is 2 period after timer start */
	int  best_match = (min_cycle-2)*carrier_period;
	/* timer_freq  -> 1s */
	/* best_period -> BEST_MATCH_INT_DELAY.10e-6 */
	/* best_period = timer_freq*BEST_MATCH_INT_DELAY/(10e6) */
	best_period = BEST_MATCH_INT_DELAY*(timer_freq / 1000000);

	/* but put the best period to the minimum that works  */
	if ( best_period < best_match ) {
		best_match = best_period;
	}

	return best_match;

}

/* in MHz */
#define INNOTECH_BASE_FREQ 4
#define MHZ		1000000
static u32 _get_innotech_period(int timer_rate, int carrier_frequency) 
{
	u32 ret;
	
	ret = ((timer_rate/INNOTECH_BASE_FREQ)*carrier_frequency)/MHZ;
// 	printk("innotech period is: %d\n",ret);
	return ret;
}

static void _set_innotech_speed(int id, int frequency, int duty_cycle) 
{
	u32 val;
	u32 period;

	/* and you will have an overflow in 1 sec         */
	/* so,                              */
	/* freq_timer     -> 1s             */
	/* carrier_period -> 1/carrier_freq */
	/* => carrier_period = freq_timer/carrier_freq */
	period = _get_innotech_period(gpt_pwm_list[id].rate, frequency);
	gpt_pwm_list[id].period = period;
	val = 0xFFFFFFFF+1-period;
	
	omap_dm_timer_set_load(gpt_pwm_list[id].timer, 1, val);

	val = 0xFFFFFFFF+1-(period*duty_cycle/256);
	omap_dm_timer_set_match(gpt_pwm_list[id].timer, 1, val);

	/* set overflow first: pwm will not toggle if first trig is match */
	omap_dm_timer_write_counter(gpt_pwm_list[id].timer,0xFFFFFFFE);
}

static void gp_irblaster_init(int carrier_frequency, int no_carrier, unsigned short *pt_cycles_list) 
{
	u32 val;
	u32 period;
	int min_cycle=0x70000000; /* this min cycle can be set to another minimum: but which value ? */
	unsigned short *pt = pt_cycles_list;

	irb_over = 0;

	//_select_irb_config();

	/* find min cycle */
	while ( *pt != 0) {
		if ( *pt < min_cycle )
			min_cycle = *pt;
		pt++;
	}

	/* enable timer clock */
	omap_dm_timer_enable(gpt_pwm_list[IRBLASTER_PWM].timer);
	omap_dm_timer_enable(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer);

	irq_irblaster_cycles = pt_cycles_list;

	if ( no_carrier ) {
		/* set the callback irq function */
		_callback_irblaster_timer_interrupt = omap2_irblaster_timer_interrupt_no_carrier;

		/* set IRBLASTER TIMER PWM */
		/* default output is 0 */
		/* toogle */
		/* trigger on overflow */
		omap_dm_timer_set_pwm(gpt_pwm_list[IRBLASTER_PWM].timer,0,1,1);
		/* set frequency */
		period = _get_innotech_period(gpt_pwm_list[IRBLASTER_PWM].rate,carrier_frequency);
		gpt_pwm_list[IRBLASTER_PWM].period = period;

		/* at init, nothing to do during few cycles */
		/* and let say, during min cycle */
		val = 0xFFFFFFFF+1-(period*min_cycle);
		omap_dm_timer_set_load(gpt_pwm_list[IRBLASTER_PWM].timer, 1, val);


		/* set IRBLASTER TIMER CTRL */
		/* timer freq is the same as the carrier */
		period = _get_innotech_period(gpt_pwm_list[IRBLASTER_TIMER_CTRL].rate,carrier_frequency);
		gpt_pwm_list[IRBLASTER_TIMER_CTRL].period = period;

		/* at init, nothing to do during few cycles */
		/* and let say, during min cycle */
		val = 0xFFFFFFFF+1-(period*min_cycle);
		omap_dm_timer_set_load(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer, 1, val);

		/* prepare the irblaster match condition */
		/* irblaster timer run twice time faster than the pwm */
		/* so this match is set on the middle of the min cycle time */
		val = 0xFFFFFFFF+1-(period*min_cycle/2);
		omap_dm_timer_set_match(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer, 1, val);

	} else {
		/* set the callback irq function */
		_callback_irblaster_timer_interrupt = omap2_irblaster_timer_interrupt_carrier;

		/* set IRBLASTER TIMER PWM */
		/* default output is 0 */
		/* toogle */
		/* trigger on match and overflow */
		omap_dm_timer_set_pwm(gpt_pwm_list[IRBLASTER_PWM].timer,0,1,2);
		/* set waveform frequency and duty cycle 1/2 = 128/256 */
		_set_innotech_speed(IRBLASTER_PWM,carrier_frequency,128);

		/* at init, no trigger = default output on pin */
	 	irq_irblaster_pwm_ctrl_reg = omap_dm_timer_read_reg(gpt_pwm_list[IRBLASTER_PWM].timer, OMAP_TIMER_CTRL_REG);
		irq_irblaster_pwm_ctrl_reg &= ~(3<<10);
		omap_dm_timer_write_reg(gpt_pwm_list[IRBLASTER_PWM].timer, OMAP_TIMER_CTRL_REG, irq_irblaster_pwm_ctrl_reg);

		/* make the start readuy */
		irq_irblaster_pwm_ctrl_reg |= OMAP_TIMER_CTRL_ST;

		/* set IRBLASTER TIMER CTRL */
		/* timer freq is the same as the carrier */
		period = _get_innotech_period(gpt_pwm_list[IRBLASTER_TIMER_CTRL].rate,carrier_frequency);
		gpt_pwm_list[IRBLASTER_TIMER_CTRL].period = period;
	
		/* at init, nothing to do during few cycles */
		/* and let say, during min cycle */
		val = 0xFFFFFFFF+1-(period*(*irq_irblaster_cycles));
		omap_dm_timer_set_load(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer, 1, val);
	
		/* set the irblaster ctrl timer match condition */
		/* to the middle of min cycle */
		/* from (end_of_cycle - min_cycle/2) to end_of_cycle, */
		/* the timer ctrl irq will pool ...*/
//		val = 0xFFFFFFFF+1-(period*min_cycle/2);
		val =  0xFFFFFFFF+1-_get_best_match_timing_int(min_cycle,gpt_pwm_list[IRBLASTER_TIMER_CTRL].period,gpt_pwm_list[IRBLASTER_TIMER_CTRL].rate);
		omap_dm_timer_set_match(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer, 1,val );

	}

	omap_dm_timer_start(gpt_pwm_list[ IRBLASTER_TIMER_CTRL].timer);
	omap_dm_timer_start(gpt_pwm_list[ IRBLASTER_PWM].timer);
}

static void gp_irblaster_start(void) 
{
	pwm_gpt_start(IRBLASTER_PWM);
	pwm_gpt_start(IRBLASTER_TIMER_CTRL);

}

static void gp_irblaster_stop(void) 
{
  	pwm_gpt_stop(IRBLASTER_PWM);
	pwm_gpt_stop(IRBLASTER_TIMER_CTRL);
}

static void gp_irblaster_wait_end(void) {
	wait_event_interruptible(irb_wait,irb_over);
	//_select_default_config();

	/* stop and disable timer clock */
	pwm_gpt_stop(IRBLASTER_TIMER_CTRL);
	pwm_gpt_stop(IRBLASTER_PWM);
}

static irblast_t irblast_test = {
	.numRepeats = 1,
};

static irblast_data_t irblast_data_test = {
	.OnceCtr = 5,
	.RepeatCtr = 1,
	.Data = {22,44,22,44,22,44,22,44,22,44,22,44},
// 	.Data = {22,44},
	.carrier = 0x66,
	.details =0,
	};

static int irblast_do(irblast_t *irblast, irblast_data_t *data)
{
	int i,j;
	unsigned short *data_to_send;
	unsigned short *pt;
	int size;
	int oncectr;
	int repeatctr;
	int numrepeats;
	int carrier = data->carrier;
	int no_carrier;

	if (data->details & CONSTANT_CARRIER)
		no_carrier = 1;
	else
		no_carrier = 0;
		
	numrepeats = irblast->numRepeats;
	
	oncectr   = data->OnceCtr * 2;
	repeatctr = data->RepeatCtr * 2;
	numrepeats = irblast->numRepeats;

	size = oncectr + repeatctr*numrepeats +1; /* +1 for the zero */

	data_to_send = kmalloc( sizeof(unsigned short) * size, GFP_KERNEL );
	if ( data_to_send == NULL) {
		printk(KERN_ERR "not enough mem (%d) for irblaster\n",size);
		return -1;
	}
	memset(data_to_send,0, (sizeof(unsigned short) * size));

	pt = data_to_send;
	for( i = 0; i < oncectr; i++ ) {
		*pt++ = data->Data[i];
	}
	for (j=0;j<numrepeats;j++) {
		for (i = oncectr; i<(oncectr+repeatctr) ; i++ ) {
			*pt++ = data->Data[i];
		}
	}

	gp_irblaster_init(carrier, no_carrier, data_to_send);
	gp_irblaster_start();

	/* wait for end */
	gp_irblaster_wait_end();
	/* to do */

	kfree(data_to_send);
	return 0;
}

static int irblast_ioctl(struct inode * inode, struct file *filp,
	     unsigned int cmd, unsigned long arg)
{
	irblast_t *irblast = (irblast_t*)filp->private_data;

	int ret = -EINVAL;
	unsigned int tmp;
		
	switch (cmd) {
	case IRBLAST_SET_REPEATS:
		if( get_user(tmp, (int *) arg) ) {
			ret = -EFAULT;
			break;
		}			
		irblast->numRepeats = tmp;
		ret = 0;
		break;
	case IRBLAST_DUMP_REGS:
			pwm_gpt_dump(0);
			pwm_gpt_dump(1);
	break;
	case IRBLAST_WRITE_SINGLE:
	//	omap_cfg_reg(gpt_pwm_list[IRBLASTER_PWM].mux_config);
		irblast_do(&irblast_test, &irblast_data_test);
	break;
	case IRBLAST_SET_GIO:
		printk(KERN_INFO "IRBLAST_SET_GIO ioctl is disabled\n");
	//	omap_cfg_reg(GIO_TX_PIN);
	//	omap_set_gpio_direction(GIO_TX, GPIO_DIR_OUTPUT);
	//	omap_set_gpio_dataout(GIO_TX, 1);
	break;
	case IRBLAST_CLEAR_GIO:
		printk(KERN_INFO "IRBLAST_CLEAR_GIO ioctl is disabled\n");
	//	omap_cfg_reg(GIO_TX_PIN);
	//	omap_set_gpio_direction(GIO_TX, GPIO_DIR_OUTPUT);
	//	omap_set_gpio_dataout(GIO_TX, 0);
	break;

	default:
		break;
	}
	return ret;
}

static ssize_t irblast_write(struct file * filp, const char * buffer, size_t count, loff_t * l)
{
	irblast_t* irblast = (irblast_t*)filp->private_data;
	irblast_data_t *data;
	int ret = 0;

	if( count != sizeof(irblast_data_t) ) {
		return -EFAULT;
	}

	if( NULL == (data = kmalloc( sizeof( irblast_data_t ), GFP_KERNEL )) ) {
		return -EFAULT;
	}

	if( 0 != copy_from_user( data, buffer, count ) ) {
		kfree( data );
		return -EFAULT;
	}

	if( data->OnceCtr == 0 && data->RepeatCtr == 0 ) {
		kfree( data );
		return -EFAULT;
	}

	if( irblast_do(irblast, data) )	{
		ret = -EFAULT;
		printk( KERN_ERR "irblast_do timed out\n" );
	} else {
		ret = sizeof(irblast_data_t);
	}

	kfree( data );

	return ret;
}

static int irblast_open(struct inode * inode, struct file * filp)
{
	filp->private_data = &irblast;
	irblast.numRepeats = NUM_OF_REPEATS; /* default */

	return 0;
}

static int irblast_release(struct inode * inode, struct file * filp)
{
	return 0;
}


static struct file_operations irblast_fops = {
	write:          irblast_write,
	ioctl:		irblast_ioctl,
	open:		irblast_open,
	release:	irblast_release,
};

static struct miscdevice irblast_miscdev = {
	.minor	= IRBLAST_MINOR,
	.name	= IRBLAST_NAME,
	.fops	= &irblast_fops,
};

/* +++++++++++++ End File operations ++++++++++++++*/


static int _init_gpt(void) 
{
	int irqno;
	int ret = 0;

	if ((gpt_pwm_list[IRBLASTER_PWM].timer = omap_dm_timer_request_specific(gpt_pwm_list[IRBLASTER_PWM].no)) == NULL) {
		printk(KERN_ERR "%s: failed to request dm-timer (%d)\n", gpt_pwm_list[IRBLASTER_PWM].name, gpt_pwm_list[IRBLASTER_PWM].no);
		return -ENODEV;
	}

	omap_dm_timer_set_source(gpt_pwm_list[IRBLASTER_PWM].timer, gpt_pwm_list[IRBLASTER_PWM].source);
	gpt_pwm_list[IRBLASTER_PWM].rate = clk_get_rate(omap_dm_timer_get_fclk(gpt_pwm_list[IRBLASTER_PWM].timer));
	printk(KERN_DEBUG "%s (%d) timer rate is: %d\n", gpt_pwm_list[IRBLASTER_PWM].name, gpt_pwm_list[IRBLASTER_PWM].no, gpt_pwm_list[IRBLASTER_PWM].rate);
	
	if ((gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer = omap_dm_timer_request_specific(gpt_pwm_list[IRBLASTER_TIMER_CTRL].no)) == NULL) {
		printk(KERN_ERR "%s: failed to request dm-timer (%d)\n", gpt_pwm_list[IRBLASTER_TIMER_CTRL].name, gpt_pwm_list[IRBLASTER_TIMER_CTRL].no);
		omap_dm_timer_free(gpt_pwm_list[IRBLASTER_PWM].timer);
		return -ENODEV;
	}

	omap_dm_timer_set_source(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer, gpt_pwm_list[IRBLASTER_TIMER_CTRL].source);
	gpt_pwm_list[IRBLASTER_TIMER_CTRL].rate = clk_get_rate(omap_dm_timer_get_fclk(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer));
	printk(KERN_DEBUG "%s (%d) timer rate is: %d\n", gpt_pwm_list[IRBLASTER_TIMER_CTRL].name, gpt_pwm_list[IRBLASTER_TIMER_CTRL].no, gpt_pwm_list[IRBLASTER_TIMER_CTRL].rate);

	/* select irblaster configuration */
	_select_irb_config();

	/* enable timer clock */
	omap_dm_timer_enable(gpt_pwm_list[IRBLASTER_PWM].timer);
	omap_dm_timer_enable(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer);

	/* set modulation mod */
	/* trigger on overflow and match */
	omap_dm_timer_set_pwm(gpt_pwm_list[IRBLASTER_PWM].timer,0,1,2);
	omap_dm_timer_set_pwm(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer, 0, 1, 2);

	//if ( gpt_pwm_list[IRBLASTER_TIMER_CTRL].mux_config )
	//	omap_cfg_reg(gpt_pwm_list[IRBLASTER_TIMER_CTRL].mux_config);

	irqno = omap_dm_timer_get_irq(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer);
	if (request_irq(irqno, omap2_irblaster_timer_interrupt, 
			IRQF_DISABLED, "gp timer", NULL)) {
		ret = -EBUSY;
		goto failed_request_irq;
	}
	omap_dm_timer_set_int_enable(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer, IRBLASTER_TIMER_CTRL_IT_TYPE);

	/* disable timer clocks after init */
	omap_dm_timer_disable(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer);
	omap_dm_timer_disable(gpt_pwm_list[IRBLASTER_PWM].timer);

	init_waitqueue_head(&irb_wait);

	_gio_debug_init();

	return 0;

failed_request_irq:
	omap_dm_timer_free(gpt_pwm_list[IRBLASTER_PWM].timer);
	omap_dm_timer_free(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer);
	return ret;
}

static int __init irblaster_probe(struct platform_device *pdev)
{
	irblast_t* pt_irblast = &irblast;
	int ret;

	gpt_pwm_list = (struct omap_pwm_timer*) pdev->dev.platform_data;
	if (gpt_pwm_list == NULL) {
		printk(KERN_ERR "irblaster_probe: No platform data !\n");
		return -ENOMEM;
	}


	if ( (ret = _init_gpt()) < 0 ) {
		printk(KERN_ERR "irblaster_probe: cir init error: %d\n", ret);
		return ret;
	}

	/* register our misc device */
	if ((ret = misc_register(&irblast_miscdev)) != 0) {
		printk(KERN_ERR "irblaster_probe: cannot register miscdev on minor=%d (err=%d)\n",
			MISC_DYNAMIC_MINOR, ret);
		return ret;
	}
	
	printk(KERN_INFO "irblaster_probe: archos irblaster driver registered. minor: %d\n", irblast_miscdev.minor);

	memset(pt_irblast, 0, sizeof(irblast_t));

	return 0;
}

static int irblaster_remove(struct platform_device *pdev)
{
	int irqno = omap_dm_timer_get_irq(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer);

	free_irq(irqno, NULL);
	misc_deregister(&irblast_miscdev);
	
	_select_default_config();

	omap_dm_timer_free(gpt_pwm_list[IRBLASTER_PWM].timer);
	omap_dm_timer_free(gpt_pwm_list[IRBLASTER_TIMER_CTRL].timer);

	return 0;
}

static struct platform_driver irblaster_driver = {
	.probe		= irblaster_probe,
	.remove		= irblaster_remove,
	.driver		= {
		.name	= IRB_DEV_NAME,
	},
};

static int __devinit irblaster_init_module(void)
{
	return platform_driver_register(&irblaster_driver);
}

static void __exit irblaster_exit_module(void)
{
	platform_driver_unregister(&irblaster_driver);
}


module_init( irblaster_init_module );
module_exit( irblaster_exit_module );

/* Module information */
MODULE_AUTHOR("Paul EBEYAN, ebeyan@archos.com");
MODULE_DESCRIPTION("IR-Blaster Driver");
MODULE_LICENSE("GPL");
