/*
 *  archos_irremote.c 
 *
 * Copyright 2009 Archos
 * Author: Jinqin DONG
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/kthread.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <mach/dmtimer.h>
#include <mach/mux.h>
#include <mach/clock.h>
#include <mach/irqs.h>
#include <mach/resource.h>
#include <mach/irremote.h>
#include <mach/archos-accessory.h>


//#define OMAP3_IRR_DEBUG

//#define DEBUG_MATCH_INT 
// DEBUG_MATCH_INT: use gptimer match event INT as a key release action (doesn't work yet), we use a simple 80ms timer in irrthread for key release decision

//#define OMAP3_IRR_NO_MSR
//treat Mousekey as the others, do not sent a key released event after one mousekey pressed

// #define IRR_RISING_CAP
// capture event on rising edge event 

#define IRR_FALLING_CAP
// capture event on falling edge event 


#ifdef OMAP3_IRR_DEBUG
	#define DBG_INT_BIT  (1<<0)
	#define DBG_DECODER_BIT  (1<<1)
	#define DBG_KEY_BIT  (1<<2)
	
	static int dbgmode = 0;
	
	#define DBG_INT(fmt, args...) if (dbgmode&DBG_INT_BIT) printk(KERN_ERR "\n" fmt,  ## args)
	#define DBG_DECODER(fmt, args...) if (dbgmode&DBG_DECODER_BIT) printk(KERN_ERR "\n" fmt, ## args)
	#define DBG_KEY(fmt, args...) if (dbgmode&DBG_KEY_BIT) printk(KERN_ERR "\n: " fmt, ## args)
	
	module_param(dbgmode, int, S_IRUGO|S_IWUSR);
	MODULE_PARM_DESC(dbgmode, "debug mode select:0x0 interal;0x2 decoder;0x04 key action");	
#else /* OMAP3_IRR_DEBUG */
	#define DBG_INT(fmt, args...)
	#define DBG_DECODER(fmt, args...)
	#define DBG_KEY(fmt, args...)
#endif  /* OMAP3_IRR_DEBUG */


#if defined CONFIG_IRPRODTEST_EN
	#define DBG_IN_BUFF_BIT  (1<<0)
	#define DBG_MATCH_VAL_BIT  (1<<1)
	
	#define DBG_IN_BUFF(state,val) if (prodtestmode&DBG_IN_BUFF_BIT) _add_report_list(state,val)
	#define DBG_MATCH_VAL(val) if (prodtestmode&DBG_MATCH_VAL_BIT) _check_match(val)
	
	#define LIST_SIZE 50
	
	struct state_time_list {
		int state;
		int time;
	};
	
	static int current_list_pt = 0;
	static int nb_report = 0;
	static struct state_time_list _list[LIST_SIZE];
	
	static int prodtestmode = 0;
	static int prodtest_match = 0;
	static int prodtest_match_val = -1;
	
	module_param(prodtestmode, int, S_IRUGO|S_IWUSR);
	MODULE_PARM_DESC(prodtestmode, "prodtest mode select:0x0 nothing;0x1 enable match: 0x2 traces in buff");
	module_param(prodtest_match, int, S_IRUGO|S_IWUSR);
	MODULE_PARM_DESC(prodtest_match, "1: decoded word has match prodtest_match_val");
	module_param(prodtest_match_val, int, S_IRUGO|S_IWUSR);
	MODULE_PARM_DESC(prodtest_match_val, "match value for prodtest");
#else /* CONFIG_IRPRODTEST_EN */
	#define DBG_IN_BUFF(state,val) while(0){}
	#define DBG_MATCH_VAL(val) while(0){}
#endif /* CONFIG_IRPRODTEST_EN */


//#define GPTIMER9_BASE					0x49040000
#define OMAP_TIMER_TCLR_REG 				0x24
#define OMAP_TIMER_TIER_REG 				0x1c
#define OMAP_TIMER_TWER_REG 				0x20
#define OMAP_TIMER_TMAR_REG				0x38

#define OMAP_TIMER_TCLR_REG_GPO_CFG			(1<<14)
#define OMAP_TIMER_TCLR_REG_CAPT_MODE			(1<<13)
#define OMAP_TIMER_TCLR_REG_PULSE_TOOGLE		(1<<12)
#define OMAP_TIMER_TCLR_REG_TRIGGER_OUTPUT_MODE 	(1<<10)
#define OMAP_TIMER_TCLR_REG_TRANSITION_CAPT_MODE	(1<<8)
#define OMAP_TIMER_TCLR_REG_COMPARE_ENABLE		(1<<6)
#define OMAP_TIMER_TCLR_REG_START_STOP_TIMER		(1<<0)

#define OMAP_TIMER_TCAR1_REG 				0x3c
#define OMAP_TIMER_TCAR2_REG 				0x44

#define OMAP_TIMER_STAT_REG				0x18
#define OMAP_TIMER_WRITE_PEND_REG			0x34
#define OMAP_TIMER_COUNTER_REG				0x28

/* timer interrupt enable bits */
#define OMAP_TIMER_INT_CAPTURE			(1 << 2)
#define OMAP_TIMER_INT_OVERFLOW			(1 << 1)
#define OMAP_TIMER_INT_MATCH			(1 << 0)

#define IRR_KEYRELEASE_TIMEOUT		msecs_to_jiffies(80)

/* Emulated relative mouse pointer */
/* Maximum step (speed of the cursor) allowed */
#define IRR_MOUSE_STEP_MAX 14
/* Number of press needed to increase the speed of the cursor */
#define IRR_MOUSE_STEP_DELAY 3

#define IRR_KEYS sizeof(IRRcodes)/sizeof(struct tIRRcodes)
/* map IRR codes to linux key codes */
#define KEY_UNMAPPED	0

static struct constraint_handle *co_lat_mpu;
static struct constraint_id cnstr_id_lat_mpu = {
	.type = RES_LATENCY_CO,
	.data = (void *)"latency",
};

//static struct omap_dm_timer *gptimer;
static wait_queue_head_t irrt_wait;
static int key_action_ok = 0;
static struct task_struct *irr_keymap_task;
static struct input_dev* irr_dev;
static struct input_dev* irr_mouse_dev;

// Emulated relative mouse pointer
static int irr_mouse_pressed = 0;
static int irr_mouse_count;

static u32 IRR_TimePrevious = 0;

static struct timer_list irrtimer_releasekey;
static int irr_timer_is_armed;
static int irr_wait_for_mouse_left;

static struct omap_dm_timer *dm_timer;

void irr_keyrelease(unsigned long d);


#if defined CONFIG_IRPRODTEST_EN
static void _add_report_list(int state, int time)
{
	_list[current_list_pt].state=state;
	_list[current_list_pt].time=time;
	current_list_pt++;
	nb_report ++;
	if ( current_list_pt == LIST_SIZE) {
		nb_report = LIST_SIZE;
		current_list_pt = 0;
	}
}

static ssize_t _display_report_list(char *buf)
{
	int idx=current_list_pt;
	int i;
	ssize_t ret=0;
	ssize_t pos=0;
	
	if (nb_report == 0) {
		ret += sprintf(buf,"nothing to report\n");
		return ret;
	}
	pos = sprintf(buf,"reports: %d\n",nb_report);
	buf += pos;
	ret += pos;

	for (i=0;i<nb_report;i++) {
		idx--;
		if ( idx == -1)
			idx = LIST_SIZE-1;

	}
	
	for (i=0;i<nb_report;i++) {
		pos = sprintf(buf,"0x%x 0x%x\n",_list[idx].state,_list[idx].time);
		buf += pos;
		ret += pos;
		idx++;
		if ( idx == LIST_SIZE)
			idx = 0;
	}
	return ret;
}

static void _clear_report_list(void)
{
	printk("clear list\n");
	current_list_pt = nb_report =0;
	memset(_list,0,sizeof(_list));
}

static ssize_t display_list(struct device *dev, struct device_attribute *attr, char* buf)
{
	return _display_report_list(buf);
}

static ssize_t set_list(struct device *dev, struct device_attribute *attr, const char* buf, size_t len)
{
	int cmd = simple_strtol(buf, NULL, 10);	
	if (cmd == 0)
		_clear_report_list();
	
	return len;
}

static DEVICE_ATTR(debug_list, S_IRUGO|S_IWUSR, display_list, set_list);

static void _check_match(int val) {
	prodtest_match |= (val == prodtest_match_val);
// 	prodtest_match = val;
}
#endif /* CONFIG_IRPRODTEST_EN */


// **************************************************************************
// Remote control button codes TO standard input device key codes

struct tIRRcodes {
	u32 irrcode;
	u16 keycode;
};

static struct tIRRcodes IRRcodes[] = {

	// GEN5/GEN6 common codes

	{ 0x0000847B, KEY_POWER},		// Power
	{ 0x0000a05f, KEY_REPLY}, 		// Ok - Play
	{ 0x00009669, KEY_UP},			// Up
	{ 0x00009c63, KEY_DOWN},		// Down
	{ 0x00009867, KEY_LEFT},		// Left
	{ 0x00009a65, KEY_RIGHT},		// Right
	{ 0x0000946b, KEY_MENU},		// Menu
	{ 0x00008e71, KEY_BACK},		// Esc
	{ 0x00008c73, KEY_FRONT},		// Tab
	{ 0x00008a75, KEY_MUTE},		// Mute
	{ 0x00008877, KEY_VOLUMEUP},		// Volume up
	{ 0x00008679, KEY_VOLUMEDOWN},		// Volume down
	{ 0x0000827d, KEY_SWITCHVIDEOMODE},	// TV

	{ 0x000040bf, KEY_Q},
	{ 0x000042bd, KEY_W},
	{ 0x000044bb, KEY_E},
	{ 0x000046b9, KEY_R},
	{ 0x000048b7, KEY_T},
	{ 0x00004ab5, KEY_Y},
	{ 0x00004cb3, KEY_U},
	{ 0x00004eb1, KEY_I},
	{ 0x000050af, KEY_O},
	{ 0x000052ad, KEY_P},
	{ 0x000054ab, KEY_A},
	{ 0x000056a9, KEY_S},
	{ 0x000058a7, KEY_D},
	{ 0x00005aa5, KEY_F},
	{ 0x00005ca3, KEY_G},
	{ 0x00005ea1, KEY_H},
	{ 0x0000609f, KEY_J},
	{ 0x0000629d, KEY_K},
	{ 0x0000649b, KEY_L},
	{ 0x00006699, KEY_Z},
	{ 0x00006897, KEY_X},
	{ 0x00006a95, KEY_C},
	{ 0x00006c93, KEY_V},
	{ 0x00006e91, KEY_B},
	{ 0x0000708f, KEY_N},
	{ 0x0000728d, KEY_M},
	{ 0x0000748b, KEY_COMMA},
	{ 0x00007689, KEY_DOT},
	{ 0x0000a25d, KEY_BACKSPACE},
	{ 0x0000c23d, KEY_LEFTALT},
	{ 0x0000c43b, KEY_RIGHTALT},
	{ 0x00007887, KEY_SPACE},
	{ 0x0000c03f, KEY_LEFTSHIFT},

	{ 0x00000201, KEY_KP8},			// CIRCLE_UP
	{ 0x00001001, KEY_KP2},			// CIRCLE_DOWN
	{ 0x00000401, KEY_KP4},			// CIRCLE_LEFT
	{ 0x00000801, KEY_KP6},			// CIRCLE_RIGHT
	{ 0x00000c01, KEY_KP7},			// CIRCLE_UP_LEFT
	{ 0x00000601, KEY_KP9},			// CIRCLE_UP_RIGHT
	{ 0x00001201, KEY_KP1},			// CIRCLE_DOWN_LEFT
	{ 0x00001801, KEY_KP3},			// CIRCLE_DOWN_RIGHT
	{ 0x000014eb, KEY_KPPLUS},		// MOUSE_LEFT

	// GEN5 specific codes

	{ 0x0000906f, KEY_LEFTBRACE},		// Page up
	{ 0x0000926d, KEY_RIGHTBRACE},		// Page down

	{ 0x0000c639, KEY_RIGHTSHIFT},

	{ 0x00000af5, KEY_KPMINUS},		// MOUSE_RIGHT
	{ 0x00009e61, KEY_KPDOT},		// HAND

	// GEN6 specific codes

	{ 0x0000a45b, KEY_HOME},		// G6_HOME
	{ 0x0000a659, KEY_PAGEDOWN},		// G6_FASTREWIND
	{ 0x0000a857, KEY_PAGEUP},		// G6_FASTFORWARD
	{ 0x0000aa55, KEY_TAB},			// G6_KEYB_TAB
	{ 0x0000ac53, KEY_ENTER},		// G6_KEYB_RETURN

	{ 0x00000e01, KEY_KATAKANA},		// G6_CIRCLEUP_LP
	{ 0x00001601, KEY_HIRAGANA},		// G6_CIRCLELEFT_LP
	{ 0x00001a01, KEY_HENKAN},		// G6_CIRCLERIGHT_LP
	{ 0x00001c01, KEY_KATAKANAHIRAGANA},	// G6_CIRCLEDOWN_LP
	{ 0x00001e01, KEY_MUHENKAN},		// G6_CIRCLELEFTUP_LP
	{ 0x00002e01, KEY_KPJPCOMMA},		// G6_CIRCLERIGHTUP_LP
	{ 0x00003601, KEY_KPENTER},		// G6_CIRCLELEFTDOWN_LP
	{ 0x00003a01, KEY_KPSLASH},		// G6_CIRCLERIGHTDOWN_LP

};

typedef enum {
	StateIdle	=0,
	StateHeader	=1,
	StateCode1	=2,
	StateCode2	=3
} IRRStateValues;

static IRRStateValues IRR_State;

typedef enum {
	irrcode_received	=0,
	irrcode_finished	=1,   //no new code after one irrcode received
	header_error		=2,
	decoder_error		=3,
	irrcode_repeated	=4
} IRRActionValues;

static IRRStateValues IRR_State = StateIdle;
static IRRActionValues IRR_Action;

static volatile u32 davinci_irr_irrcode;
static u32 std_key_code;		// the last valid std_key_code that was received
static u32 rpt_key_code = 0;		// the last valid std_key_code that was reported to standard input device

/*
static inline unsigned int _gpt9_read(unsigned int reg_offset)
{
	return readl( io_p2v(GPTIMER9_BASE) + reg_offset);

}

static inline void _gpt9_write(unsigned int reg_offset,unsigned int value)
{
	writel(value, (io_p2v(GPTIMER9_BASE) + reg_offset) );
	while (readl( io_p2v(GPTIMER9_BASE) + OMAP_TIMER_WRITE_PEND_REG) );
}
*/

static int irr_gtimer_init(void)
{
	printk("irr_gtimer_init  \n");

	/* REVISIT: Check 24xx TIOCP_CFG settings after idle works */

#ifdef IRR_RISING_CAP
	omap_dm_timer_write_reg(dm_timer, OMAP_TIMER_TCLR_REG,0x4123);
#endif
#ifdef IRR_FALLING_CAP
	omap_dm_timer_write_reg(dm_timer, OMAP_TIMER_TCLR_REG,0x4223);
#endif

	omap_dm_timer_set_int_enable(dm_timer,
		OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_OVERFLOW /*|
		OMAP_TIMER_INT_MATCH*/);

#ifndef CONFIG_OMAP3430_ES2
	/* Sil errata 1.31, first write access to TCRR discarded on ES1.0*/
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)){
		omap_dm_timer_write_reg(dm_timer, OMAP_TIMER_COUNTER_REG,0x0);
		while(omap_dm_timer_read_reg(dm_timer, OMAP_TIMER_WRITE_PEND_REG));
	}
#endif
	return 0;
}

static void irr_gtimer_exit(void)
{
	printk("irr_gtimer_exit\n");
	omap_dm_timer_free(dm_timer);
}


// **************************************************************************
// Function that reports a key release
// (when no more key repeat code have been received for a given timeout)
void irr_keyrelease(unsigned long d)
{
	if ( irr_wait_for_mouse_left == 1 ) {
		irr_wait_for_mouse_left = 0;
		DBG_KEY("ignore mouse left");
		input_report_key(irr_dev, KEY_KPPLUS, 0); // released	
	} else {
/*		if ( rpt_key_code == KEY_KPPLUS ) {
			input_report_key(irr_dev, KEY_KPPLUS, 1); //pressed
			DBG_KEY("mouse left pressed\n");
		}*/						

		input_report_key(irr_dev, rpt_key_code, 0); // released
		DBG_KEY("kr%d",rpt_key_code);
	}
	
	if (irr_mouse_pressed) {
		input_report_key(irr_mouse_dev, BTN_LEFT, 0);
		irr_mouse_pressed = 0;
		input_sync(irr_mouse_dev);
	}

	irr_timer_is_armed = 0;
}

static u16 irr_GetKeyFromCode(u32 irrcode)
{
	int i;

	u16 key = KEY_UNMAPPED;
	for (i=0; i<IRR_KEYS; i++) {
		if (IRRcodes[i].irrcode == irrcode) {
			key = IRRcodes[i].keycode;
			break;
		}
	}
	return key;
}

static void update_mouse_position(void) {
	int irr_mouse_step;
	int irr_mouse_x = 0;
	int irr_mouse_y = 0;

	if (irr_timer_is_armed)			// It's a long press
		irr_mouse_count++;
	else
		irr_mouse_count = 1;

	irr_mouse_step = irr_mouse_count/IRR_MOUSE_STEP_DELAY;
	if (irr_mouse_step > IRR_MOUSE_STEP_MAX) irr_mouse_step = IRR_MOUSE_STEP_MAX;

	switch (std_key_code) {
		case KEY_KATAKANA:		// G6_CIRCLEUP_LP
		case KEY_KP8:			// CIRCLE_UP
			irr_mouse_y = -irr_mouse_step;
			break;
		case KEY_KATAKANAHIRAGANA:	// G6_CIRCLEDOWN_LP
		case KEY_KP2:			// CIRCLE_DOWN
			irr_mouse_y = irr_mouse_step;
			break;
		case KEY_HIRAGANA:		// G6_CIRCLELEFT_LP
		case KEY_KP4:			// CIRCLE_LEFT
			irr_mouse_x = -irr_mouse_step;
			break;
		case KEY_HENKAN:		// G6_CIRCLERIGHT_LP
		case KEY_KP6:			// CIRCLE_RIGHT
			irr_mouse_x = irr_mouse_step;
			break;
		case KEY_MUHENKAN:		// G6_CIRCLELEFTUP_LP
		case KEY_KP7:			// CIRCLE_UP_LEFT
			irr_mouse_x = -irr_mouse_step;
			irr_mouse_y = -irr_mouse_step;
			break;
		case KEY_KPJPCOMMA:		// G6_CIRCLERIGHTUP_LP
		case KEY_KP9:			// CIRCLE_UP_RIGHT
			irr_mouse_x = irr_mouse_step;
			irr_mouse_y = -irr_mouse_step;
			break;
		case KEY_KPENTER:		// G6_CIRCLELEFTDOWN_LP
		case KEY_KP1:			// CIRCLE_DOWN_LEFT
			irr_mouse_x = -irr_mouse_step;
			irr_mouse_y = irr_mouse_step;
			break;
		case KEY_KPSLASH:		// G6_CIRCLERIGHTDOWN_LP
		case KEY_KP3:			// CIRCLE_DOWN_RIGHT
			irr_mouse_x = irr_mouse_step;
			irr_mouse_y = irr_mouse_step;
			break;
		default:
			/* If another key is pressed, the cursor is not
				moving anymore, so we have to reset the cursor speed */
			irr_mouse_count = 0;
			break;
	}

	switch (std_key_code) {
		case KEY_KATAKANA:		// G6_CIRCLEUP_LP
		case KEY_KATAKANAHIRAGANA:	// G6_CIRCLEDOWN_LP
		case KEY_HIRAGANA:		// G6_CIRCLELEFT_LP
		case KEY_HENKAN:		// G6_CIRCLERIGHT_LP
		case KEY_MUHENKAN:		// G6_CIRCLELEFTUP_LP
		case KEY_KPJPCOMMA:		// G6_CIRCLERIGHTUP_LP
		case KEY_KPENTER:		// G6_CIRCLELEFTDOWN_LP
		case KEY_KPSLASH:		// G6_CIRCLERIGHTDOWN_LP
		case KEY_KPPLUS:		// MOUSE_LEFT
			if (!irr_mouse_pressed) {
				input_report_key(irr_mouse_dev, BTN_LEFT, 1);
				irr_mouse_pressed = 1;
			}
			break;
		default:
			if (irr_mouse_pressed) {
				input_report_key(irr_mouse_dev, BTN_LEFT, 0);
				irr_mouse_pressed = 0;
			}
			break;
	}

	input_report_rel(irr_mouse_dev, REL_X, irr_mouse_x);
	input_report_rel(irr_mouse_dev, REL_Y, irr_mouse_y);
	input_sync(irr_mouse_dev);
}

static int irr_keymap_thread(void *null)
{
	current->flags &= ~PF_NOFREEZE;
	
	DBG_KEY("IRR thread init\n");

	do {
		long wait_status;

		wait_status = wait_event_interruptible(irrt_wait, 
				key_action_ok|kthread_should_stop());

		if (kthread_should_stop())
			break;

		switch (IRR_Action) {
		case irrcode_received:
			std_key_code = irr_GetKeyFromCode(davinci_irr_irrcode);
			DBG_MATCH_VAL(davinci_irr_irrcode);
			if ( std_key_code != KEY_UNMAPPED ) {
				//DBG_KEY("%x received, matched to: %d\n",davinci_irr_irrcode,std_key_code);
				DBG_DECODER("%xr m%d\n",davinci_irr_irrcode,std_key_code);
#ifdef OMAP3_IRR_NO_MSR
				if ((davinci_irr_irrcode & 0xff) == 0x01 ) { 
					// we suppose it is not possible to press a mousekey 
					// 250ms after a normal key pressed
					input_report_key(irr_dev, std_key_code, 1); // pressed
					rpt_key_code = std_key_code;
					break;
				}
#endif
				update_mouse_position();

				if ( irr_timer_is_armed ) {
					if ( std_key_code == rpt_key_code ) {
						mod_timer(&irrtimer_releasekey, jiffies + IRR_KEYRELEASE_TIMEOUT);
					} else {
						if ( rpt_key_code == KEY_KPPLUS ) {		
							input_report_key(irr_dev, std_key_code, 1); // mousepad serie 2 pressed
							DBG_KEY("ms2p%d\n",std_key_code);
						} else if ( std_key_code == KEY_KPPLUS ) {		
							input_report_key(irr_dev, rpt_key_code, 0); // mousepad serie 2 released
							irr_wait_for_mouse_left = 1;
							DBG_KEY("ms2r%d\n",rpt_key_code);
							
						} else {		
							input_report_key(irr_dev, rpt_key_code, 0); // released
							input_report_key(irr_dev, std_key_code, 1); // pressed
							DBG_KEY("nk%d\n",std_key_code);
						}
						
						rpt_key_code = std_key_code;
						mod_timer(&irrtimer_releasekey, jiffies + IRR_KEYRELEASE_TIMEOUT);
					}
				} else {	
					// Send MOUSE LEFT button pressed event while releasing it
					input_report_key(irr_dev, std_key_code, 1); //pressed
					DBG_KEY("fk%d\n",std_key_code);
						
					irrtimer_releasekey.expires = jiffies + IRR_KEYRELEASE_TIMEOUT;
					irr_timer_is_armed = 1;
					add_timer(&irrtimer_releasekey);
					rpt_key_code = std_key_code;
				}

			} else {
				DBG_DECODER("%xr nm\n",davinci_irr_irrcode);
			};
			break;
			
		case irrcode_finished:
			DBG_DECODER("cf\n");
			break;
			
		case header_error:
			DBG_DECODER("er\n");
			if ( irr_timer_is_armed )
				mod_timer(&irrtimer_releasekey, jiffies + IRR_KEYRELEASE_TIMEOUT);
			break;
			
		case decoder_error:
			DBG_DECODER("der\n");
			if ( irr_timer_is_armed )
				mod_timer(&irrtimer_releasekey, jiffies + IRR_KEYRELEASE_TIMEOUT);
			break;
			
		default:
			break;
		}

		key_action_ok = 0;

	} while (1);
	return 0;
}


static void  _wake_thread(IRRActionValues irraction,int irrcode)
{
	if (irraction != irrcode_finished) 
		davinci_irr_irrcode = irrcode;
	IRR_Action = irraction;
	key_action_ok = 1;
	wake_up_interruptible( &irrt_wait );
}

#ifdef DEBUG_MATCH_INT
static void _enalbe_irq(void)
{
	u32 status = omap_dm_timer_read_reg(dm_timer, OMAP_TIMER_TIER_REG);
	omap_dm_timer_write_reg(dm_timer, OMAP_TIMER_TIER_REG , status|(OMAP_TIMER_INT_CAPTURE|OMAP_TIMER_INT_OVERFLOW));

	status = omap_dm_timer_read_reg(dm_timer, OMAP_TIMER_TWER_REG);
	omap_dm_timer_write_reg(dm_timer, OMAP_TIMER_TWER_REG , status|(OMAP_TIMER_INT_CAPTURE|OMAP_TIMER_INT_OVERFLOW));

}
#endif

inline static void _set_match_irq(int on)
{
	if (on == 1) {
		u32 status = omap_dm_timer_read_reg(dm_timer, OMAP_TIMER_TIER_REG);
		omap_dm_timer_write_reg(dm_timer, OMAP_TIMER_TIER_REG , status|OMAP_TIMER_INT_MATCH);
	} else {
		u32 status = omap_dm_timer_read_reg(dm_timer, OMAP_TIMER_TIER_REG);
		omap_dm_timer_write_reg(dm_timer, OMAP_TIMER_TIER_REG , status&~OMAP_TIMER_INT_MATCH);
	}
}

static inline void IRR_decode(u32 IRR_TimeDiff)
{
	static u16 IRR_Nbits ;
	static volatile u32 IRR_IncWord;

	DBG_IN_BUFF(IRR_State,IRR_TimeDiff);

	if (IRR_State == StateIdle && IRR_TimeDiff > 275 && IRR_TimeDiff < 290) {
		DBG_IN_BUFF(0xAA,IRR_TimeDiff);
		IRR_State = StateHeader;
		return;
	}

	if ( IRR_State == StateIdle ) {
		IRR_State = StateHeader;
	} else if ( IRR_State == StateHeader ) {
		// This may be the second edge i.e. end of the header
		if ( IRR_TimeDiff < 60) {
			_wake_thread(header_error,IRR_IncWord);
			IRR_State = StateIdle;
		} else
		if ( IRR_TimeDiff < 80 ) {
			//10
			// 29 = 14.5 ms
			// This was a normal header GEN5 header
			IRR_State = StateCode1;
			IRR_Nbits = 0;
			IRR_IncWord = 0;
		} else {
			// It is a too long pulse. we assume it is a new frame -> restart the operation
			_wake_thread(header_error, IRR_IncWord);
			IRR_State = StateIdle;
		}
	} else {
		//if ( IRR_State == State2Code ) {
		if ( IRR_TimeDiff <= 11 ) {
			// GEN5   0:<=6   1:<=10
			// We received a 0
			IRR_Nbits ++;
			IRR_IncWord = (IRR_IncWord << 1);
		} else if ( IRR_TimeDiff <= 19 ) {
			// 5 = 2.5 ms
			// We received a 1
			IRR_Nbits ++;
			IRR_IncWord = (IRR_IncWord << 1) + 1;
		} else if ( IRR_TimeDiff <= 22 ) {
				IRR_Nbits += 2;
				IRR_IncWord = (IRR_IncWord << 2);
		} else {
			_wake_thread(decoder_error,IRR_IncWord);
			IRR_State = StateIdle;
		}

		if ( IRR_Nbits == (( 3 - IRR_State) ? 17 : 32 )) {   //17 or 32 
			// We received a complete frame - let us try to get the key code from the frame
			_wake_thread(irrcode_received,IRR_IncWord);;
			IRR_State = StateIdle;
			IRR_IncWord = 0;
			IRR_Nbits = 0;
		}
	}
}

inline static irqreturn_t irr_timer_interrupt(int irq, void *dev_id)
{
	u32 IRR_TimeInterval = 0;
	u32 status = omap_dm_timer_read_reg(dm_timer, OMAP_TIMER_STAT_REG);
	if (status|OMAP_TIMER_INT_CAPTURE) {
		u32 IRR_TimeCurrent = omap_dm_timer_read_reg(dm_timer, OMAP_TIMER_TCAR1_REG);
#ifdef DEBUG_MATCH_INT
		omap_dm_timer_write_reg(dm_timer, OMAP_TIMER_TMAR_REG, IRR_TimeCurrent+ 2000);
		// set code finished timeout as 20ms(160) 
		// 250ms:2000
		_set_match_irq(1);
#endif
		IRR_TimeInterval = IRR_TimeCurrent - IRR_TimePrevious;
		IRR_TimePrevious = IRR_TimeCurrent;
		IRR_decode(IRR_TimeInterval);
		omap_dm_timer_write_reg(dm_timer, OMAP_TIMER_STAT_REG , status);
	} 
#ifdef DEBUG_MATCH_INT
	 else if (status|OMAP_TIMER_INT_MATCH) {
		// irrcode finised here!
		_set_match_irq(0);
		_wake_thread(irrcode_finished,0);
		omap_dm_timer_write_reg(dm_timer, OMAP_TIMER_STAT_REG , OMAP_TIMER_INT_MATCH);
	}
#endif

	return IRQ_HANDLED;
}

#define IRR_DEV_NAME	"archos-irremote"
#define IRR_MOUSE_DEV_NAME	"archos-irremote-mouse"

static int __init irremote_probe(struct platform_device *pdev)
{
	int ret = 0;
	int res = 0;
	int i = 0;
	int irqno;
	struct irremote_data *data;

	data = (struct irremote_data*) pdev->dev.platform_data;

	dm_timer = omap_dm_timer_request_specific(data->timer);
	if (dm_timer == NULL) {
		printk(KERN_ERR "irremote_probe: No timer in platform data !\n");
		return -ENOMEM;
	}

	omap_dm_timer_set_source(dm_timer, OMAP_TIMER_SRC_32_KHZ);

	irr_dev = input_allocate_device();
	if (irr_dev == NULL) {
		goto failed_timer;
	}
	
	irr_mouse_dev = input_allocate_device();
	if (irr_mouse_dev == NULL)
		goto failed_alloc;

	/* setup input device */
	set_bit(EV_KEY, irr_dev->evbit);
	set_bit(EV_KEY, irr_mouse_dev->evbit);
	set_bit(EV_REL, irr_mouse_dev->evbit);

	/* Enable auto repeat feature of Linux input subsystem */
	irr_dev->rep[REP_DELAY] = 1000;
	irr_dev->rep[REP_PERIOD] = 50;
	set_bit(EV_REP, irr_dev->evbit);

	irr_dev->name = IRR_DEV_NAME;
	irr_dev->phys = "archos-ir/input0";

	irr_mouse_dev->name = IRR_MOUSE_DEV_NAME;
	irr_mouse_dev->phys = "archos-ir-mouse/input0";

	for (i=0; i<IRR_KEYS; i++)
		input_set_capability(irr_dev, EV_KEY, IRRcodes[i].keycode);

	input_set_capability(irr_mouse_dev, EV_KEY, BTN_LEFT);
	input_set_capability(irr_mouse_dev, EV_KEY, BTN_RIGHT);
	input_set_capability(irr_mouse_dev, EV_REL, REL_X);
	input_set_capability(irr_mouse_dev, EV_REL, REL_Y);

	ret = input_register_device(irr_dev);
	if (ret < 0) {
		printk(KERN_DEBUG "irremote_probe: Unable to register ir-remote device\n");
		goto failed_dev;
	}

	ret = input_register_device(irr_mouse_dev);
	if (ret < 0) {
		printk(KERN_DEBUG "irremote_probe: Unable to register ir-remote mouse device\n");
		goto failed_dev2;
	}

	// Timer initialization for key release
	irr_timer_is_armed = 0;
	init_timer(&irrtimer_releasekey);
	irrtimer_releasekey.function = irr_keyrelease;
	irrtimer_releasekey.data = 0;

	irqno = omap_dm_timer_get_irq(dm_timer);

	res = request_irq(irqno, irr_timer_interrupt, IRQF_SHARED, IRR_DEV_NAME, irr_dev);
	if (res < 0) {	// res may be -EBUSY
		printk(KERN_ERR "irremote_probe: unable to allocate IRR IRQ %d!!\n", irqno);
		goto failed_irq;
	} else {
		printk("irremote_probe: enable IRR IRQ %d success ! \n", irqno);
	}

	// Kernel thread init and start
	init_waitqueue_head(&irrt_wait);
	irr_keymap_task = kthread_run(irr_keymap_thread, NULL, "irrd");
	if (irr_keymap_task <= 0) {
		printk(KERN_ERR "irremote_probe: unable to start kernel thread\n");
		goto failed_kthread;
	}

	/* Enabling IR, disabling UART-3 */
#ifdef CONFIG_BOTH_UART_IR
	archos_accessory_set_irmode("both");
#else
	archos_accessory_set_irmode("ir");
#endif

#if defined CONFIG_IRPRODTEST_EN
 	if (device_create_file( &(pdev->dev), &dev_attr_debug_list) < 0)
 		printk(KERN_DEBUG "irremote_probe: unable to create debug_list attribute\n");
#endif

	if (irr_gtimer_init())
		goto failed_gtimer;

	/* set a constraint on MPU latency, don't allow to dive into deep cpu idle */
	co_lat_mpu = constraint_get("irremote", &cnstr_id_lat_mpu);
	constraint_set(co_lat_mpu, CO_LATENCY_ON);

	return 0;

failed_gtimer:
	kthread_stop(irr_keymap_task);
failed_kthread:
	// Release the IRQ
    	free_irq(irqno, irr_dev);
failed_irq:
	input_unregister_device(irr_mouse_dev);
failed_dev2:
	input_unregister_device(irr_dev);
failed_dev:
	input_free_device(irr_mouse_dev);
failed_alloc:
	input_free_device(irr_dev);
failed_timer:
	omap_dm_timer_free(dm_timer);
	return -ENODEV;
}

static int irremote_remove(struct platform_device *pdev)
{
#if defined CONFIG_IRPRODTEST_EN
	device_remove_file(&(pdev->dev), &dev_attr_debug_list);
#endif
	free_irq(omap_dm_timer_get_irq(dm_timer), irr_dev);
	printk("irremote_remove: remove irremote module\n");
	
	input_unregister_device(irr_dev);
	input_unregister_device(irr_mouse_dev);
	printk("irremote_remove: unregister device success\n");
	kthread_stop(irr_keymap_task);
	printk("irremote_remove: irr thread success\n");
	irr_gtimer_exit();
	printk("irremote_remove: iirr_gtimer_exit\n");

	/* Set default mode */
	archos_accessory_set_irmode("default");

	constraint_remove(co_lat_mpu);
	constraint_put(co_lat_mpu);

	return 0;
}

static int irrremote_suspend(struct platform_device *dev, pm_message_t state)
{
	printk("irrremote_suspend\n");
	return 0;
}

static int irrremote_resume(struct platform_device *dev)
{
	printk("irrremote_resume\n");
	return 0;
}

static struct platform_driver irremote_driver = {
	.probe		= irremote_probe,
	.remove		= irremote_remove,
	.suspend	= irrremote_suspend,
	.resume		= irrremote_resume,
	.driver		= {
		.name	= IRR_DEV_NAME,
	},
};

static int __devinit irremote_init_module(void)
{
	return platform_driver_register(&irremote_driver);
}

static void __exit irremote_exit_module(void)
{
	platform_driver_unregister(&irremote_driver);
}


module_init( irremote_init_module );
module_exit( irremote_exit_module );

/* Module information */
MODULE_AUTHOR("jinqin DONG, dong@archos.com");
MODULE_DESCRIPTION("OMAP3430 Archos IrRemote Driver");
MODULE_LICENSE("GPL");
