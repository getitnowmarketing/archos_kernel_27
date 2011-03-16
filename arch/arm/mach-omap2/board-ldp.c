/*
 * linux/arch/arm/mach-omap2/board-ldp.c
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 * Nishant Kamat <nskamat@ti.com>
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl4030.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/board-ldp.h>
#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/keypad.h>
#include <mach/gpmc.h>
#include <mach/hsmmc.h>
#include <mach/usb-musb.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <mach/control.h>
#ifdef CONFIG_OMAP3_PM
#include "prcm-regs.h"
#include "ti-compat.h"
#include <mach/prcm_34xx.h>
#endif

#ifdef CONFIG_VIDEO_OMAP3
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#if defined(CONFIG_VIDEO_OV3640) || defined(CONFIG_VIDEO_OV3640_MODULE)
#include <../drivers/media/video/ov3640.h>
#include <../drivers/media/video/isp/ispcsi2.h>
static	struct omap34xxcam_hw_config *hwc;
#define OV3640_CSI2_CLOCK_POLARITY	0	/* +/- pin order */
#define OV3640_CSI2_DATA0_POLARITY	0	/* +/- pin order */
#define OV3640_CSI2_DATA1_POLARITY	0	/* +/- pin order */
#define OV3640_CSI2_CLOCK_LANE		1	 /* Clock lane position: 1 */
#define OV3640_CSI2_DATA0_LANE		2	 /* Data0 lane position: 2 */
#define OV3640_CSI2_DATA1_LANE		3	 /* Data1 lane position: 3 */
#define OV3640_CSI2_PHY_THS_TERM	4
#define OV3640_CSI2_PHY_THS_SETTLE	14
#define OV3640_CSI2_PHY_TCLK_TERM	0
#define OV3640_CSI2_PHY_TCLK_MISS	1
#define OV3640_CSI2_PHY_TCLK_SETTLE	14
#endif
#endif

#define       SDP3430_SMC91X_CS 	3
#define ENABLE_VAUX1_DEDICATED		0x03
#define ENABLE_VAUX1_DEV_GRP		0x20

#define ENABLE_VAUX3_DEDICATED        	0x03
#define ENABLE_VAUX3_DEV_GRP  		0x20
#define TWL4030_MSECURE_GPIO		22

#define TWL4030_VAUX4_DEV_GRP		0x23
#define TWL4030_VAUX4_DEDICATED		0x26

#ifdef CONFIG_OMAP3_PM
#define CONTROL_SYSC_SMARTIDLE  	(0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE   	(0x1)
#define PRCM_INTERRUPT_MASK     	(1 << 11)
#define UART1_INTERRUPT_MASK    	(1 << 8)
#define UART2_INTERRUPT_MASK    	(1 << 9)
#define UART3_INTERRUPT_MASK    	(1 << 10)
#define TWL4030_MSECURE_GPIO    	22
int console_detect(char *str);
unsigned int uart_interrupt_mask_value;
u32 *console_padconf_reg;
bool console_detected;
#endif

static struct resource ldp_smc911x_resources[] = {
	[0] = {
		.start	= OMAP34XX_ETHR_START,
		.end	= OMAP34XX_ETHR_START + SZ_4K,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device ldp_smc911x_device = {
	.name		= "smc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ldp_smc911x_resources),
	.resource	= ldp_smc911x_resources,
};

#ifdef CONFIG_OMAP3_PM
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

static u32 *uart_detect(void){
	char str[7];

	if (console_detected)
		return console_padconf_reg;

	console_padconf_reg = NULL;
	console_detected = 0;

	if (console_detect(str))
		printk(KERN_INFO"Invalid console paramter\n");

	if (!strcmp(str, "ttyS0")) {
		console_padconf_reg = (u32 *)(&CONTROL_PADCONF_UART1_CTS);
		uart_interrupt_mask_value = UART1_INTERRUPT_MASK;
		}
	else if (!strcmp(str, "ttyS1")) {
		console_padconf_reg = (u32 *)(&CONTROL_PADCONF_UART2_TX);
		uart_interrupt_mask_value = UART2_INTERRUPT_MASK;
		}
	else if (!strcmp(str, "ttyS2")) {
		console_padconf_reg = (u32 *)(&CONTROL_PADCONF_UART3_RTS_SD);
		uart_interrupt_mask_value = UART3_INTERRUPT_MASK;
		}
	else
		printk(KERN_INFO
		"Unable to recongnize Console UART!\n");

	if (console_padconf_reg)
		console_detected = 1;

	return console_padconf_reg;
}

void  init_wakeupconfig(void)
{
	u32 *ptr;
	ptr = uart_detect();
	*ptr |= (u32)((IO_PAD_WAKEUPENABLE | IO_PAD_OFFPULLUDENABLE |
			IO_PAD_OFFOUTENABLE | IO_PAD_OFFENABLE |
			IO_PAD_INPUTENABLE | IO_PAD_PULLUDENABLE)
				<<  IO_PAD_HIGH_SHIFT);

}

bool is_console_wakeup(void)
{
	if ((*uart_detect() >> IO_PAD_HIGH_SHIFT) & IO_PAD_WAKEUPEVENT)
		return 1;
	return 0;
}
void uart_padconf_control(void)
{
	u32 *ptr;
	ptr = (u32 *)uart_detect();
		*ptr |= (u32)((IO_PAD_WAKEUPENABLE)
			<< IO_PAD_HIGH_SHIFT);
}

void setup_board_wakeup_source(u32 wakeup_source)
{
	if ((wakeup_source & PRCM_WAKEUP_T2_KEYPAD) ||
		(wakeup_source & PRCM_WAKEUP_TOUCHSCREEN)) {
		PRCM_GPIO1_SYSCONFIG = 0x15;
		PM_WKEN_WKUP |= 0x8;
		PM_MPUGRPSEL_WKUP = 0x8;
		/* Unmask GPIO interrupt*/
		INTC_MIR_0 = ~((1<<29));
	}
	if (wakeup_source & PRCM_WAKEUP_T2_KEYPAD) {
		CONTROL_PADCONF_SYS_NIRQ &= 0xFFFFFFF8;
		CONTROL_PADCONF_SYS_NIRQ |= 0x4;
		GPIO1_SETIRQENABLE1 |= 0x1;
		GPIO1_SETWKUENA |= 0x1;
		GPIO1_FALLINGDETECT |= 0x1;
	}
	/* Unmasking the PRCM interrupts */
	INTC_MIR_0 &= ~PRCM_INTERRUPT_MASK;
	if (wakeup_source & PRCM_WAKEUP_UART) {
		/* Unmasking the UART interrupts */
		INTC_MIR_2 = ~uart_interrupt_mask_value;
	}
}

static void scm_clk_init(void)
{
	struct clk *p_omap_ctrl_clk = NULL;

	p_omap_ctrl_clk = clk_get(NULL, "omapctrl_ick");
	if (p_omap_ctrl_clk != NULL) {
		if (clk_enable(p_omap_ctrl_clk) != 0) {
			printk(KERN_ERR "failed to enable scm clks\n");
			clk_put(p_omap_ctrl_clk);
		}
	}
	/* Sysconfig set to SMARTIDLE and AUTOIDLE */
	CONTROL_SYSCONFIG = (CONTROL_SYSC_SMARTIDLE | CONTROL_SYSC_AUTOIDLE);
}
#endif

static int ldp_twl4030_keymap[] = {
	KEY(0, 0, KEY_1),
	KEY(1, 0, KEY_2),
	KEY(2, 0, KEY_3),
	KEY(0, 1, KEY_4),
	KEY(1, 1, KEY_5),
	KEY(2, 1, KEY_6),
	KEY(3, 1, KEY_F5),
	KEY(0, 2, KEY_7),
	KEY(1, 2, KEY_8),
	KEY(2, 2, KEY_9),
	KEY(3, 2, KEY_F6),
	KEY(0, 3, KEY_F7),
	KEY(1, 3, KEY_0),
	KEY(2, 3, KEY_F8),
	KEY(5, 4, KEY_MUTE),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(5, 5, KEY_VOLUMEDOWN),
	0
};

static struct twl4030_keypad_data ldp_kp_twl4030_data = {
	.rows		= 6,
	.cols		= 6,
	.keymap		= ldp_twl4030_keymap,
	.keymapsize	= ARRAY_SIZE(ldp_twl4030_keymap),
	.rep		= 1,
	.irq		= TWL4030_MODIRQ_KEYPAD,
};


static struct gpio_keys_button ldp_gpio_keys_buttons[] = {
	[0] = {
		.code		= KEY_ENTER,
		.gpio		= 101,
		.desc		= "enter sw",
		.active_low = 1,
		.debounce_interval = 30,
	},
	[1] = {
		.code		= KEY_F1,
		.gpio		= 102,
		.desc		= "func 1",
		.active_low = 1,
		.debounce_interval = 30,
	},
	[2] = {
		.code		= KEY_F2,
		.gpio		= 103,
		.desc		= "func 2",
		.active_low = 1,
		.debounce_interval = 30,
	},
	[3] = {
		.code		= KEY_F3,
		.gpio		= 104,
		.desc		= "func 3",
		.active_low = 1,
		.debounce_interval = 30,
	},
	[4] = {
		.code		= KEY_F4,
		.gpio		= 105,
		.desc		= "func 4",
		.active_low = 1,
		.debounce_interval = 30,
	},
	[5] = {
		.code		= KEY_LEFT,
		.gpio		= 106,
		.desc		= "left sw",
		.active_low = 1,
		.debounce_interval = 30,
	},
	[6] = {
		.code		= KEY_RIGHT,
		.gpio		= 107,
		.desc		= "right sw",
		.active_low = 1,
		.debounce_interval = 30,
	},
	[7] = {
		.code		= KEY_UP,
		.gpio		= 108,
		.desc		= "up sw",
		.active_low = 1,
		.debounce_interval = 30,
	},
	[8] = {
		.code		= KEY_DOWN,
		.gpio		= 109,
		.desc		= "down sw",
		.active_low = 1,
		.debounce_interval = 30,
	},
};

static struct gpio_keys_platform_data ldp_gpio_keys = {
	.buttons		= ldp_gpio_keys_buttons,
	.nbuttons		= ARRAY_SIZE(ldp_gpio_keys_buttons),
	.rep			= 1,
};

static struct platform_device ldp_gpio_keys_device = {
	.name		= "gpio-keys",
	.id			= -1,
	.dev		= {
		.platform_data	= &ldp_gpio_keys,
	},
};


static int ts_gpio;

static int __init msecure_init(void)
{
	int ret = 0;

#ifdef CONFIG_RTC_DRV_TWL4030
	/* 3430ES2.0 doesn't have msecure/gpio-22 line connected to T2 */
	if (omap_type() == OMAP2_DEVICE_TYPE_GP &&
			system_rev < OMAP3430_REV_ES2_0) {
		void __iomem *msecure_pad_config_reg =
			omap_ctrl_base_get() + 0xA3C;
		int mux_mask = 0x04;
		u16 tmp;

		ret = gpio_request(TWL4030_MSECURE_GPIO, "msecure");
		if (ret < 0) {
			printk(KERN_ERR "msecure_init: can't"
				"reserve GPIO:%d !\n", TWL4030_MSECURE_GPIO);
			goto out;
		}
		/*
		 * TWL4030 will be in secure mode if msecure line from OMAP
		 * is low. Make msecure line high in order to change the
		 * TWL4030 RTC time and calender registers.
		 */

		tmp = __raw_readw(msecure_pad_config_reg);
		tmp &= 0xF8;	/* To enable mux mode 03/04 = GPIO_RTC */
		tmp |= mux_mask;/* To enable mux mode 03/04 = GPIO_RTC */
		__raw_writew(tmp, msecure_pad_config_reg);

		gpio_direction_output(TWL4030_MSECURE_GPIO, 1);
	}
out:
#endif
	return ret;
}

/**
 * @brief ads7846_dev_init : Requests & sets GPIO line for pen-irq
 *
 * @return - void. If request gpio fails then Flag KERN_ERR.
 */
static void ads7846_dev_init(void)
{
	if (omap_request_gpio(ts_gpio) < 0) {
		printk(KERN_ERR "can't get ads746 pen down GPIO\n");
		return;
	}

	omap_set_gpio_direction(ts_gpio, 1);

	omap_set_gpio_debounce(ts_gpio, 1);
	omap_set_gpio_debounce_time(ts_gpio, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !omap_get_gpio_datain(ts_gpio);
}

/*
 * This enable(1)/disable(0) the voltage for TS: uses twl4030 calls
 */
static int ads7846_vaux_control(int vaux_cntrl)
{
	int ret = 0;

#ifdef CONFIG_TWL4030_CORE
	/* check for return value of ldo_use: if success it returns 0 */
	if (vaux_cntrl == VAUX_ENABLE) {
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX1_DEDICATED, TWL4030_VAUX1_DEDICATED))
			return -EIO;
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX1_DEV_GRP, TWL4030_VAUX1_DEV_GRP))
			return -EIO;
	} else if (vaux_cntrl == VAUX_DISABLE) {
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			0x00, TWL4030_VAUX1_DEDICATED))
			return -EIO;
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			0x00, TWL4030_VAUX1_DEV_GRP))
			return -EIO;
	}
#else
	ret = -EIO;
#endif
	return ret;
}

static struct ads7846_platform_data tsc2046_config __initdata = {
#ifdef CONFIG_ADS_SCALED_EV
	.x_min			= 310,
	.y_min			= 217,
	.x_max			= 3716,
	.y_max			= 3782,
#else
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
#endif
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 20,
	.debounce_tol		= 10,
	.debounce_rep		= 1,
 	.get_pendown_state	= ads7846_get_pendown_state,
 	.keep_vref_on		= 1,
 	.vaux_control		= ads7846_vaux_control,
	.settle_delay_usecs	= 100,
};


static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

#ifdef CONFIG_FB_OMAP_LCD_WVGA
static struct omap2_mcspi_device_config wvgalcd_mcspi_config = {
	.turbo_mode		= 0,
	.single_channel		= 1,  /* 0: slave, 1: master */
};
#endif

static struct spi_board_info ldp_spi_board_info[] __initdata = {
	[0] = {
		/*
		 * TSC2046 operates at a max freqency of 2MHz, so
		 * operate slightly below at 1.5MHz
		 */
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &tsc2046_mcspi_config,
		.irq			= 0,
		.platform_data		= &tsc2046_config,
	},
#ifdef CONFIG_FB_OMAP_LCD_WVGA
	[1] = {
		.modalias		= "wvgalcd",
		.bus_num		= 1,
		.chip_select		= 2,
		.max_speed_hz		= 375000,
		.controller_data 	= &wvgalcd_mcspi_config,
	},
#endif
};

static struct platform_device *ldp_devices[] __initdata = {
       &ldp_smc911x_device,
       &ldp_gpio_keys_device,
};

static inline void __init ldp_init_smc911x(void)
{
       int eth_cs;
       unsigned long cs_mem_base;
       int eth_gpio = 0;

       eth_cs = LDP_SMC911X_CS;

       if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
               printk(KERN_ERR "Failed to request GPMC mem for smc911x\n");
               return;
       }

       ldp_smc911x_resources[0].start = cs_mem_base + 0x0;
       ldp_smc911x_resources[0].end   = cs_mem_base + 0xf;
       udelay(100);

       eth_gpio = LDP_SMC911X_GPIO;

       ldp_smc911x_resources[1].start = OMAP_GPIO_IRQ(eth_gpio);

       if (omap_request_gpio(eth_gpio) < 0) {
               printk(KERN_ERR "Failed to request GPIO%d for smc911x IRQ\n",
                       eth_gpio);
               return;
       }
       omap_set_gpio_direction(eth_gpio, 1);
}


static void __init omap_ldp_init_irq(void)
{
	omap2_init_common_hw(NULL);
	omap_init_irq();
#ifdef CONFIG_OMAP3_PM
	/* System Control module clock initialization */
	scm_clk_init();
#endif
	omap_gpio_init();
        ldp_init_smc911x();
}

static struct omap_uart_config ldp_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_board_config_kernel ldp_config[] __initdata = {
	{ OMAP_TAG_UART,	&ldp_uart_config },
};

static int ldp_batt_table[] = {
/* 0 C*/
30800, 29500, 28300, 27100,
26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
11600, 11200, 10800, 10400, 10000, 9630,   9280,   8950,   8620,   8310,
8020,   7730,   7460,   7200,   6950,   6710,   6470,   6250,   6040,   5830,
5640,   5450,   5260,   5090,   4920,   4760,   4600,   4450,   4310,   4170,
4040,   3910,   3790,   3670,   3550
};

static struct twl4030_bci_platform_data ldp_bci_data = {
	.battery_tmp_tbl	= ldp_batt_table,
	.tblsize		= ARRAY_SIZE(ldp_batt_table),
};

static struct twl4030_usb_data ldp_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_gpio_platform_data ldp_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
};

static struct twl4030_madc_platform_data ldp_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data ldp_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci		= &ldp_bci_data,
	.madc		= &ldp_madc_data,
	.usb		= &ldp_usb_data,
	.gpio		= &ldp_gpio_data,
	.keypad		= &ldp_kp_twl4030_data,
};

#if defined(CONFIG_VIDEO_OV3640) || defined(CONFIG_VIDEO_OV3640_MODULE)

static struct omap34xxcam_sensor_config ov3640_hwc = {
	.sensor_isp = 0,
#if defined(CONFIG_VIDEO_OV3640_CSI2)
	.xclk = OMAP34XXCAM_XCLK_B,
#else
	.xclk = OMAP34XXCAM_XCLK_A,
#endif
	.capture_mem = 2592 * 1944 * 2 * 2,
};

static struct isp_interface_config ov3640_if_config = {
	.ccdc_par_ser = ISP_CSIA,
	.dataline_shift = 0x0,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.vdint0_timing = 0x0,
	.vdint1_timing = 0x0,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.wenlog = ISPCCDC_CFG_WENLOG_AND,
	.dcsub = OV3640_BLACK_LEVEL_10BIT,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_BG_GR,
	.wbal.coef0 = 0x23,
	.wbal.coef1 = 0x20,
	.wbal.coef2 = 0x20,
	.wbal.coef3 = 0x30,
	.u.csi.crc = 0x0,
	.u.csi.mode = 0x0,
	.u.csi.edge = 0x0,
	.u.csi.signalling = 0x0,
	.u.csi.strobe_clock_inv = 0x0,
	.u.csi.vs_edge = 0x0,
	.u.csi.channel = 0x1,
	.u.csi.vpclk = 0x1,
	.u.csi.data_start = 0x0,
	.u.csi.data_size = 0x0,
	.u.csi.format = V4L2_PIX_FMT_SGRBG10,
};

static int ov3640_sensor_set_prv_data(void *priv)
{

	hwc = priv;
	hwc->u.sensor.xclk = ov3640_hwc.xclk;
	hwc->u.sensor.sensor_isp = ov3640_hwc.sensor_isp;
	hwc->dev_index = 1;
	hwc->dev_minor = 4;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	hwc->interface_type = ISP_CSIA;

#if defined(CONFIG_VIDEO_OV3640_CSI2)
	hwc->csi2.hw_csi2.lanes.clock.polarity = OV3640_CSI2_CLOCK_POLARITY;
	hwc->csi2.hw_csi2.lanes.clock.position = OV3640_CSI2_CLOCK_LANE;
	hwc->csi2.hw_csi2.lanes.data[0].polarity = OV3640_CSI2_DATA0_POLARITY;
	hwc->csi2.hw_csi2.lanes.data[0].position = OV3640_CSI2_DATA0_LANE;
	hwc->csi2.hw_csi2.lanes.data[1].polarity = OV3640_CSI2_DATA1_POLARITY;
	hwc->csi2.hw_csi2.lanes.data[1].position = OV3640_CSI2_DATA1_LANE;
	hwc->csi2.hw_csi2.phy.ths_term = OV3640_CSI2_PHY_THS_TERM;
	hwc->csi2.hw_csi2.phy.ths_settle = OV3640_CSI2_PHY_THS_SETTLE;
	hwc->csi2.hw_csi2.phy.tclk_term = OV3640_CSI2_PHY_TCLK_TERM;
	hwc->csi2.hw_csi2.phy.tclk_miss = OV3640_CSI2_PHY_TCLK_MISS;
	hwc->csi2.hw_csi2.phy.tclk_settle = OV3640_CSI2_PHY_TCLK_SETTLE;
#endif

	return 0;
}

static int ov3640_sensor_power_set(enum v4l2_power power)
{
	struct isp_csi2_lanes_cfg lanecfg;
	struct isp_csi2_phy_cfg phyconfig;
	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	switch (power) {
	case V4L2_POWER_ON:
		printk(KERN_DEBUG "ov3640_sensor_power_set(ON)\n");
		isp_csi2_reset();

		lanecfg.clk.pol = OV3640_CSI2_CLOCK_POLARITY;
		lanecfg.clk.pos = OV3640_CSI2_CLOCK_LANE;
		lanecfg.data[0].pol = OV3640_CSI2_DATA0_POLARITY;
		lanecfg.data[0].pos = OV3640_CSI2_DATA0_LANE;
		lanecfg.data[1].pol = OV3640_CSI2_DATA1_POLARITY;
		lanecfg.data[1].pos = OV3640_CSI2_DATA1_LANE;
		lanecfg.data[2].pol = 0;
		lanecfg.data[2].pos = 0;
		lanecfg.data[3].pol = 0;
		lanecfg.data[3].pos = 0;
		isp_csi2_complexio_lanes_config(&lanecfg);
		isp_csi2_complexio_lanes_update(true);

		phyconfig.ths_term = OV3640_CSI2_PHY_THS_TERM;
		phyconfig.ths_settle = OV3640_CSI2_PHY_THS_SETTLE;
		phyconfig.tclk_term = OV3640_CSI2_PHY_TCLK_TERM;
		phyconfig.tclk_miss = OV3640_CSI2_PHY_TCLK_MISS;
		phyconfig.tclk_settle = OV3640_CSI2_PHY_TCLK_SETTLE;
		isp_csi2_phy_config(&phyconfig);
		isp_csi2_phy_update(true);

		isp_configure_interface(&ov3640_if_config);

		if (previous_power == V4L2_POWER_OFF) {
#ifdef CONFIG_TWL4030_CORE
			/* turn on analog power */
#if defined(CONFIG_VIDEO_OV3640_CSI2)
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_1_8_V, TWL4030_VAUX4_DEDICATED);
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX4_DEV_GRP);
#else
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_2_8_V, TWL4030_VAUX2_DEDICATED);
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX2_DEV_GRP);
#endif
			udelay(100);
#else
#error "no power companion board defined!"
#endif
			/* Request and configure gpio pins */
			if (omap_request_gpio(OV3640_RESET_GPIO) != 0) {
				printk(KERN_ERR "Could not request GPIO %d",
					OV3640_RESET_GPIO);
				return -EIO;
			}
			if (omap_request_gpio(OV3640_STANDBY_GPIO) != 0) {
				printk(KERN_ERR "Could not request GPIO %d",
					OV3640_STANDBY_GPIO);
				return -EIO;
			}
			/* set to output mode */
			omap_set_gpio_direction(OV3640_RESET_GPIO, 0);
			omap_set_gpio_direction(OV3640_STANDBY_GPIO, 0);

			/* Turn ON Omnivision sensor */
			omap_set_gpio_dataout(OV3640_RESET_GPIO, 1);
			omap_set_gpio_dataout(OV3640_STANDBY_GPIO, 0);
			udelay(100);

			/* RESET Omnivision sensor */
			omap_set_gpio_dataout(OV3640_RESET_GPIO, 0);
			udelay(100);
			omap_set_gpio_dataout(OV3640_RESET_GPIO, 1);
		}
		break;
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "ov3640_sensor_power_set(OFF)\n");
		/* Power Down Sequence */

#ifdef CONFIG_TWL4030_CORE
#if defined(CONFIG_VIDEO_OV3640_CSI2)
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX4_DEV_GRP);
#else
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX2_DEV_GRP);
#endif
#else
#error "no power companion board defined!"
#endif
		omap_free_gpio(OV3640_RESET_GPIO);
		omap_free_gpio(OV3640_STANDBY_GPIO);
		break;
	case V4L2_POWER_STANDBY:
		printk(KERN_DEBUG "ov3640_sensor_power_set(STANDBY)\n");
		break;
	}
	previous_power = power;
	return 0;
}

static struct ov3640_platform_data sdp3430_ov3640_platform_data = {
	.power_set      = ov3640_sensor_power_set,
	.priv_data_set  = ov3640_sensor_set_prv_data,
	.default_regs   = ov3640_common[0],
};

#endif

static struct i2c_board_info __initdata ldp_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &ldp_twldata,
	},
};

static struct i2c_board_info __initdata ldp_i2c_boardinfo_2[] = {
#if defined(CONFIG_VIDEO_OV3640) || defined(CONFIG_VIDEO_OV3640_MODULE)
	{
		I2C_BOARD_INFO("ov3640", OV3640_I2C_ADDR),
		.platform_data = &sdp3430_ov3640_platform_data,
	},
#endif
};

static int __init omap_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, ldp_i2c_boardinfo,
			ARRAY_SIZE(ldp_i2c_boardinfo));
	omap_register_i2c_bus(2, 2600, ldp_i2c_boardinfo_2,
			ARRAY_SIZE(ldp_i2c_boardinfo_2));
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static void __init omap_ldp_init(void)
{
	omap_i2c_init();
	platform_add_devices(ldp_devices, ARRAY_SIZE(ldp_devices));
	omap_board_config = ldp_config;
	omap_board_config_size = ARRAY_SIZE(ldp_config);
	ts_gpio = 54;
	ldp_spi_board_info[0].irq = OMAP_GPIO_IRQ(ts_gpio);
	spi_register_board_info(ldp_spi_board_info,
				ARRAY_SIZE(ldp_spi_board_info));

#ifdef CONFIG_OMAP3_PM
	prcm_init();
#endif
	msecure_init();
	ads7846_dev_init();
	ldp_flash_init();
	omap_serial_init();
	usb_musb_init();
	hsmmc_init();
}

static void __init omap_ldp_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP_LDP, "OMAP LDP board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_ldp_map_io,
	.init_irq	= omap_ldp_init_irq,
	.init_machine	= omap_ldp_init,
	.timer		= &omap_timer,
MACHINE_END
