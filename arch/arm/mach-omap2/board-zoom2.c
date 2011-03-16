/*
 * linux/arch/arm/mach-omap2/board-zoom2.c
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 * Vikram Pandita <vikram.pandita@ti.com>
 *
 * Modified from mach-omap2/board-ldp.c
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
#include <linux/synaptics_i2c_rmi.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl4030.h>
#include <linux/mm.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/board-zoom2.h>
#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/keypad.h>
#include <mach/gpmc.h>
#include <mach/hsmmc.h>
#include <mach/usb-musb.h>
#include <mach/mux.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <mach/control.h>
#include <mach/mux.h>
#ifdef CONFIG_OMAP3_PM
#include "prcm-regs.h"
#include "ti-compat.h"
#include <mach/prcm_34xx.h>
#endif

#if defined(CONFIG_PDA_POWER) && defined(CONFIG_TWL4030_BCI_BATTERY)
#include <linux/pda_power.h>
#endif

#ifdef CONFIG_SPI_IDCC
#include <linux/spi/idcc6071.h>
#endif

#ifdef CONFIG_VIDEO_OMAP3
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#if defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)
#include <../drivers/media/video/imx046.h>
#include <../drivers/media/video/isp/ispcsi2.h>
#define IMX046_CSI2_CLOCK_POLARITY	0	/* +/- pin order */
#define IMX046_CSI2_DATA0_POLARITY	0	/* +/- pin order */
#define IMX046_CSI2_DATA1_POLARITY	0	/* +/- pin order */
#define IMX046_CSI2_CLOCK_LANE		1	 /* Clock lane position: 1 */
#define IMX046_CSI2_DATA0_LANE		2	 /* Data0 lane position: 2 */
#define IMX046_CSI2_DATA1_LANE		3	 /* Data1 lane position: 3 */
#define IMX046_CSI2_PHY_THS_TERM	2
#define IMX046_CSI2_PHY_THS_SETTLE	23
#define IMX046_CSI2_PHY_TCLK_TERM	0
#define IMX046_CSI2_PHY_TCLK_MISS	1
#define IMX046_CSI2_PHY_TCLK_SETTLE	14
#ifndef CONFIG_TWL4030_CORE
#error "no power companion board defined!"
#endif
#endif
#endif

#ifdef CONFIG_VIDEO_LV8093
#include <../drivers/media/video/lv8093.h>
#define LV8093_PS_GPIO			7
/* GPIO7 is connected to lens PS pin through inverter */
#define LV8093_PWR_OFF			1
#define LV8093_PWR_ON			(!LV8093_PWR_OFF)
#endif

#define OMAP_SYNAPTICS_GPIO		163

#define SDP3430_SMC91X_CS		3
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

#ifdef CONFIG_WL127X_POWER
static int wl127x_gpios[] = {
	109,    /* Bluetooth Enable GPIO */
	161,    /* FM Enable GPIO */
	61,     /* BT Active LED */
};

static struct platform_device ldp_wl127x_device = {
	.name           = "wl127x",
	.id             = -1,
	.dev.platform_data = &wl127x_gpios,
};
#endif

#if defined(CONFIG_PDA_POWER) && defined(CONFIG_TWL4030_BCI_BATTERY)
static char *twl4030_supplicants[] = {
	"bq27000-battery",
};

static struct pda_power_pdata ac_usb_power_info = {
	.is_ac_online    = twl4030_is_ac_online,
	.is_usb_online   = twl4030_is_usb_online,
	.supplied_to     = twl4030_supplicants,
	.num_supplicants = ARRAY_SIZE(twl4030_supplicants),
};

static struct platform_device ac_usb_power = {
	.name = "pda-power",
	.id   = -1,
	.dev  = {
		.platform_data = &ac_usb_power_info,
	},
};
#endif

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
		printk(KERN_INFO "Invalid console paramter\n");

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
	else if (!strcmp(str, "ttyS3")) {
		/* No quart wakeup yet */
		console_padconf_reg = NULL;
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
	char str[7];

	if (console_detect(str))
		printk(KERN_INFO"Invalid console paramter\n");

	ptr = uart_detect();
	if (ptr != NULL)
		*ptr |= (u32)((IO_PAD_WAKEUPENABLE | IO_PAD_OFFPULLUDENABLE |
			IO_PAD_OFFOUTENABLE | IO_PAD_OFFENABLE |
			IO_PAD_INPUTENABLE | IO_PAD_PULLUDENABLE)
				<<  IO_PAD_HIGH_SHIFT);
	else
		printk(KERN_INFO "IO Pad wakeup capability not configured \n");

}

bool is_console_wakeup(void)
{
	char str[7];
	u32 *ptr;

	if (console_detect(str))
		printk(KERN_INFO"Invalid console paramter\n");

	ptr = (u32 *)uart_detect();
	if (ptr != NULL) {
		if ((*uart_detect() >> IO_PAD_HIGH_SHIFT) & IO_PAD_WAKEUPEVENT)
			return 1;
	}
	return 0;
}

void uart_padconf_control(void)
{
	u32 *ptr;
	char str[7];

	if (console_detect(str))
		printk(KERN_INFO"Invalid console paramter\n");

	ptr = (u32 *)uart_detect();
	if (ptr != NULL)
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

/* Zoom2 has Qwerty keyboard*/
static int ldp_twl4030_keymap[] = {
	KEY(0, 0, KEY_E),
	KEY(1, 0, KEY_R),
	KEY(2, 0, KEY_T),
	KEY(3, 0, KEY_HOME),
	KEY(6, 0, KEY_I),
	KEY(7, 0, KEY_LEFTSHIFT),
	KEY(0, 1, KEY_D),
	KEY(1, 1, KEY_F),
	KEY(2, 1, KEY_G),
	KEY(3, 1, KEY_SEND),
	KEY(6, 1, KEY_K),
	KEY(7, 1, KEY_ENTER),
	KEY(0, 2, KEY_X),
	KEY(1, 2, KEY_C),
	KEY(2, 2, KEY_V),
	KEY(3, 2, KEY_END),
	KEY(6, 2, KEY_DOT),
	KEY(7, 2, KEY_CAPSLOCK),
	KEY(0, 3, KEY_Z),
	KEY(1, 3, KEY_KPPLUS),
	KEY(2, 3, KEY_B),
	KEY(3, 3, KEY_F1),
	KEY(6, 3, KEY_O),
	KEY(7, 3, KEY_SPACE),
	KEY(0, 4, KEY_W),
	KEY(1, 4, KEY_Y),
	KEY(2, 4, KEY_U),
	KEY(3, 4, KEY_F2),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(6, 4, KEY_L),
	KEY(7, 4, KEY_LEFT),
	KEY(0, 5, KEY_S),
	KEY(1, 5, KEY_H),
	KEY(2, 5, KEY_J),
	KEY(3, 5, KEY_F3),
	KEY(5, 5, KEY_VOLUMEDOWN),
	KEY(6, 5, KEY_M),
	KEY(4, 5, KEY_ENTER),
	KEY(7, 5, KEY_RIGHT),
	KEY(0, 6, KEY_Q),
	KEY(1, 6, KEY_A),
	KEY(2, 6, KEY_N),
	KEY(3, 6, KEY_BACKSPACE),
	KEY(6, 6, KEY_P),
	KEY(7, 6, KEY_UP),
	KEY(6, 7, KEY_SELECT),
	KEY(7, 7, KEY_DOWN),
	KEY(0, 7, KEY_PROG1),	/*MACRO 1 <User defined> */
	KEY(1, 7, KEY_PROG2),	/*MACRO 2 <User defined> */
	KEY(2, 7, KEY_PROG3),	/*MACRO 3 <User defined> */
	KEY(3, 7, KEY_PROG4),	/*MACRO 4 <User defined> */
	0
};

static struct twl4030_keypad_data ldp_kp_twl4030_data = {
	.rows		= 8,
	.cols		= 8,
	.keymap		= ldp_twl4030_keymap,
	.keymapsize	= ARRAY_SIZE(ldp_twl4030_keymap),
	.rep		= 1,
	.irq		= TWL4030_MODIRQ_KEYPAD,
};

#ifdef CONFIG_SPI_IDCC
static int idcc_srdy_gpio;

typedef enum gpio_direction {
	GPIO_OUTPUT = 0,
	GPIO_INPUT,
}gpio_direction_t;

typedef enum GPIO_INIT {
	GPIO_INIT_PULLUP = 0,
	GPIO_INIT_PULLDOWN,
	GPIO_INIT_NONE,
}gpio_init_t;

static void idcc6071_configure_gpio(int gpio,
			            gpio_direction_t direction,
				    gpio_init_t      init_value)
{
    printk("IDCC6071_CFG_GPIO Configuring GPIO\n");
    if(omap_request_gpio(gpio) < 0) {
           printk(KERN_ERR "can't get %d gpio\n", gpio);
	   return;
    }

    if ((direction != GPIO_INPUT) && (direction != GPIO_OUTPUT)) {
          printk(KERN_ERR "error direction %d gpio %d\n", direction, gpio);
          return;
    }

    printk("IDCC6071_CFG_GPIO Configuring GPIO %d Dir %s\n", gpio,
							     (direction == 0)?"Out":"In");
    omap_set_gpio_direction(gpio, direction);

    // Pull up GPIO
    if (init_value == GPIO_INIT_PULLUP)
	omap_set_gpio_dataout(gpio, 1);
    else if (init_value == GPIO_INIT_PULLDOWN)
        omap_set_gpio_dataout(gpio, 0);

    return;
}

static void idcc6071_dev_init(void)
{
      printk("IDCC6071_DEV_INIT\n");

      omap_cfg_reg(AA3_3430_MCSPI2_CLK);
      omap_cfg_reg(Y2_3430_MCSPI2_SIMO);
      omap_cfg_reg(Y3_3430_MCSPI2_SOMI);
      omap_cfg_reg(Y4_3430_MCSPI2_CS0);
      omap_cfg_reg(Y5_3430_MCSPI2_CS1);
      omap_cfg_reg(MRDY_CFG_REG);     /* configure MRDY */
      omap_cfg_reg(SRDY_CFG_REG);     /* configure SRDY */
      omap_cfg_reg(A23_3430_GPIO_95);
      omap_cfg_reg(D24_3430_GPIO_103);
      omap_cfg_reg(C26_3430_GPIO_110);

      if (omap_request_gpio(idcc_srdy_gpio) < 0) {
              printk(KERN_ERR "can't get SRDY GPIO\n");
              return;
      }

      omap_set_gpio_direction(idcc_srdy_gpio, 1);

      if (omap_request_gpio(SPI_GPIO_MRDY) < 0) {
              printk(KERN_ERR "can't get MRDY GPIO\n");
              return;
       }
       omap_set_gpio_direction(SPI_GPIO_MRDY, 0);

       //idcc6071_configure_gpio(MODEM_GPIO_AUDIO, GPIO_OUTPUT, GPIO_INIT_PULLUP);
       //idcc6071_configure_gpio(MODEM_GPIO_RESET, GPIO_OUTPUT, GPIO_INIT_PULLUP);
       //idcc6071_configure_gpio(MODEM_GPIO_PWRON, GPIO_OUTPUT, GPIO_INIT_PULLUP);
}

#endif

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

#ifdef CONFIG_SPI_IDCC
static struct omap2_mcspi_device_config idcc6071_mcspi_config = {
       .turbo_mode             = 0,
       .single_channel         = 1,  /* 0: slave, 1: master */
};
#endif

static struct omap2_mcspi_device_config wvgalcd_mcspi_config = {
	.turbo_mode		= 0,
	.single_channel		= 1,  /* 0: slave, 1: master */
};

static struct spi_board_info ldp_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "wvgalcd",
		.bus_num		= 1,
		.chip_select		= 2,
		.max_speed_hz		= 375000,
		.controller_data 	= &wvgalcd_mcspi_config,
	},
#ifdef CONFIG_SPI_IDCC
        {
               .modalias               = "idcc6071",
               .bus_num                = 2,
               .chip_select            = 0,
               .max_speed_hz           = 3000*1000, //* 3MHz for now, later: *//       = 8000*1000, // 8MHz
               .controller_data        = &idcc6071_mcspi_config,
	       .irq                    = 0,
        }
#endif
};

static struct platform_device *ldp_devices[] __initdata = {
#ifdef CONFIG_WL127X_POWER
       &ldp_wl127x_device,
#endif
#if defined(CONFIG_PDA_POWER) && defined(CONFIG_TWL4030_BCI_BATTERY)
       &ac_usb_power,
#endif
};

static void __init omap_ldp_init_irq(void)
{
	omap2_init_common_hw(NULL);
	omap_init_irq();
#ifdef CONFIG_OMAP3_PM
	/* System Control module clock initialization */
	scm_clk_init();
#endif
	omap_gpio_init();
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

#ifdef CONFIG_VIDEO_LV8093
static int lv8093_lens_power_set(enum v4l2_power power)
{
	static enum v4l2_power previous_pwr = V4L2_POWER_OFF;

	switch (power) {
	case V4L2_POWER_ON:
		printk(KERN_DEBUG "lv8093_lens_power_set(ON)\n");
		if (previous_pwr == V4L2_POWER_OFF) {
			if (omap_request_gpio(LV8093_PS_GPIO) != 0) {
				printk(KERN_WARNING "Could not request GPIO %d"
					" for LV8093\n", LV8093_PS_GPIO);
				return -EIO;
			}

			omap_set_gpio_dataout(LV8093_PS_GPIO, LV8093_PWR_OFF);
			omap_set_gpio_direction(LV8093_PS_GPIO,
					GPIO_DIR_OUTPUT);
		}
		omap_set_gpio_dataout(LV8093_PS_GPIO, LV8093_PWR_ON);
		break;
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "lv8093_lens_power_set(OFF)\n");
		omap_free_gpio(LV8093_PS_GPIO);
		break;
	case V4L2_POWER_STANDBY:
		printk(KERN_DEBUG "lv8093_lens_power_set(STANDBY)\n");
		omap_set_gpio_dataout(LV8093_PS_GPIO, LV8093_PWR_OFF);
		break;
	}
	previous_pwr = power;
	return 0;
}

static int lv8093_lens_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->dev_index = 2;
	hwc->dev_minor = 5;
	hwc->dev_type = OMAP34XXCAM_SLAVE_LENS;
	hwc->interface_type = ISP_CSIA;
	return 0;
}

static struct lv8093_platform_data zoom2_lv8093_platform_data = {
	.power_set      = lv8093_lens_power_set,
	.priv_data_set  = lv8093_lens_set_prv_data,
};
#endif

#if defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)

static struct omap34xxcam_sensor_config imx046_hwc = {
	.sensor_isp  = 0,
	.xclk        = OMAP34XXCAM_XCLK_B,
	.capture_mem = PAGE_ALIGN(3280 * 2464 * 2) * 2,
};

static int imx046_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.xclk	= imx046_hwc.xclk;
	hwc->u.sensor.sensor_isp = imx046_hwc.sensor_isp;
	hwc->dev_index		= 2;
	hwc->dev_minor		= 5;
	hwc->dev_type		= OMAP34XXCAM_SLAVE_SENSOR;
	hwc->interface_type	= ISP_CSIA;

	hwc->csi2.hw_csi2.lanes.clock.polarity   = IMX046_CSI2_CLOCK_POLARITY;
	hwc->csi2.hw_csi2.lanes.clock.position   = IMX046_CSI2_CLOCK_LANE;
	hwc->csi2.hw_csi2.lanes.data[0].polarity = IMX046_CSI2_DATA0_POLARITY;
	hwc->csi2.hw_csi2.lanes.data[0].position = IMX046_CSI2_DATA0_LANE;
	hwc->csi2.hw_csi2.lanes.data[1].polarity = IMX046_CSI2_DATA1_POLARITY;
	hwc->csi2.hw_csi2.lanes.data[1].position = IMX046_CSI2_DATA1_LANE;
	hwc->csi2.hw_csi2.phy.ths_term    = IMX046_CSI2_PHY_THS_TERM;
	hwc->csi2.hw_csi2.phy.ths_settle  = IMX046_CSI2_PHY_THS_SETTLE;
	hwc->csi2.hw_csi2.phy.tclk_term   = IMX046_CSI2_PHY_TCLK_TERM;
	hwc->csi2.hw_csi2.phy.tclk_miss   = IMX046_CSI2_PHY_TCLK_MISS;
	hwc->csi2.hw_csi2.phy.tclk_settle = IMX046_CSI2_PHY_TCLK_SETTLE;
	return 0;
}

static struct isp_interface_config imx046_if_config = {
	.ccdc_par_ser 		= ISP_CSIA,
	.dataline_shift 	= 0x0,
	.hsvs_syncdetect 	= ISPCTRL_SYNC_DETECT_VSRISE,
	.vdint0_timing 		= 0x0,
	.vdint1_timing 		= 0x0,
	.strobe 		= 0x0,
	.prestrobe 		= 0x0,
	.shutter 		= 0x0,
	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,
	.dcsub 			= IMX046_BLACK_LEVEL_AVG,
	.raw_fmt_in		= ISPCCDC_INPUT_FMT_RG_GB,
	.wbal.coef0		= 0x23,
	.wbal.coef1		= 0x20,
	.wbal.coef2		= 0x20,
	.wbal.coef3		= 0x39,
	.u.csi.crc 		= 0x0,
	.u.csi.mode 		= 0x0,
	.u.csi.edge 		= 0x0,
	.u.csi.signalling 	= 0x0,
	.u.csi.strobe_clock_inv = 0x0,
	.u.csi.vs_edge 		= 0x0,
	.u.csi.channel 		= 0x0,
	.u.csi.vpclk 		= 0x2,
	.u.csi.data_start 	= 0x0,
	.u.csi.data_size 	= 0x0,
	.u.csi.format 		= V4L2_PIX_FMT_SGRBG10,
};


static int imx046_sensor_power_set(enum v4l2_power power)
{
	struct isp_csi2_lanes_cfg lanecfg;
	struct isp_csi2_phy_cfg phyconfig;
	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	int err = 0;

	switch (power) {
	case V4L2_POWER_ON:
		/* Power Up Sequence */
		printk(KERN_DEBUG "imx046_sensor_power_set(ON)\n");
		isp_csi2_reset();

		lanecfg.clk.pol = IMX046_CSI2_CLOCK_POLARITY;
		lanecfg.clk.pos = IMX046_CSI2_CLOCK_LANE;
		lanecfg.data[0].pol = IMX046_CSI2_DATA0_POLARITY;
		lanecfg.data[0].pos = IMX046_CSI2_DATA0_LANE;
		lanecfg.data[1].pol = IMX046_CSI2_DATA1_POLARITY;
		lanecfg.data[1].pos = IMX046_CSI2_DATA1_LANE;
		lanecfg.data[2].pol = 0;
		lanecfg.data[2].pos = 0;
		lanecfg.data[3].pol = 0;
		lanecfg.data[3].pos = 0;
		isp_csi2_complexio_lanes_config(&lanecfg);
		isp_csi2_complexio_lanes_update(true);

		isp_csi2_ctrl_config_ecc_enable(true);

		phyconfig.ths_term = IMX046_CSI2_PHY_THS_TERM;
		phyconfig.ths_settle = IMX046_CSI2_PHY_THS_SETTLE;
		phyconfig.tclk_term = IMX046_CSI2_PHY_TCLK_TERM;
		phyconfig.tclk_miss = IMX046_CSI2_PHY_TCLK_MISS;
		phyconfig.tclk_settle = IMX046_CSI2_PHY_TCLK_SETTLE;
		isp_csi2_phy_config(&phyconfig);
		isp_csi2_phy_update(true);

		isp_configure_interface(&imx046_if_config);

		if (previous_power == V4L2_POWER_OFF) {
			/* Request and configure gpio pins */
			if (omap_request_gpio(IMX046_RESET_GPIO) != 0)
				return -EIO;

			/* nRESET is active LOW. set HIGH to release reset */
			omap_set_gpio_dataout(IMX046_RESET_GPIO, 1);

			/* set to output mode */
			omap_set_gpio_direction(IMX046_RESET_GPIO,
				GPIO_DIR_OUTPUT);

			/* turn on analog power */
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_2_8_V, TWL4030_VAUX2_DEDICATED);
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX2_DEV_GRP);

			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_1_8_V, TWL4030_VAUX4_DEDICATED);
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX4_DEV_GRP);
			udelay(100);

			/* have to put sensor to reset to guarantee detection */
			omap_set_gpio_dataout(IMX046_RESET_GPIO, 0);
			udelay(1500);

			/* nRESET is active LOW. set HIGH to release reset */
			omap_set_gpio_dataout(IMX046_RESET_GPIO, 1);
			udelay(300);
		}
		break;
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "imx046_sensor_power_set(OFF)\n");
		/* Power Down Sequence */
		isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);

		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX4_DEV_GRP);
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX2_DEV_GRP);
		omap_free_gpio(IMX046_RESET_GPIO);
		break;
	case V4L2_POWER_STANDBY:
		printk(KERN_DEBUG "imx046_sensor_power_set(STANDBY)\n");
		/*TODO*/
		break;
	}

	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return err;
}

static struct imx046_platform_data zoom2_imx046_platform_data = {
	.power_set            = imx046_sensor_power_set,
	.priv_data_set        = imx046_sensor_set_prv_data,
	.default_regs         = NULL,
	.set_xclk             = isp_set_xclk,
	.cfg_interface_bridge = isp_configure_interface_bridge,
	.csi2_lane_count      = isp_csi2_complexio_lanes_count,
	.csi2_cfg_vp_out_ctrl = isp_csi2_ctrl_config_vp_out_ctrl,
	.csi2_ctrl_update     = isp_csi2_ctrl_update,
	.csi2_cfg_virtual_id  = isp_csi2_ctx_config_virtual_id,
	.csi2_ctx_update      = isp_csi2_ctx_update,
	.csi2_calc_phy_cfg0   = isp_csi2_calc_phy_cfg0,
};
#endif


static struct i2c_board_info __initdata zoom2_i2c_bus1_info[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &ldp_twldata,
	},
};


static void synaptics_dev_init(void)
{
	/* Set the ts_gpio pin mux */
	omap_cfg_reg(R21_3430_GPIO163);

	if (omap_request_gpio(OMAP_SYNAPTICS_GPIO) < 0) {
		printk(KERN_ERR "can't get synaptics pen down GPIO\n");
		return;
	}
	omap_set_gpio_direction(OMAP_SYNAPTICS_GPIO, 1);
	omap_set_gpio_debounce(OMAP_SYNAPTICS_GPIO, 1);
	omap_set_gpio_debounce_time(OMAP_SYNAPTICS_GPIO, 0xa);
}

static int synaptics_power(int power_state)
{
	/* TODO: synaptics is powered by vbatt */
	return 0;
}

static struct synaptics_i2c_rmi_platform_data synaptics_platform_data[] = {
	{
		.version	= 0x0,
		.power		= &synaptics_power,
		.flags		= SYNAPTICS_SWAP_XY,
	}
};

static struct i2c_board_info __initdata zoom2_i2c_bus2_info[] = {
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME,  SYNAPTICS_I2C_ADDR),
		.platform_data = &synaptics_platform_data,
		.irq = 0,
	},
#endif
#if defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)
	{
		I2C_BOARD_INFO("imx046", IMX046_I2C_ADDR),
		.platform_data = &zoom2_imx046_platform_data,
	},
#endif
#ifdef CONFIG_VIDEO_LV8093
	{
		I2C_BOARD_INFO(LV8093_NAME,  LV8093_AF_I2C_ADDR),
		.platform_data = &zoom2_lv8093_platform_data,
	},
#endif
};

static int __init omap_i2c_init(void)
{
	omap_register_i2c_bus(1, 100, zoom2_i2c_bus1_info,
			ARRAY_SIZE(zoom2_i2c_bus1_info));
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI
	zoom2_i2c_bus2_info[0].irq = OMAP_GPIO_IRQ(OMAP_SYNAPTICS_GPIO);
#endif
	omap_register_i2c_bus(2, 100, zoom2_i2c_bus2_info,
			ARRAY_SIZE(zoom2_i2c_bus2_info));
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

#ifdef CONFIG_OMAP_HS_MMC3
static void config_wlan_gpio(void)
{
	/* WLAN IRQ */
	omap_cfg_reg(B24_3430_GPIO101);
	omap_cfg_reg(W21_3430_GPIO162);
}
#endif

static void config_mcbsp(void)
{
	omap_cfg_reg(J9_3430_MCBSP4_CLKX);
	omap_cfg_reg(G8_3430_MCBSP4_DR);
	omap_cfg_reg(H9_3430_MCBSP4_DX);
	omap_cfg_reg(W5_3430_MCBSP4_FSX);
	omap_cfg_reg(J16_3430_MCBSP_CLKS);
}

extern int __init omap_zoom2_debugboard_init(void);

static void __init omap_ldp_init(void)
{
	omap_i2c_init();
	platform_add_devices(ldp_devices, ARRAY_SIZE(ldp_devices));
	omap_board_config = ldp_config;
	omap_board_config_size = ARRAY_SIZE(ldp_config);
#ifdef CONFIG_SPI_IDCC
        idcc_srdy_gpio = SPI_GPIO_SRDY;
        ldp_spi_board_info[1].irq = OMAP_GPIO_IRQ(idcc_srdy_gpio);
#endif

	spi_register_board_info(ldp_spi_board_info,
				ARRAY_SIZE(ldp_spi_board_info));
	synaptics_dev_init();

#ifdef CONFIG_OMAP3_PM
	prcm_init();
#endif
	msecure_init();
#ifdef CONFIG_SPI_IDCC
	idcc6071_dev_init();
#endif

	ldp_flash_init();
	omap_serial_init();
	omap_zoom2_debugboard_init();
	usb_musb_init();
#ifdef CONFIG_OMAP_HS_MMC3
	config_wlan_gpio();
#endif
	config_mcbsp();
	hsmmc_init();
}

static void __init omap_ldp_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP_ZOOM2, "OMAP ZOOM2 board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_ldp_map_io,
	.init_irq	= omap_ldp_init_irq,
	.init_machine	= omap_ldp_init,
	.timer		= &omap_timer,
MACHINE_END
