#define CONFIG_OMAP3430_ES2
/**
 * drivers/video/omap/omap2_disp_out.c
 *
 * Driver for LCD and TV output on OMAP24xx SDPs
 *	- Tested on OMAP2420 H4
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * History:
 * 30-Mar-2006	Khasim	Added LCD data lines 18/16 support
 * 15-Apr-2006	Khasim	Modified proc fs to sysfs
 * 20-Apr-2006	Khasim	Modified PM/DPM support for Mobi-Linux
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/delay.h>
#ifdef CONFIG_TRACK_RESOURCES
#include <linux/device.h>
#endif
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <asm/delay.h>

#include <mach/gpio.h>
#include <mach/clock.h>
#include <mach/hardware.h>
#include <mach/mux.h>
#include <mach/power_companion.h>
#include <mach/resource.h>
#include <mach/display.h>

#if defined(CONFIG_MACH_OMAP_2430SDP) || defined (CONFIG_MACH_OMAP_3430SDP) ||\
    defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
#include <linux/i2c/twl4030.h>
#endif
#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif
#if defined(CONFIG_FB_OMAP_LCD_WVGA) || defined(CONFIG_PM)
#include <linux/spi/spi.h>
#endif
#include "omap_fb.h"

#define DRIVER			"omap2_disp_out"
#define	DRIVER_DESC		"OMAP Display output driver"

#define OMAP2_LCD_DRIVER	"omap2_lcd"
#define OMAP2_TV_DRIVER		"omap2_tv"


#ifdef CONFIG_MACH_OMAP_H4
#define OMAP24xx_LCD_DEVICE		"h4_lcd"
#define OMAP24xx_TV_DEVICE		"h4_tv"
#endif

#ifdef CONFIG_MACH_OMAP_2430SDP
#define OMAP24xx_LCD_DEVICE		"sdp2430_lcd"
#define OMAP24xx_TV_DEVICE		"sdp2430_tv"
#endif

#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP_LDP)\
					|| defined(CONFIG_MACH_OMAP_ZOOM2)
#if 0
#define OMAP24xx_LCD_DEVICE		"sdp3430_lcd"
#define OMAP24xx_TV_DEVICE		"sdp3430_tv"
#else
#define OMAP24xx_LCD_DEVICE		"omap2_lcd"
#define OMAP24xx_TV_DEVICE		"omap2_tv"
#endif
#endif

#ifdef CONFIG_FB_OMAP_720P_STREAMING
#define H4_LCD_XRES		1280
#define H4_LCD_YRES		720
#define H4_LCD_PIXCLOCK_MAX	13468 /* ??? in pico seconds  */
#define H4_LCD_PIXCLOCK_MIN	13468  /* ??? in pico seconds */
#else
#ifdef CONFIG_FB_OMAP_LCD_WVGA
#define H4_LCD_XRES		800
#define H4_LCD_YRES		480
#define H4_LCD_PIXCLOCK_MAX	45871 /* in pico seconds - freq 21.8 MHz*/
#define H4_LCD_PIXCLOCK_MIN	37000 /* in pico seconds - freq 25.7 MHz*/
#else
#ifdef CONFIG_FB_OMAP_LCD_VGA
#define H4_LCD_XRES		480
#define H4_LCD_YRES		640
#define H4_LCD_PIXCLOCK_MAX	46300 /* in pico seconds  */
#define H4_LCD_PIXCLOCK_MIN	37000  /* in pico seconds */
#else
#ifdef CONFIG_OMAP3430_ES2
#define H4_LCD_XRES		240
#define H4_LCD_YRES		320
#define H4_LCD_PIXCLOCK_MAX	167000 /* in pico seconds  */
#define H4_LCD_PIXCLOCK_MIN	152000  /* in pico seconds */
#else
#define H4_LCD_XRES		240
#define H4_LCD_YRES		320
#define H4_LCD_PIXCLOCK_MAX	185186 /* freq 5.4 MHz */
#define H4_LCD_PIXCLOCK_MIN	138888 /* freq 7.2 MHz */
#endif
#endif
#endif
#endif

#define H4_TV_XRES		640
#define H4_TV_YRES		480

#ifdef CONFIG_MACH_OMAP_2430SDP
#define LCD_PANEL_ENABLE_GPIO		154
#define LCD_PANEL_BACKLIGHT_GPIO	91
#endif

#if defined(CONFIG_ARCH_OMAP3430)
#if defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
#define LCD_PANEL_ENABLE_GPIO		(15 + OMAP_MAX_GPIO_LINES)
#define LCD_PANEL_QVGA_GPIO		56
#define LCD_PANEL_BACKLIGHT_GPIO	(7 + OMAP_MAX_GPIO_LINES)
#define DVI_OUTPUT_GPIO			8
#define DVI_POWER_ON_GPIO		167
#elif defined(CONFIG_OMAP3430_ES2)
#define LCD_PANEL_ENABLE_GPIO		5
#define LCD_PANEL_BACKLIGHT_GPIO	8
#else
#define LCD_PANEL_ENABLE_GPIO		28
#define LCD_PANEL_BACKLIGHT_GPIO	24
#endif
#endif

#define TV_INT_GPIO			33
#define TV_DETECT_DELAY			40 /*Delay for TV detection logic*/

#define CONFIG_OMAP2_LCD
#define ENABLE_VDAC_DEDICATED		0x03
#define ENABLE_VDAC_DEV_GRP             0x20
#define ENABLE_VPLL2_DEDICATED		0x05
#define ENABLE_VPLL2_DEV_GRP            0xE0

#ifdef CONFIG_VIDEO_OMAP24XX_TVOUT
#define CONFIG_OMAP2_TV
#define MENELAUS_I2C_ADAP_ID		0
#endif

extern int omap24xx_get_dss1_clock(void);
extern ssize_t
fb_out_show(struct device *dev, struct device_attribute *attr, char *buf);

extern ssize_t
fb_out_store(struct device *dev, struct device_attribute *attr,
	     const char *buffer, size_t count);

extern int fb_out_layer;
extern int omap24xxfb_set_output_layer(int layer);

#ifdef CONFIG_FB_OMAP_720P_STREAMING
extern int config_dsipll_lclk(int lck, int sysclk, int syclk2, int mode);
#endif
#ifdef CONFIG_ARCH_OMAP34XX
extern int lpr_enabled;
#endif

#ifdef CONFIG_OMAP_DSI
extern void edisco_init(u32 *handle);
#endif

#ifdef CONFIG_OMAP3_PM
struct res_handle *lcd_rhandle;
struct res_handle *dsi_rhandle;
struct res_handle *tv_rhandle;
#endif

/*------------------------------------------------------------------------*/

#if defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
unsigned char lcd_panel_reset_gpio;
#endif

#ifdef CONFIG_FB_OMAP_LCD_WVGA
static struct spi_device *wvgalcd_spi;
#endif

int automatic_link;
#ifdef CONFIG_OMAP2_LCD

static int dvi_in_use; /* dvi output flag */
static int lcd_in_use;
static int lcd_backlight_state = 1;

static int previous_lcd_backlight_state = 1;

#ifdef CONFIG_MACH_OMAP_H4
static int h4_read_gpio_expa(u8 *val);
static int h4_write_gpio_expa(u8 val);
#endif

static void lcd_backlight_off(struct work_struct *work);
static void lcd_backlight_on(struct work_struct *work);

#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT

DECLARE_WORK(lcd_bklight_on, lcd_backlight_on);
DECLARE_WORK(lcd_bklight_off, lcd_backlight_off);

static void lcd_panel_enable(struct work_struct *work);
static void lcd_panel_disable(struct work_struct *work);

DECLARE_WORK(lcd_panel_on, lcd_panel_enable);
DECLARE_WORK(lcd_panel_off, lcd_panel_disable);

#ifdef CONFIG_FB_OMAP_LCD_WVGA
static int spi_wvgalcd_send(unsigned char reg_addr, unsigned char reg_data);
static int initialize_nec_wvgalcd(void);
#endif    /* CONFIG_FB_OMAP_LCD_WVGA */

static void
power_lcd_backlight(int level)
{
	switch (level) {
		case LCD_OFF:
			if(!in_interrupt())
				lcd_backlight_off(NULL);
			else
				schedule_work(&lcd_bklight_off);
			break;
		default:
			if(!in_interrupt())
				lcd_backlight_on(NULL);
			else
				schedule_work(&lcd_bklight_on);
			break;
	}
}

static void
power_lcd_panel(int level)
{
	switch (level) {
		case LCD_OFF:
			if(!in_interrupt())
				lcd_panel_disable(NULL);
			else
				schedule_work(&lcd_panel_off);
			break;
		default:
			if(!in_interrupt())
				lcd_panel_enable(NULL);
			else
				schedule_work(&lcd_panel_on);
			break;
	}
}

#ifdef CONFIG_MACH_OMAP_H4

/* Write to GPIO EXPA on the board.
 * The GPIO expanders need an independent I2C client driver.
 */
static int
h4_write_gpio_expa(u8 val)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[1];

	adap = i2c_get_adapter(0);
	if (!adap)
		return -ENODEV;
	msg->addr = 0x20;	/* I2C address of GPIO EXPA */
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;
	data[0] = val;
	err = i2c_transfer(adap, msg, 1);
	if (err >= 0)
		return 0;
	return err;
}

/* Read from GPIO EXPA on the board.
 * The GPIO expanders need an independent I2C client driver.
 */
static int
h4_read_gpio_expa(u8 *val)
{
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[1];

	adap = i2c_get_adapter(0);
	if (!adap)
		return -ENODEV;
	msg->addr = 0x20;	/* I2C address of GPIO EXPA */
	msg->flags = I2C_M_RD;
	msg->len = 1;
	msg->buf = data;
	err = i2c_transfer(adap, msg, 1);
	*val = data[0];
	if (err >= 0)
		return 0;
	return err;
}
#endif

static void
lcd_panel_enable(struct work_struct *work)
{
#ifdef CONFIG_MACH_OMAP_H4
        unsigned char expa;
        int err;

        /* read current state of GPIO EXPA outputs */
        if ((err = h4_read_gpio_expa(&expa))) {
                printk(KERN_ERR DRIVER
                       "Error reading GPIO EXPA\n");
                return;
        }
        /* Set GPIO EXPA P7 (LCD_ENVDD) to power-up LCD and
         * set GPIO EXPA P5 (LCD_ENBKL) to turn on backlight
         */
        if ((err = h4_write_gpio_expa(expa | 0x80))) {
                printk(KERN_ERR DRIVER
                       "Error writing to GPIO EXPA\n");
                return;
        }
#endif
#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 1);
	/* power to the RGB lines from T2 is issued separately in
	 * omap2_dss-rgb_enable */
#elif defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 1);
#endif
}
void
omap2_dss_rgb_enable(void)
{
#ifdef CONFIG_MACH_OMAP_2430SDP
	twl4030_vaux2_ldo_use();
#else
	if(is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
#ifdef CONFIG_OMAP3_PM
		resource_request(dsi_rhandle, T2_VAUX3_2V80);
#else
		if (0 != twl4030_vaux3_ldo_use())
			printk(KERN_WARNING "omap2_disp: twl4030_vaux3_ldo_use returns error \n");
#endif
	}
	else {
#ifdef CONFIG_OMAP3_PM
		resource_request(lcd_rhandle, T2_VPLL2_1V80);
#else
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				ENABLE_VPLL2_DEDICATED,TWL4030_VPLL2_DEDICATED);

		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				ENABLE_VPLL2_DEV_GRP,TWL4030_VPLL2_DEV_GRP);
#endif
		mdelay(4);
	}
#endif
}

EXPORT_SYMBOL(omap2_dss_rgb_enable);


static void
lcd_panel_disable(struct work_struct *work)
{
#ifdef CONFIG_MACH_OMAP_H4
        unsigned char expa;
        int err;

        /* read current state of GPIO EXPA outputs */
        if ((err = h4_read_gpio_expa(&expa))) {
                printk(KERN_ERR DRIVER
                       "Error reading GPIO EXPA\n");
                return;
        }

        /* Clear GPIO EXPA P7 (LCD_ENVDD) to power-uoff LCD and
         * clear GPIO EXPA P5 (LCD_ENBKL) to turn off backlight
         */
        if ((err = h4_write_gpio_expa(expa & ~0x80))) {
                printk(KERN_ERR DRIVER
                       "Error writing to GPIO EXPA\n");
                return;
        }
#endif
#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 0);
	/* power to the RGB lines is disabled in omap2_dss_rgb_disable */
#elif defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 0);
#endif
}

		void
omap2_dss_rgb_disable(void)
{
#ifdef CONFIG_MACH_OMAP_2430SDP
		twl4030_vaux2_ldo_unuse();
#else
		if(is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
#ifdef CONFIG_OMAP3_PM
				resource_release(dsi_rhandle);
#else
				if (0 != twl4030_vaux3_ldo_unuse())
						printk(KERN_WARNING "omap2_disp: twl4030_vaux3_ldo_unuse returns error \n");
#endif
		}
		else {
#ifdef CONFIG_OMAP3_PM
				resource_release(lcd_rhandle);
#else
				twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						0x0, TWL4030_VPLL2_DEDICATED);

				twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
						0x0, TWL4030_VPLL2_DEV_GRP);
#endif
				mdelay(4);
		}
#endif

}
EXPORT_SYMBOL(omap2_dss_rgb_disable);

static void
lcd_backlight_on(struct work_struct *work)
{
#ifdef CONFIG_MACH_OMAP_H4
	unsigned char expa;
	int err;

	/* read current state of GPIO EXPA outputs */
	if ((err = h4_read_gpio_expa(&expa))) {
		printk(KERN_ERR DRIVER
		       "Error reading GPIO EXPA\n");
		return;
	}

	/* Set GPIO EXPA P7 (LCD_ENVDD) to power-up LCD and
	 * set GPIO EXPA P5 (LCD_ENBKL) to turn on backlight
	 */
	if ((err = h4_write_gpio_expa(expa | 0x20))) {
		printk(KERN_ERR DRIVER
		       "Error writing to GPIO EXPA\n");
		return;
	}
	lcd_backlight_state = LCD_ON;
#endif
#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_set_gpio_dataout(LCD_PANEL_BACKLIGHT_GPIO, 1);
	lcd_backlight_state = LCD_ON;
#elif defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 1);
	lcd_backlight_state = LCD_ON;
#endif
}

static void
lcd_backlight_off(struct work_struct *work)
{
#ifdef CONFIG_MACH_OMAP_H4
	unsigned char expa;
	int err;

	/* read current state of GPIO EXPA outputs */
	if ((err = h4_read_gpio_expa(&expa))) {
		printk(KERN_ERR DRIVER
		       "Error reading GPIO EXPA\n");
		return;
	}

	/* Clear GPIO EXPA P7 (LCD_ENVDD) to power-uoff LCD and
	 * clear GPIO EXPA P5 (LCD_ENBKL) to turn off backlight
	 */
	if ((err = h4_write_gpio_expa(expa & ~0x20))) {
		printk(KERN_ERR DRIVER
		       "Error writing to GPIO EXPA\n");
		return;
	}
	lcd_backlight_state = LCD_OFF;
#endif
#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_set_gpio_dataout(LCD_PANEL_BACKLIGHT_GPIO, 0);
	lcd_backlight_state = LCD_OFF;
#elif defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 0);
	lcd_backlight_state = LCD_OFF;
#endif
}

void enable_backlight(void)
{
	/* If already enabled, return*/
	if (lcd_in_use)
		return;
	if (previous_lcd_backlight_state == LCD_ON) {
		power_lcd_backlight(LCD_ON);

#ifdef CONFIG_FB_OMAP_LCD_WVGA
	spi_setup(wvgalcd_spi);
	/* Wake up sequence */
	spi_wvgalcd_send(2, 0x00);  /* R2 = 00h */
	initialize_nec_wvgalcd();
#endif
#if 0
		power_lcd_panel(LCD_ON);
#endif
	}
	lcd_in_use = 1;
}
EXPORT_SYMBOL(enable_backlight);

void disable_backlight(void)
{
	/* If LCD is already disabled, return*/
	if (!lcd_in_use)
		return;
	previous_lcd_backlight_state = lcd_backlight_state;
	power_lcd_backlight(LCD_OFF);
#if 0
	power_lcd_panel(LCD_OFF);
#endif
#ifdef CONFIG_FB_OMAP_LCD_WVGA
       /* Stand-by sequence */
        spi_wvgalcd_send(2, 0x01);  /* R2 = 01h */
        mdelay(40);
#endif

	lcd_in_use = 0;
}
EXPORT_SYMBOL(disable_backlight);

#if defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
static void
enable_dvi_output(void)
{
	/* If already enable, return*/
	if (dvi_in_use)
		return;
	omap2_disp_set_lcddatalines(LCD_DATA_LINE_24BIT);
	/* DVI output enable sequence */
	omap_set_gpio_direction(LCD_PANEL_QVGA_GPIO, 1);
	omap_set_gpio_direction(DVI_POWER_ON_GPIO, 0);
	omap_set_gpio_direction(DVI_OUTPUT_GPIO, 0);
	omap_set_gpio_dataout(DVI_POWER_ON_GPIO, 1);
	omap_set_gpio_dataout(DVI_OUTPUT_GPIO, 0);

	dvi_in_use = 1;
}

static void
disable_dvi_output(void)
{
	/* If is already disable, return*/
	if (!dvi_in_use)
		return;
	omap2_disp_set_lcddatalines(LCD_DATA_LINE_18BIT);
	/* DVI output disable sequence */
	omap_set_gpio_dataout(DVI_OUTPUT_GPIO, 1);
	omap_set_gpio_dataout(DVI_POWER_ON_GPIO, 0);
	omap_set_gpio_direction(DVI_OUTPUT_GPIO, 1);
	omap_set_gpio_direction(DVI_POWER_ON_GPIO, 1);
	omap_set_gpio_direction(LCD_PANEL_QVGA_GPIO, 0);

	dvi_in_use = 0;
}
#endif

static int gpio_reserved = 0;

/*---------------------------------------------------------------------------*/
#ifdef CONFIG_FB_OMAP_LCD_WVGA
/* NEC WVGA LCD related
 * We're intializing all register for the WVGA LCD */

static int
spi_wvgalcd_send(unsigned char reg_addr, unsigned char reg_data)
{
      int ret = 0;
      unsigned int cmd = 0;
      unsigned int data = 0;
      cmd = 0x0000 | reg_addr; /* register address write */
      data = 0x0100 | reg_data ; /* register data write */
      data = (cmd << 16) | data;
      spi_write(wvgalcd_spi, (unsigned char *)&data, 4);
      udelay(10);
      return ret;
}

static int initialize_nec_wvgalcd(void)
{
	/* Initialization Sequence
	* No information was provided by NEC for the following registers */
	spi_wvgalcd_send(3, 0x01);    /* R3 = 01h */
	spi_wvgalcd_send(0, 0x00);    /* R0 = 00h */
	spi_wvgalcd_send(1, 0x01);    /* R1 = 0x01 (normal), 0x03 (reversed) */
	spi_wvgalcd_send(4, 0x00);    /* R4 = 00h */
	spi_wvgalcd_send(5, 0x14);    /* R5 = 14h */
	spi_wvgalcd_send(6, 0x24);    /* R6 = 24h */
	spi_wvgalcd_send(16, 0xD7);   /* R16 = D7h */
	spi_wvgalcd_send(17, 0x00);   /* R17 = 00h */
	spi_wvgalcd_send(18, 0x00);   /* R18 = 00h */
	spi_wvgalcd_send(19, 0x55);   /* R19 = 55h */
	spi_wvgalcd_send(20, 0x01);   /* R20 = 01h */
	spi_wvgalcd_send(21, 0x70);   /* R21 = 70h */
	spi_wvgalcd_send(22, 0x1E);   /* R22 = 1Eh */
	spi_wvgalcd_send(23, 0x25);   /* R23 = 25h */
	spi_wvgalcd_send(24, 0x25);   /* R24 = 25h */
	spi_wvgalcd_send(25, 0x02);   /* R25 = 02h */
	spi_wvgalcd_send(26, 0x02);   /* R26 = 02h */
	spi_wvgalcd_send(27, 0xA0);   /* R27 = A0h */
	spi_wvgalcd_send(32, 0x2F);   /* R32 = 2Fh */
	spi_wvgalcd_send(33, 0x0F);   /* R33 = 0Fh */
	spi_wvgalcd_send(34, 0x0F);   /* R34 = 0Fh */
	spi_wvgalcd_send(35, 0x0F);   /* R35 = 0Fh */
	spi_wvgalcd_send(36, 0x0F);   /* R36 = 0Fh */
	spi_wvgalcd_send(37, 0x0F);   /* R37 = 0Fh */
	spi_wvgalcd_send(38, 0x0F);   /* R38 = 0Fh */
	spi_wvgalcd_send(39, 0x00);   /* R39 = 00h */
	spi_wvgalcd_send(40, 0x02);   /* R40 = 02h */
	spi_wvgalcd_send(41, 0x02);   /* R41 = 02h */
	spi_wvgalcd_send(42, 0x02);   /* R42 = 02h */
	spi_wvgalcd_send(43, 0x0F);   /* R43 = 0Fh */
	spi_wvgalcd_send(44, 0x0F);   /* R44 = 0Fh */
	spi_wvgalcd_send(45, 0x0F);   /* R45 = 0Fh */
	spi_wvgalcd_send(46, 0x0F);   /* R46 = 0Fh */
	spi_wvgalcd_send(47, 0x0F);   /* R47 = 0Fh */
	spi_wvgalcd_send(48, 0x0F);   /* R48 = 0Fh */
	spi_wvgalcd_send(49, 0x0F);   /* R49 = 0Fh */
	spi_wvgalcd_send(50, 0x00);   /* R50 = 00h */
	spi_wvgalcd_send(51, 0x02);   /* R51 = 02h */
	spi_wvgalcd_send(52, 0x02);   /* R52 = 02h */
	spi_wvgalcd_send(53, 0x02);   /* R53 = 02h */
	spi_wvgalcd_send(80, 0x0C);   /* R80 = 0Ch */
	spi_wvgalcd_send(83, 0x42);   /* R83 = 42h */
	spi_wvgalcd_send(84, 0x42);   /* R84 = 42h */
	spi_wvgalcd_send(85, 0x41);   /* R85 = 41h */
	spi_wvgalcd_send(86, 0x14);   /* R86 = 14h */
	spi_wvgalcd_send(89, 0x88);   /* R89 = 88h */
	spi_wvgalcd_send(90, 0x01);   /* R90 = 01h */
	spi_wvgalcd_send(91, 0x00);   /* R91 = 00h */
	spi_wvgalcd_send(92, 0x02);   /* R92 = 02h */
	spi_wvgalcd_send(93, 0x0C);   /* R93 = 0Ch */
	spi_wvgalcd_send(94, 0x1C);   /* R94 = 1Ch */
	spi_wvgalcd_send(95, 0x27);   /* R95 = 27h */
	spi_wvgalcd_send(98, 0x49);   /* R98 = 49h */
	spi_wvgalcd_send(99, 0x27);   /* R99 = 27h */
	spi_wvgalcd_send(102, 0x76);  /* R102 = 76h */
	spi_wvgalcd_send(103, 0x27);  /* R103 = 27h */
	spi_wvgalcd_send(112, 0x01);  /* R112 = 01h */
	spi_wvgalcd_send(113, 0x0E);  /* R113 = 0Eh */
	spi_wvgalcd_send(114, 0x02);  /* R114 = 02h */
	spi_wvgalcd_send(115, 0x0C);  /* R115 = 0Ch */
	spi_wvgalcd_send(118, 0x0C);  /* R118 = 0Ch */
	spi_wvgalcd_send(121, 0x30); /* R121 = 0x30 (normal), 0x10 (reversed) */
	spi_wvgalcd_send(130, 0x00);  /* R130 = 00h */
	spi_wvgalcd_send(131, 0x00);  /* R131 = 00h */
	spi_wvgalcd_send(132, 0xFC);  /* R132 = FCh */
	spi_wvgalcd_send(134, 0x00);  /* R134 = 00h */
	spi_wvgalcd_send(136, 0x00);  /* R136 = 00h */
	spi_wvgalcd_send(138, 0x00);  /* R138 = 00h */
	spi_wvgalcd_send(139, 0x00);  /* R139 = 00h */
	spi_wvgalcd_send(140, 0x00);  /* R140 = 00h */
	spi_wvgalcd_send(141, 0xFC);  /* R141 = FCh */
	spi_wvgalcd_send(143, 0x00);  /* R143 = 00h */
	spi_wvgalcd_send(145, 0x00);  /* R145 = 00h */
	spi_wvgalcd_send(147, 0x00);  /* R147 = 00h */
	spi_wvgalcd_send(148, 0x00);  /* R148 = 00h */
	spi_wvgalcd_send(149, 0x00);  /* R149 = 00h */
	spi_wvgalcd_send(150, 0xFC);  /* R150 = FCh */
	spi_wvgalcd_send(152, 0x00);  /* R152 = 00h */
	spi_wvgalcd_send(154, 0x00);  /* R154 = 00h */
	spi_wvgalcd_send(156, 0x00);  /* R156 = 00h */
	spi_wvgalcd_send(157, 0x00);  /* R157 = 00h */
	udelay(20);
	spi_wvgalcd_send(2, 0x00);    /* R2 = 00h */

	return 0;
}
#endif
/*------------------------------------------------------------------------*/

#ifndef CONFIG_LCD_IOCTL
static
#endif
int omap_lcd_init(struct omap_lcd_info *info)
{
#ifdef CONFIG_FB_OMAP_LCD_WVGA
		u32 pixclock    = H4_LCD_PIXCLOCK_MAX,  /* picoseconds */
		left_margin	= 4,	/* pixclocks */
		right_margin	= 6,	/* pixclocks */
		upper_margin	= 4,	/* line clocks */
		lower_margin	= 3,	/* line clocks */
		hsync_len	= 1,	/* pixclocks */
		vsync_len	= 1,	/* line clocks */
		sync		= 1,	/* hsync & vsync polarity */
		acb		= 0x28,	/* AC-bias pin frequency */
		ipc		= 0,	/* Invert pixel clock */
		onoff		= 0;	/* HSYNC/VSYNC Pixel clk Control*/
#else
#ifdef CONFIG_FB_OMAP_LCD_VGA
	u32	pixclock	= H4_LCD_PIXCLOCK_MAX,/* picoseconds */
		left_margin	= 80,		/* pixclocks */
		right_margin	= 90,		/* pixclocks */
		upper_margin	= 0,		/* line clocks */
		lower_margin	= 0,		/* line clocks */
		hsync_len	= 3,		/* pixclocks */
		vsync_len	= 2,		/* line clocks */
		sync		= 1,		/* hsync & vsync polarity */
		acb		= 0x28,		/* AC-bias pin frequency */
		ipc		= 1,		/* Invert pixel clock */
		onoff		= 1;		/* HSYNC/VSYNC Pixel clk Control*/
#else
#ifdef CONFIG_OMAP3430_ES2
	u32	pixclock	= H4_LCD_PIXCLOCK_MAX,/* picoseconds */
		left_margin	= 39,		/* pixclocks */
		right_margin	= 45,		/* pixclocks */
		upper_margin	= 1,		/* line clocks */
		lower_margin	= 0,		/* line clocks */
		hsync_len	= 3,		/* pixclocks */
		vsync_len	= 2,		/* line clocks */
		sync		= 1,		/* hsync & vsync polarity */
		acb		= 0x28,		/* AC-bias pin frequency */
		ipc		= 1,		/* Invert pixel clock */
		onoff		= 1;		/* HSYNC/VSYNC Pixel clk Control*/
#else
	u32	pixclock	= H4_LCD_PIXCLOCK_MAX,/* picoseconds */
		left_margin	= 40,		/* pixclocks */
		right_margin	= 4,		/* pixclocks */
		upper_margin	= 8,		/* line clocks */
		lower_margin	= 2,		/* line clocks */
		hsync_len	= 4,		/* pixclocks */
		vsync_len	= 2,		/* line clocks */
		sync		= 0,		/* hsync & vsync polarity */
		acb		= 0,		/* AC-bias pin frequency */
		ipc		= 0,		/* Invert pixel clock */
		onoff		= 0;		/* HSYNC/VSYNC Pixel clk Control*/

#endif
#endif
#endif
#ifndef CONFIG_OMAP_DSI
	u32 clkdiv;
#endif
#ifdef CONFIG_OMAP_DSI
	u32  handle;
#endif

#ifdef CONFIG_LCD_IOCTL
	if (info) {
		pixclock     = info->pixclock,
		left_margin  = info->left_margin,
		right_margin = info->right_margin,
		upper_margin = info->upper_margin,
		lower_margin = info->lower_margin,
		hsync_len    = info->hsync_len,
		vsync_len    = info->vsync_len,
		sync         = info->sync,
		acb          = info->acb,
		ipc          = info->ipc,
		onoff        = info->onoff;
	}

	if (gpio_reserved == 1)
		goto bypass_gpio;
#endif

#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_request_gpio(LCD_PANEL_ENABLE_GPIO);  /* LCD panel */
	omap_request_gpio(LCD_PANEL_BACKLIGHT_GPIO);	 /* LCD backlight */
	omap_set_gpio_direction(LCD_PANEL_ENABLE_GPIO, 0); /* output */
	omap_set_gpio_direction(LCD_PANEL_BACKLIGHT_GPIO, 0); /* output */
#elif defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)

	if (is_sil_rev_greater_than(OMAP3430_REV_ES3_0)) {
		/* Production Zoom2 Board:
		 * GPIO-96 is the LCD_RESET_GPIO
		 */
		omap_cfg_reg(C25_3430_GPIO96);
		lcd_panel_reset_gpio = 96;
	} else {
		/* Pilot Zoom2 board
		 * GPIO-55 is the LCD_RESET_GPIO
		 */
		omap_cfg_reg(XX_3430_GPIO55);
		lcd_panel_reset_gpio = 55;
	}

	omap_request_gpio(lcd_panel_reset_gpio);
	omap_request_gpio(LCD_PANEL_QVGA_GPIO);
	omap_request_gpio(DVI_OUTPUT_GPIO);
	omap_request_gpio(DVI_POWER_ON_GPIO);
	gpio_request(LCD_PANEL_ENABLE_GPIO, "lcd-panel");  /* LCD panel */
	gpio_request(LCD_PANEL_BACKLIGHT_GPIO, "lcd-bklit"); /* LCD backlight */

#ifdef CONFIG_FB_OMAP_720P_STREAMING
	omap_set_gpio_direction(DVI_POWER_ON_GPIO, 0);
	omap_set_gpio_direction(DVI_OUTPUT_GPIO, 0);
	omap_set_gpio_dataout(DVI_POWER_ON_GPIO, 1);
	omap_set_gpio_dataout(DVI_OUTPUT_GPIO, 0);
#else
	omap_set_gpio_direction(LCD_PANEL_QVGA_GPIO, 0);
	omap_set_gpio_direction(lcd_panel_reset_gpio, 0);
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 0); /* output */
	gpio_direction_output(LCD_PANEL_BACKLIGHT_GPIO, 0); /* output */

#ifdef CONFIG_FB_OMAP_LCD_VGA
	omap_set_gpio_dataout(LCD_PANEL_QVGA_GPIO, 0);
#else
	omap_set_gpio_dataout(LCD_PANEL_QVGA_GPIO, 1);
#endif
	omap_set_gpio_dataout(lcd_panel_reset_gpio, 1);
#endif
#endif

#ifdef CONFIG_LCD_IOCTL
bypass_gpio:
	gpio_reserved = 1;
#endif
	omap2_dss_rgb_enable();
#ifdef CONFIG_FB_OMAP_720P_STREAMING
#ifdef CONFIG_OMAP3_PM
	resource_request(dsi_rhandle, T2_VAUX3_2V80);
#else
	if (0 != twl4030_vaux3_ldo_use())
		printk(KERN_WARNING "omap2_disp: twl4030_vaux3_ldo_use returns error \n");
#endif
#endif
	omap2_disp_get_dss();

#ifdef CONFIG_FB_OMAP_LCD_WVGA
	/* Initialization of the NEC WVGA LCD */
	initialize_nec_wvgalcd();
#endif

#ifndef CONFIG_OMAP_DSI
	omap2_disp_set_panel_size(OMAP2_OUTPUT_LCD, H4_LCD_XRES, H4_LCD_YRES);

	/* clkdiv = pixclock / (omap2 dss1 clock period) */
	clkdiv = pixclock / (1000000000UL/omap24xx_get_dss1_clock());
	clkdiv /= 1000;

#ifdef CONFIG_FB_OMAP_720P_STREAMING
	/* Request dsi pll for 148MHZ */
	config_dsipll_lclk(148, 2, 26, 1);
	/* config_dsipll_lclk(108, 2, 26, 1); */
	/* edisco_init(&handle); */
#endif

	omap2_disp_config_lcd(clkdiv,
				left_margin - 1,	/* hbp */
				right_margin - 1,	/* hfp */
				hsync_len - 1,		/* hsw */
				upper_margin,		/* vbp */
				lower_margin,		/* vfp */
				vsync_len - 1		/* vsw */
				);

	omap2_disp_lcdcfg_polfreq(sync,	/* horizontal sync active low */
				sync,	/* vertical sync active low */
				acb,	/* ACB */
				ipc,	/* IPC */
				onoff	/* ONOFF */
				);

	omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);
	udelay(20);
#else

	/* Enable power to DSI, edisco and initialize the edisco */
#ifdef CONFIG_OMAP3_PM
	resource_request(dsi_rhandle, T2_VAUX3_2V80);
#else
	if (0 != twl4030_vaux3_ldo_use())
			printk(KERN_WARNING "omap2_disp: twl4030_vaux3_ldo_unuse returns error \n");
#endif
	omap_set_gpio_dataout(LCD_PANEL_ENABLE_GPIO, 1);

	omap_set_gpio_dataout(LCD_PANEL_BACKLIGHT_GPIO, 1);

	printk("Setting pin mux for DSI !!!! \n");
	omap_writel(0x02010201,0x480020dc);
	omap_writel(0x02010201,0x480020e0);
	omap_writel(0x02010201,0x480020e4);
	printk("Calling to initialize edisco !!!! \n");
	edisco_init(&handle);
	printk("Edisco handle = 0x%x !!!!!\n",handle);
	// edisco_write(handle, 0x12,&ebuf_len,&ebuf);
#endif

#ifndef CONFIG_FB_OMAP_720P_STREAMING
	enable_backlight();
	power_lcd_panel(LCD_ON);
#endif
#if !defined(CONFIG_OMAP_DSI) && !defined(CONFIG_FB_OMAP_720P_STREAMING)
	omap2_disp_put_dss();
#endif

	printk(KERN_DEBUG DRIVER
	       "LCD panel %dx%d\n", H4_LCD_XRES, H4_LCD_YRES);
	return 0;
}

static int
lcd_exit(void)
{
	if (!lcd_in_use)
		return 0;

	omap2_disp_get_dss();
	omap2_disp_disable_output_dev(OMAP2_OUTPUT_LCD);
#ifdef CONFIG_FB_OMAP_LCD_WVGA
	/* Power Off sequence */
	spi_wvgalcd_send(16, 0x05);  /* R16 = 05h */
	udelay(20);
	spi_wvgalcd_send(16, 0x01);  /* R16 = 01h */
	udelay(20);
	spi_wvgalcd_send(16, 0x00);  /* R16 = 00h */
	udelay(20);
	spi_wvgalcd_send(3, 0x01);  /* R3 = 01h */
#endif
	omap2_disp_put_dss();

#if defined(CONFIG_MACH_OMAP_2430SDP) || defined(CONFIG_MACH_OMAP_3430SDP)
	omap_free_gpio(LCD_PANEL_ENABLE_GPIO);  /* LCD panel */
	omap_free_gpio(LCD_PANEL_BACKLIGHT_GPIO);  /* LCD backlight */
#elif defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
	omap_free_gpio(lcd_panel_reset_gpio);
	omap_free_gpio(LCD_PANEL_QVGA_GPIO);
	gpio_free(LCD_PANEL_ENABLE_GPIO);  /* LCD panel */
	gpio_free(LCD_PANEL_BACKLIGHT_GPIO);  /* LCD backlight */
#endif

	lcd_in_use = 0;

	return 0;
}
/* ------------------------------------------------------------------------------ */
/* Power and device Management */

static int __init
lcd_probe(struct platform_device *odev)
{
#ifdef CONFIG_FB_OMAP_LCD_WVGA
	return 0;
#else
	omap_cfg_reg(AF21_3430_GPIO8);
	omap_cfg_reg(B23_3430_GPIO167);
	return omap_lcd_init(0);
#endif
}

#ifdef CONFIG_FB_OMAP_LCD_WVGA
static int wvgalcd_probe(struct spi_device *spi)
{
	omap_cfg_reg(AF21_3430_GPIO8);
	omap_cfg_reg(B23_3430_GPIO167);
	omap_cfg_reg(AB1_3430_McSPI1_CS2);
	wvgalcd_spi = spi;
	wvgalcd_spi->mode = SPI_MODE_0;
	wvgalcd_spi->bits_per_word = 32;
	spi_setup(wvgalcd_spi);

	return omap_lcd_init(0);
}
#endif

#ifdef CONFIG_PM
static int lcd_suspend(struct platform_device *odev, pm_message_t state);
static int lcd_resume(struct platform_device *odev);
#ifdef CONFIG_FB_OMAP_LCD_WVGA
static int wvgalcd_suspend(struct spi_device *spi, pm_message_t state);
static int wvgalcd_resume(struct spi_device *spi);
#endif   /* CONFIG_FB_OMAP_LCD_WVGA */
#endif   /* CONFIG_PM */

#ifdef CONFIG_FB_OMAP_LCD_WVGA
static struct spi_driver wvga_lcd_driver = {
       .probe		= wvgalcd_probe,
#ifdef CONFIG_PM
       .suspend		= wvgalcd_suspend,
       .resume		= wvgalcd_resume,
#endif
	.driver		= {
		.name	= "wvgalcd",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
};
#endif

static struct platform_driver omap2_lcd_driver = {
	.driver = {
		.name   = OMAP2_LCD_DRIVER,
	},
	.probe          = lcd_probe,
#ifdef CONFIG_PM
	.suspend        = lcd_suspend,
	.resume         = lcd_resume,
#endif
};

static struct platform_device lcd_device = {
        .name     = OMAP24xx_LCD_DEVICE,
	.id    = 9,
};

#ifdef CONFIG_PM
static int
lcd_suspend(struct platform_device *odev, pm_message_t state)
{
	if (!lcd_in_use)
		return 0;
	disable_backlight();
	omap2_dss_rgb_disable();
	automatic_link = 0;

	return 0;
}

static int
lcd_resume(struct platform_device *odev)
{
	if (lcd_in_use)
		return 0;
	omap2_dss_rgb_enable();
	udelay(20);
	enable_backlight();

	return 0;
}

#ifdef CONFIG_FB_OMAP_LCD_WVGA
static int
wvgalcd_suspend(struct spi_device *spi, pm_message_t state)
{
	if (!lcd_in_use)
		return 0;
	disable_backlight();
	/* Stand-by sequence */
	spi_wvgalcd_send(2, 0x01);  /* R2 = 01h */
	mdelay(40);
	omap2_dss_rgb_disable();
	automatic_link = 0;

	return 0;
}

static int
wvgalcd_resume(struct spi_device *spi)
{
	if (lcd_in_use)
		return 0;
	omap2_dss_rgb_enable();
	udelay(20);

	/* Wake up sequence */
	spi_wvgalcd_send(2, 0x00);  /* R2 = 00h */
	enable_backlight();

	return 0;
}
#endif

#endif /* CONFIG_PM */
#endif	/* CONFIG_OMAP2_LCD */

/*---------------------------------------------------------------------------*/

#ifdef CONFIG_OMAP2_TV

static int tv_in_use;

static void h4_i2c_tvout_off(struct work_struct *work);
static void h4_i2c_tvout_on(struct work_struct *work);

DECLARE_WORK(h4_tvout_on, h4_i2c_tvout_on);
DECLARE_WORK(h4_tvout_off, h4_i2c_tvout_off);

static void
power_tv(int level)
{
	switch(level) {
		case TV_OFF:
			if(!in_interrupt())
				h4_i2c_tvout_off(NULL);
			else
				schedule_work(&h4_tvout_off);
			break;
		default:
			if(!in_interrupt())
				h4_i2c_tvout_on(NULL);
			else
				schedule_work(&h4_tvout_on);
		break;
	}
}

static void
h4_i2c_tvout_off(struct work_struct *work)
{
#if defined (CONFIG_MACH_OMAP_H4) || defined (CONFIG_TWL4030_CORE_M1)
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	/*
	 * Turn OFF TV block (AVDD and VREF) in menelaus chip
	 * MENELAUS_LDO_CTRL8 (0x11 -> 0x03)
	 */
	adap = i2c_get_adapter(MENELAUS_I2C_ADAP_ID);
	if (!adap) {
		printk(KERN_ERR DRIVER
		       "Unable to get I2C adapter \n");
	}
	msg->addr = 0x72;/* I2C address of Menelaus Chip */
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = 0x11; /* LD0_CTRL8 */
	data[1] = 0x00; /* Disable bits for the 0.5V reference LDO */
	err = i2c_transfer(adap, msg, 1);
	if (err > 2) {
		printk(KERN_ERR DRIVER
				"Disabling TV block through Menelaus failed %d\n",err);
	}
#endif
#if defined (CONFIG_OMAP2_TV)
	omap2_disp_get_all_clks();
	omap2_disp_set_tvref(TVREF_OFF);
	omap2_disp_put_all_clks();
#endif

#if (defined(CONFIG_TWL4030_CORE_T2) && defined(CONFIG_I2C_TWL4030_CORE))  \
		|| defined(CONFIG_ARCH_OMAP34XX)
	omap2_disp_get_all_clks();
	omap2_disp_set_tvref(TVREF_OFF);
	omap2_disp_put_all_clks();
#ifdef CONFIG_OMAP3_PM
	resource_release(tv_rhandle);
#else
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,0x00,TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,0x00,TWL4030_VDAC_DEV_GRP);
#endif
#endif
}

static void
h4_i2c_tvout_on(struct work_struct *work)
{
#if defined (CONFIG_MACH_OMAP_H4) || defined (CONFIG_TWL4030_CORE_M1)
	struct i2c_adapter *adap;
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

        /*
	 * Turn ON TV block (AVDD and VREF) in menelaus chip
	 * MENELAUS_LDO_CTRL8 (0x11 -> 0x03)
	 */
	adap = i2c_get_adapter(MENELAUS_I2C_ADAP_ID);
	if (!adap) {
		printk(KERN_ERR DRIVER
		       "Unable to get I2C adapter \n");
	}
	msg->addr = 0x72;/* I2C address of Menelaus Chip */
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = 0x11; /* LD0_CTRL8 */
	data[1] = 0x03;	/* Enable bits for the 0.5V reference
			 * and the VADAC LDO
			 */
	err = i2c_transfer(adap, msg, 1);
	if (err > 2) {
		printk(KERN_ERR DRIVER
				"Enabling TV block through Menelaus failed %d\n",err);
	}
#endif
#if defined (CONFIG_OMAP2_TV)
	omap2_disp_get_all_clks();
	omap2_disp_set_tvref(TVREF_ON);
	omap2_disp_put_all_clks();
#endif
#if (defined(CONFIG_TWL4030_CORE_T2) && defined(CONFIG_I2C_TWL4030_CORE)) \
		|| defined(CONFIG_ARCH_OMAP34XX)
	omap2_disp_get_all_clks();
	omap2_disp_set_tvref(TVREF_ON);
	omap2_disp_put_all_clks();

#ifdef CONFIG_OMAP3_PM
	resource_request(tv_rhandle,T2_VDAC_1V80);
#else
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED,TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP,TWL4030_VDAC_DEV_GRP);
#endif
#endif
}

static int
tv_init(void)
{
	omap2_disp_get_all_clks();

	omap2_disp_set_tvstandard(NTSC_M);
	omap2_disp_set_panel_size(OMAP2_OUTPUT_TV, H4_TV_XRES, H4_TV_YRES);

	omap2_disp_put_all_clks();
	printk(KERN_DEBUG DRIVER
	       "TV %dx%d interlaced\n", H4_TV_XRES, H4_TV_YRES);
#if defined(CONFIG_MACH_OMAP_3430SDP)
	omap_request_gpio(TV_INT_GPIO);
	omap_set_gpio_direction(TV_INT_GPIO, 1);
#endif

#ifdef CONFIG_OMAP3_PM
	resource_request(tv_rhandle,T2_VDAC_1V80);
	resource_release(tv_rhandle);
#endif
	return 0;
}

static int
tv_exit(void)
{
		if (!tv_in_use)
		return 0;

	omap2_disp_get_all_clks();
	omap2_disp_disable_output_dev(OMAP2_OUTPUT_TV);
	power_tv(TV_OFF);
	omap2_disp_put_all_clks();
#if defined(CONFIG_MACH_OMAP_3430SDP)
	omap_free_gpio(TV_INT_GPIO);
#endif
	tv_in_use = 0;
	return 0;
}

static int __init tv_probe(struct platform_device *odev);
#ifdef CONFIG_PM
static int tv_suspend(struct platform_device *odev, pm_message_t state);
static int tv_resume(struct platform_device *odev);
#endif

static struct platform_driver omap2_tv_driver = {
	.driver = {
		.name   = OMAP2_TV_DRIVER,
	},
	.probe          = tv_probe,
#ifdef CONFIG_PM
	.suspend        = tv_suspend,
	.resume         = tv_resume,
#endif
};

static struct platform_device tv_device = {
        .name     = OMAP24xx_TV_DEVICE,
	.id    = 10,
};

static int __init
tv_probe(struct platform_device *odev)
{
	return tv_init();
}

#ifdef CONFIG_PM
static int
tv_suspend(struct platform_device *odev, pm_message_t state)
{
		if (!tv_in_use)
			return 0;

	/* TODO-- need to delink DSS and TV clocks.. For now, TV is put to
	 * off in fb_blank and put_dss */

        tv_in_use = 0;
		return 0;
}

static int
tv_resume(struct platform_device *odev)
{
	if (tv_in_use)
		return 0;

	/* TODO-- need to delink DSS and TV clocks.. For now, TV is put to
	 * on in fb_blank and get_dss */
	tv_in_use = 1;
	return 0;
}

#endif	/* CONFIG_PM */

#endif /* CONFIG_OMAP2_TV */

/*---------------------------------------------------------------------------*/
/* Sysfs support */

struct board_properties {
	struct module *owner;
};
static struct board_properties *bd;

struct dispc_device {
	struct device dev;
	struct board_properties *props;
};
static struct dispc_device *new_bd;

#ifdef CONFIG_OMAP2_TV
static struct dispc_device *wss_bd;
#endif

#define FN_IN	printk(KERN_DEBUG "%s Entry \n", __func__);

static void dispc_class_release(struct class *class)
{
	if (new_bd != NULL)
		kfree(new_bd);
#ifdef CONFIG_OMAP2_TV
	if (wss_bd != NULL)
		kfree(wss_bd);
#endif
	if(bd != NULL) kfree(bd);
}

struct class dispc_class = {
	.name = "display_control",
	.class_release = dispc_class_release,
};

static ssize_t
read_layer_out(char *buf, int layer)
{
	int p = 0;
	int output_dev = omap2_disp_get_output_dev(layer);

	switch (output_dev) {
	case OMAP2_OUTPUT_LCD:
		p = (dvi_in_use) ? sprintf(buf, "dvi\n") : \
			sprintf(buf, "lcd\n");
		break;
	case OMAP2_OUTPUT_TV:
		p = sprintf(buf, "tv\n");
		break;
	}
	return(p);
}

static ssize_t
write_layer_out(const char *buffer, size_t count,int layer)
{
	int out_dev;
	if (!buffer || (count == 0))
		return 0;

	/* only 'lcd', 'dvi' or 'tv' are valid inputs */
	if (strncmp(buffer, "lcd", 3) == 0) {
#ifdef CONFIG_FB_OMAP_720P_STREAMING
		out_dev = omap2_disp_get_output_dev(layer);
		printk(KERN_INFO "lcd is not a valid input\n");
#else
		out_dev = OMAP2_OUTPUT_LCD;
#endif
#if defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
		if (dvi_in_use)
			disable_dvi_output();
#endif
	}
#ifdef CONFIG_OMAP2_TV
	else if (strncmp(buffer, "tv", 2) == 0) {
		out_dev = OMAP2_OUTPUT_TV;
#if defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
		if (dvi_in_use)
			disable_dvi_output();
#endif
	}
#endif
	/* dvi input is needed on LDP*/
#if defined(CONFIG_MACH_OMAP_LDP) || defined(CONFIG_MACH_OMAP_ZOOM2)
	else if (strncmp(buffer, "dvi", 3) == 0) {
		out_dev = OMAP2_OUTPUT_LCD;
		if (!dvi_in_use)
			enable_dvi_output();
	}
#endif
	else
		return -EINVAL;

#ifdef CONFIG_OMAP2_TV
	if (out_dev == OMAP2_OUTPUT_TV) {
			if (tv_in_use == 0) {
			omap2_disp_get_all_clks();
				h4_i2c_tvout_on(NULL);
				omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);
				tv_in_use = 1;
			omap2_disp_put_all_clks();
			}
	}
#endif

	if(omap2_disp_get_output_dev(layer) == out_dev)
		return count;

	omap2_disp_get_all_clks();

	if (fb_out_layer != OMAP2_GRAPHICS)
	{
		omap2_disp_disable_layer(fb_out_layer);
		omap2_disp_release_layer (fb_out_layer);
	}
	mdelay(1);
	omap2_disp_set_output_dev(layer, out_dev);
	mdelay(1);

	if(fb_out_layer != OMAP2_GRAPHICS)
		omap24xxfb_set_output_layer(fb_out_layer);
	omap2_disp_put_all_clks();

#ifdef CONFIG_OMAP2_TV
	if ((omap2_disp_get_output_dev(OMAP2_GRAPHICS) == OMAP2_OUTPUT_LCD) &&
		(omap2_disp_get_output_dev(OMAP2_VIDEO1) == OMAP2_OUTPUT_LCD) &&
		(omap2_disp_get_output_dev(OMAP2_VIDEO2) == OMAP2_OUTPUT_LCD)) {
			if (tv_in_use != 0) {
				omap2_disp_get_all_clks();
				omap2_disp_disable_output_dev(OMAP2_OUTPUT_TV);
				h4_i2c_tvout_off(NULL);
				tv_in_use = 0;
				omap2_disp_put_all_clks();
			}
	}
#endif

	return count;
}

static ssize_t
graphics_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return(read_layer_out(buf,OMAP2_GRAPHICS));
}

static ssize_t
graphics_store(struct device *dev, struct device_attribute *attr,
	       const char *buffer, size_t count)
{
	return(write_layer_out(buffer,count,OMAP2_GRAPHICS));
}

static ssize_t
video1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return(read_layer_out(buf,OMAP2_VIDEO1));
}

static ssize_t
video1_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	return(write_layer_out(buffer,count,OMAP2_VIDEO1));
}

static ssize_t
video2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return(read_layer_out(buf,OMAP2_VIDEO2));
}

static ssize_t
video2_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	return(write_layer_out(buffer,count,OMAP2_VIDEO2));
}

static ssize_t
lcdbacklight_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int p =0;

	switch (lcd_backlight_state) {
	case LCD_ON:
		p = sprintf(buf, "on\n");
	break;
	case LCD_OFF:
		p = sprintf(buf, "off\n");
	break;
	}
	return p;
}

static ssize_t
lcdbacklight_store(struct device *dev, struct device_attribute *attr,
			const char *buffer, size_t count)
{
	/*
	 * If the LCD is already in suspend state do not accept any changes to
	 * backlight
	 */

	if (!lcd_in_use)
		return -EINVAL;

	if (!buffer || (count == 0))
		return 0;

	if (strncmp(buffer,"on",2) == 0){
		power_lcd_backlight(LCD_ON);
	}
	else if (strncmp(buffer, "off", 3) == 0){
		power_lcd_backlight(LCD_OFF);
	}
	else{
		return -EINVAL;
	}
	return count;
}

static ssize_t
lcd_data_lines_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int p =0;
	int current_lcddatalines_state;

	current_lcddatalines_state = omap2_disp_get_lcddatalines();

	switch (current_lcddatalines_state) {
	case LCD_DATA_LINE_24BIT:
			p = sprintf(buf, "24 bits\n");
			break;
	case LCD_DATA_LINE_18BIT:
			p = sprintf(buf, "18 bits\n");
			break;
	case LCD_DATA_LINE_16BIT:
			p = sprintf(buf, "16 bits\n");
			break;
	}
	return (p);
}

static ssize_t
lcd_data_lines_store(struct device *dev, struct device_attribute *attr,
			const char *buffer, size_t count)
{
    int no_of_data_lines;

	if (!buffer || (count == 0))
		return 0;

	if (strncmp(buffer, "24", 2) == 0)
		no_of_data_lines = LCD_DATA_LINE_24BIT;
	else if (strncmp(buffer, "18", 2) == 0)
		no_of_data_lines = LCD_DATA_LINE_18BIT;
	else if (strncmp(buffer, "16", 2) == 0)
		no_of_data_lines = LCD_DATA_LINE_16BIT;
	else
		return -EINVAL;

	omap2_disp_set_lcddatalines(no_of_data_lines);
	return count;
}

static ssize_t
dithering_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int p=0;
	int current_dither_state;

        omap2_disp_get_dss();
	current_dither_state = omap2_disp_get_dithering();
	switch (current_dither_state) {
		case DITHERING_ON:	p = sprintf(buf, "on\n");
			break;
		case DITHERING_OFF:	p = sprintf(buf, "off\n");
			break;
	}
        omap2_disp_put_dss();
	return p;
}

static ssize_t
dithering_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	int dither_state;

	if (!buffer || (count == 0)){
		return 0;
	}
	if (strncmp(buffer,"on",2) == 0){
		dither_state = DITHERING_ON;
	}
	else if (strncmp(buffer, "off", 3) == 0){
		dither_state = DITHERING_OFF;
	}
	else
		return -EINVAL;
        omap2_disp_get_dss();
	omap2_disp_set_dithering(dither_state);
        omap2_disp_put_dss();
	return count;
}


#ifdef CONFIG_ARCH_OMAP34XX
static ssize_t
lcd_alphablend_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int p=0;
	int alphablend_state;

  omap2_disp_get_dss();
	alphablend_state = omap2_disp_get_alphablend(OMAP2_OUTPUT_LCD);
	switch (alphablend_state) {
		case 0:	p = sprintf(buf, "off\n");
			break;
		case 1:	p = sprintf(buf, "on\n");
			break;
	}
        omap2_disp_put_dss();
	return p;
}

static ssize_t
lcd_alphablend_store(struct device *dev, struct device_attribute *attr,
		     const char *buffer, size_t count)
{
	int alpha_state;

	if (!buffer || (count == 0)){
		return 0;
	}
	if (strncmp(buffer,"on",2) == 0){
		alpha_state = 1;
	}
	else if (strncmp(buffer, "off", 3) == 0){
		alpha_state = 0;
	}
	else
		return -EINVAL;
        omap2_disp_get_dss();
	omap2_disp_set_alphablend(OMAP2_OUTPUT_LCD,alpha_state);
        omap2_disp_put_dss();
	return count;
}

static ssize_t
tv_alphablend_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int p=0;
	int alphablend_state;

        omap2_disp_get_dss();
	alphablend_state = omap2_disp_get_alphablend(OMAP2_OUTPUT_TV);
	switch (alphablend_state) {
		case 0:	p = sprintf(buf, "off\n");
			break;
		case 1:	p = sprintf(buf, "on\n");
			break;
	}
        omap2_disp_put_dss();
	return p;
}

static ssize_t
tv_alphablend_store(struct device *dev, struct device_attribute *attr,
		    const char *buffer, size_t count)
{
	int alpha_state;

	if (!buffer || (count == 0)){
		return 0;
	}
	if (strncmp(buffer,"on",2) == 0){
		alpha_state = 1;
	}
	else if (strncmp(buffer, "off", 3) == 0){
		alpha_state = 0;
	}
	else
		return -EINVAL;
        omap2_disp_get_dss();
	omap2_disp_set_alphablend(OMAP2_OUTPUT_TV,alpha_state);
        omap2_disp_put_dss();
	return count;
}
static ssize_t
gfx_global_alpha_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int p=0;
	unsigned char alphablend_value;

        omap2_disp_get_dss();
	alphablend_value = omap2_disp_get_global_alphablend_value(OMAP2_GRAPHICS);
	p = sprintf(buf, "%d \n", alphablend_value);
	omap2_disp_put_dss();
	return p;
}

static ssize_t
gfx_global_alpha_store(struct device *dev, struct device_attribute *attr,
		       const char *buffer, size_t count)
{
	unsigned char alpha_value;

	if (!buffer || (count == 0)){
		return 0;
	}

	if (sscanf(buffer,"%hhu",&alpha_value) != 1) {
		printk(KERN_ERR "gfx_global_alpha_store: Invalid value \n");
		return -EINVAL;
	}

	omap2_disp_get_dss();
	omap2_disp_set_global_alphablend_value(OMAP2_GRAPHICS,alpha_value);
        omap2_disp_put_dss();
	return count;
}


static ssize_t
vid2_global_alpha_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int p=0;
	int alphablend_value;

        omap2_disp_get_dss();
	alphablend_value = omap2_disp_get_global_alphablend_value(OMAP2_VIDEO2);
	p = sprintf(buf, "%d \n", alphablend_value);
	omap2_disp_put_dss();
	return p;
}

static ssize_t
vid2_global_alpha_store(struct device *dev, struct device_attribute *attr,
			const char *buffer, size_t count)
{
	int alpha_value;

	if (!buffer || (count == 0)){
		return 0;
	}

	if (sscanf(buffer,"%d",&alpha_value) != 1) {
		printk(KERN_ERR "gfx_global_alpha_store: Invalid value \n");
		return -EINVAL;
	}

	omap2_disp_get_dss();
	omap2_disp_set_global_alphablend_value(OMAP2_VIDEO2,alpha_value);
        omap2_disp_put_dss();
	return count;
}


static ssize_t lpr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int p = 0;

	switch (lpr_enabled) {
		case 0:	p = sprintf(buf, "disable\n");
			break;

		case 1:	p = sprintf(buf, "enable\n");
			break;
	}
	return p;
}

static ssize_t lpr_store(struct device *dev, struct device_attribute *attr,
			 const char *buffer, size_t count)
{
	int rc;

	if (!buffer || (count == 0))
		return 0;

	if (strncmp(buffer, "enable", 6) == 0) {
		if ((rc = omap2_disp_lpr_enable())) {
			printk("can't enable lpr!\n");
			return rc;
		}
	}
	else if (strncmp(buffer, "disable", 7) == 0) {
		if ((rc = omap2_disp_lpr_disable())) {
			printk("can't disable lpr!\n");
			return rc;
		}
	}
	else {
		return -EINVAL;
	}

	return count;
}

static ssize_t gfx_fifo_low_threshold_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int thrs;

	omap2_disp_get_dss();
	thrs = omap2_disp_get_gfx_fifo_low_threshold();
	omap2_disp_put_dss();

	return sprintf(buf, "%d\n", thrs);
}

static ssize_t gfx_fifo_low_threshold_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buffer,
					    size_t count)
{
	unsigned int v;

	if (!buffer || (count == 0)){
		return 0;
	}

	if (sscanf(buffer, "%d", &v) != 1) {
		printk(KERN_ERR "gfx_fifo_low_threshold: Invalid value\n");
		return -EINVAL;
	}

	omap2_disp_get_dss();
	omap2_disp_set_gfx_fifo_low_threshold(v);
	omap2_disp_put_dss();

	return count;
}

static ssize_t gfx_fifo_high_threshold_show(struct device *dev,
					    struct device_attribute *attr, char *buf)
{
	int thrs;

	omap2_disp_get_dss();
	thrs = omap2_disp_get_gfx_fifo_high_threshold();
	omap2_disp_put_dss();

	return sprintf(buf, "%d\n", thrs);
}

static ssize_t gfx_fifo_high_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t count)
{
	unsigned int v;

	if (!buffer || (count == 0)){
		return 0;
	}

	if (sscanf(buffer, "%d", &v) != 1) {
		printk(KERN_ERR "gfx_fifo_high_threshold: Invalid value\n");
		return -EINVAL;
	}

	omap2_disp_get_dss();
	omap2_disp_set_gfx_fifo_high_threshold(v);
	omap2_disp_put_dss();

	return count;
}


#endif

#ifdef CONFIG_OMAP2_TV
static ssize_t
tv_standard_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int p = 0;
	int current_tvstd;

	omap2_disp_get_dss();
	current_tvstd = omap2_disp_get_tvstandard();

	switch (current_tvstd) {
	case PAL_BDGHI:
		p = sprintf(buf, "pal_bdghi\n");
		break;
	case PAL_NC:
		p = sprintf(buf, "pal_nc\n");
		break;
	case PAL_N:
		p = sprintf(buf, "pal_n\n");
		break;
	case PAL_M:
		p = sprintf(buf, "pal_m\n");
		break;
	case PAL_60:
		p = sprintf(buf, "pal_60\n");
		break;
	case NTSC_M:
		p = sprintf(buf, "ntsc_m\n");
		break;
	case NTSC_J:
		p = sprintf(buf, "ntsc_j\n");
		break;
	case NTSC_443:
		p = sprintf(buf, "ntsc_443\n");
		break;
	}
	omap2_disp_put_dss();
	return (p);
}

static ssize_t
tv_standard_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	int tv_std;

	if (!buffer || (count == 0))
		return 0;

	if (strncmp(buffer, "pal_bdghi", 9) == 0)
		tv_std = PAL_BDGHI;
	else if (strncmp(buffer, "pal_nc", 6) == 0)
		tv_std = PAL_NC;
	else if (strncmp(buffer, "pal_n", 5) == 0)
		tv_std = PAL_N;
	else if (strncmp(buffer, "pal_m", 5) == 0)
		tv_std = PAL_M;
	else if (strncmp(buffer, "pal_60", 6) == 0)
		tv_std = PAL_60;
	else if (strncmp(buffer, "ntsc_m", 6) == 0)
		tv_std = NTSC_M;
	else if (strncmp(buffer, "ntsc_j", 6) == 0)
		tv_std = NTSC_J;
	else if (strncmp(buffer, "ntsc_443", 8) == 0)
		tv_std = NTSC_443;
	else
		return -EINVAL;
	omap2_disp_get_all_clks();
	omap2_disp_set_tvstandard(tv_std);
	omap2_disp_put_all_clks();
	return count;

}

static void enable_tv_detect(void){
	omap2_disp_get_all_clks();
	if (!tv_in_use) {
		omap2_disp_set_tvref(TVREF_ON);
#ifdef CONFIG_OMAP3_PM
		resource_request(tv_rhandle, T2_VDAC_1V80);
#else
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED, TWL4030_VDAC_DEDICATED);
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);
#endif
	}
	omap2_enable_tv_detect();
}

static void disable_tv_detect(void){
	omap2_disable_tv_detect();
	if (!tv_in_use) {
		omap2_disp_set_tvref(TVREF_OFF);
#ifdef CONFIG_OMAP3_PM
	resource_release(tv_rhandle);
#else
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
		TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
		TWL4030_VDAC_DEV_GRP);
#endif
       }
	omap2_disp_put_all_clks();
}

ssize_t
set_output_device(const char *buffer, int layer) {
	ssize_t ret;
	if (strncmp(buffer, "lcd", 3) == 0)
		ret = write_layer_out("lcd", 3, layer);
	else  if (strncmp(buffer, "tv", 2) == 0)
		ret = write_layer_out("tv", 2, layer);
	else
		ret = -EINVAL;
	return  ret;
}
EXPORT_SYMBOL(set_output_device);

int
get_tv_state(void){
	int tv_state;

	enable_tv_detect();
	msleep(TV_DETECT_DELAY);
	tv_state = omap_get_gpio_datain(TV_INT_GPIO);
	disable_tv_detect();
	return tv_state;
}
EXPORT_SYMBOL(get_tv_state);

static ssize_t
tv_state_show(struct device *dev, struct device_attribute *attr, char *buf) {
	return  sprintf(buf, "%d\n", get_tv_state());
}

static ssize_t
tv_state_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count) {
	if (strncmp(buffer, "1", 1) == 0)
		automatic_link = 1;
	else if (strncmp(buffer, "0", 1) == 0)
		automatic_link = 0;
	return count;
}
static ssize_t
wss_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int p = 0;
	int state;

	FN_IN
	state = omap2_disp_get_wss_enable();

	switch (state) {
	case WSS_OFF:
		p = sprintf(buf, "off\n");
		break;
	case WSS_ODD_ON:
		p = sprintf(buf, "odd on\n");
		break;
	case WSS_EVEN_ON:
		p = sprintf(buf, "even on\n");
		break;
	case WSS_EVEN_ODD_ON:
		p = sprintf(buf, "even and odd on\n");
		break;
	default:
		p = sprintf(buf, "invalid\n");
		break;
	}
	return p;
}

static ssize_t
wss_enable_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
	FN_IN
	if (!buffer || (count == 0))
		return 0;

	if (strncmp(buffer, "OFF", 3) == 0)
		omap2_disp_set_wss_enable(WSS_OFF);
	else if (strncmp(buffer, "odd", 3) == 0)
		omap2_disp_set_wss_enable(WSS_ODD_ON);
	else if (strncmp(buffer, "even", 4) == 0)
		omap2_disp_set_wss_enable(WSS_EVEN_ON);
	else if (strncmp(buffer, "even_odd", 8) == 0)
		omap2_disp_set_wss_enable(WSS_EVEN_ODD_ON);
	else
		return -EINVAL;
	return count;
}


static ssize_t
wss_fcc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int p = 0;
	unsigned int wssfcc;

	FN_IN

	wssfcc = omap2_disp_get_wss_fcc();
	p = sprintf(buf, "0x%x\n", wssfcc);
	return p;
}

static ssize_t
wss_fcc_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
    int wss_fcc;

	FN_IN
	if (!buf || (count == 0))
		return 0;

	if (sscanf(buf, "%x", &wss_fcc) != 1) {
		printk(KERN_ERR "wss_fcc_store: failed to get value from buffer \n");
		return -EINVAL;
	}
	omap2_disp_set_wss_fcc(wss_fcc);
	return count;
}


static ssize_t
wss_fwss_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int p = 0;
	int wssfwss;

	FN_IN

	wssfwss = omap2_disp_get_wss_fwss();
	p = sprintf(buf, "0x%x\n", wssfwss);
	return p;
}

static ssize_t
wss_fwss_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
    unsigned int wss_fwss;

	FN_IN
	if (!buf || (count == 0))
		return 0;

	if (sscanf(buf, "%x", &wss_fwss) != 1) {
		printk(KERN_ERR "wss_fwss_store: failed to get value from buffer \n");
		return -EINVAL;
	}
	omap2_disp_set_wss_fwss(wss_fwss);
	return count;
}

static ssize_t
wss_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int p = 0;
	unsigned int data;

	FN_IN
	data = omap2_disp_get_wss_data();
	p = sprintf(buf, "0x%x\n", data);
	return p;
}

static ssize_t
wss_data_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count)
{
    unsigned int wss_data;

	FN_IN
	if (!buffer || (count == 0))
		return 0;

	if (sscanf(buffer, "%x", &wss_data) != 1) {
		printk(KERN_ERR "wss_data_store: failed to get value from buffer \n");
		return -EINVAL;
	}
	omap2_disp_set_wss_data(wss_data);
	return count;
}

#endif

#define DECLARE_ATTR(_name,_mode,_show,_store)                  \
{                                                               \
	.attr   = { .name = __stringify(_name), .mode = _mode, .owner = THIS_MODULE },  \
	.show   = _show,                                        \
	.store  = _store,                                       \
}

static struct device_attribute bl_device_attributes[] = {
	DECLARE_ATTR(dithering,      S_IRWXUGO, dithering_show,    dithering_store),
	DECLARE_ATTR(graphics,       S_IRWXUGO, graphics_show,     graphics_store),
	DECLARE_ATTR(video1,         S_IRWXUGO, video1_show,       video1_store),
	DECLARE_ATTR(video2,         S_IRWXUGO, video2_show,       video2_store),
	DECLARE_ATTR(lcdbacklight,   S_IRWXUGO, lcdbacklight_show, lcdbacklight_store),
	DECLARE_ATTR(lcd_data_lines, S_IRWXUGO, lcd_data_lines_show, lcd_data_lines_store),
	DECLARE_ATTR(frame_buffer,   S_IRWXUGO, fb_out_show, fb_out_store),

#ifdef CONFIG_ARCH_OMAP34XX
	DECLARE_ATTR(lcd_alphablend, S_IRWXUGO, lcd_alphablend_show,
					lcd_alphablend_store),
	DECLARE_ATTR(tv_alphablend,  S_IRWXUGO, tv_alphablend_show,
					tv_alphablend_store),
	DECLARE_ATTR(gfx_global_alpha,  S_IRWXUGO, gfx_global_alpha_show,
					gfx_global_alpha_store),
	DECLARE_ATTR(vid2_global_alpha,  S_IRWXUGO, vid2_global_alpha_show,
					vid2_global_alpha_store),
	DECLARE_ATTR(lpr, S_IRWXUGO, lpr_show, lpr_store),
	DECLARE_ATTR(gfx_fifo_low_threshold, S_IRWXUGO,
		     gfx_fifo_low_threshold_show,
		     gfx_fifo_low_threshold_store),
	DECLARE_ATTR(gfx_fifo_high_threshold, S_IRWXUGO,
		     gfx_fifo_high_threshold_show,
		     gfx_fifo_high_threshold_store),
#endif
#ifdef CONFIG_OMAP2_TV
	DECLARE_ATTR(tv_standard, 0644, tv_standard_show, tv_standard_store),
	DECLARE_ATTR(tv_state, S_IRWXUGO, tv_state_show, tv_state_store),
	/* TODO -- make the WSS conditional compile */
	DECLARE_ATTR(wss_enable, S_IRWXUGO, wss_enable_show, wss_enable_store),
	DECLARE_ATTR(wss_fcc, S_IRWXUGO, wss_fcc_show, wss_fcc_store),
	DECLARE_ATTR(wss_fwss, S_IRWXUGO, wss_fwss_show, wss_fwss_store),
	DECLARE_ATTR(wss_data, S_IRWXUGO, wss_data_show, wss_data_store),
#endif
};

static int
create_sysfs_files(void)
{

	int rc=0,i,ret;

	if((ret = class_register(&dispc_class)) != 0 )
		return ret;

	bd = kmalloc(sizeof(struct board_properties), GFP_KERNEL);
	if (unlikely(!bd))
		return -ENOMEM;

	bd->owner = THIS_MODULE;
	new_bd = kmalloc(sizeof(struct dispc_device), GFP_KERNEL);
	if (unlikely(!new_bd))
		return -ENOMEM;
#ifdef CONFIG_OMAP2_TV
	wss_bd = kmalloc(sizeof(struct dispc_device), GFP_KERNEL);
	if (unlikely(!wss_bd))
		return -ENOMEM;
	wss_bd->props = bd;
	memset(&wss_bd->dev, 0, sizeof(wss_bd->dev));
	wss_bd->dev.class = &dispc_class;
	strlcpy(wss_bd->dev.bus_id, "wss_control", sizeof("wss_control"));

	rc = device_register(&wss_bd->dev);
	if (unlikely(rc)) {
		kfree(wss_bd);
		return -EPERM;
	}
#endif
	new_bd->props = bd;
	memset(&new_bd->dev, 0, sizeof(new_bd->dev));
	new_bd->dev.class = &dispc_class;
	strlcpy(new_bd->dev.bus_id, "omap_disp_control",
				sizeof("omap_disp_control"));
	rc = device_register(&new_bd->dev);

	if (unlikely(rc)) {
		kfree(new_bd);
		return -EPERM;
	}
	for (i = 0; i < ARRAY_SIZE(bl_device_attributes); i++) {
		rc = device_create_file(&new_bd->dev, &bl_device_attributes[i]);
		if (unlikely(rc)) {
			while (--i >= 0){
				device_remove_file(&new_bd->dev,
						&bl_device_attributes[i]);
			}
			device_unregister(&new_bd->dev);
			/* No need to kfree(new_bd) since release() method does it for us*/
			return -EPERM;
		}
	}
	return 0;
}

static void
remove_sysfs_files(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(bl_device_attributes); i++) {
		device_remove_file(&new_bd->dev,
						&bl_device_attributes[i]);
	}
	device_unregister(&new_bd->dev);
}

static int __init
omap2_dispout_init(void)
{
	automatic_link = 0;

	if (create_sysfs_files() < 0) {
		printk(KERN_ERR DRIVER
		       "Could not create sysfs files for display control\n");
		return -ENODEV;
	}
#ifdef CONFIG_OMAP3_PM
	lcd_rhandle = NULL;
	dsi_rhandle = NULL;
	tv_rhandle = NULL;
#ifdef CONFIG_OMAP2_LCD
		dvi_in_use = 0;
		lcd_rhandle = resource_get("LCD", "t2_vpll2");
		if (lcd_rhandle == NULL) {
				printk(KERN_ERR DRIVER ": Failed to get lcd power resources !! \n");
				return -ENODEV;
		}
#endif
#ifdef CONFIG_OMAP2_TV
		tv_in_use = 0;
		tv_rhandle = resource_get("TV", "t2_vdac");
		if (tv_rhandle == NULL) {
				printk(KERN_ERR DRIVER ": Failed to get TV power resources !! \n");
				return -ENODEV;
		}
#endif
#if defined(CONFIG_OMAP_DSI) || defined(CONFIG_FB_OMAP_720P_STREAMING)
		dsi_rhandle = resource_get("DSI", "t2_vaux3");
		if (dsi_rhandle == NULL) {
				printk(KERN_ERR DRIVER ": Failed to get DSI power resources !! \n");
				return -ENODEV;
		}
#endif

#endif

#ifdef CONFIG_OMAP2_LCD
	/* Register the driver with LDM */
	if (platform_driver_register(&omap2_lcd_driver)) {
		printk(KERN_ERR DRIVER ": failed to register omap2_lcd driver\n");
		return -ENODEV;
	}

	/* Register the device with LDM */
	if (platform_device_register(&lcd_device)) {
		printk(KERN_ERR DRIVER ": failed to register lcd device\n");
		platform_driver_unregister(&omap2_lcd_driver);
		return -ENODEV;
	}
#endif

#ifdef CONFIG_OMAP2_TV
	/* Register the driver with LDM */
	if (platform_driver_register(&omap2_tv_driver)) {
		printk(KERN_ERR DRIVER ": failed to register omap2_tv driver\n");
		return -ENODEV;
	}

	/* Register the device with LDM */
	if (platform_device_register(&tv_device)) {
		printk(KERN_ERR DRIVER ": failed to register tv device\n");
		platform_driver_unregister(&omap2_tv_driver);
		return -ENODEV;
	}
#endif

#ifdef CONFIG_FB_OMAP_LCD_WVGA
	/* Register the driver with SPI */
	if (spi_register_driver(&wvga_lcd_driver)) {
		printk(KERN_ERR DRIVER ": failed to\
					 register wvga_lcd driver\n");
		return -ENODEV;
	}
#endif

		return 0;
}
device_initcall(omap2_dispout_init);

static void __exit
omap2_dispout_exit(void)
{
	remove_sysfs_files();

#ifdef CONFIG_OMAP2_LCD
	lcd_exit();
#ifdef CONFIG_FB_OMAP_LCD_WVGA
	spi_unregister_driver(&wvga_lcd_driver);
#endif
	platform_device_unregister(&lcd_device);
	platform_driver_unregister(&omap2_lcd_driver);
#endif

#ifdef CONFIG_OMAP2_TV
	tv_exit();
	platform_device_unregister(&tv_device);
	platform_driver_unregister(&omap2_tv_driver);
#endif
#ifdef CONFIG_OMAP3_PM
#ifdef CONFIG_OMAP2_LCD
		resource_put(lcd_rhandle);
#endif
#ifdef CONFIG_OMAP2_TV
		resource_put(tv_rhandle);
#endif
#ifdef CONFIG_OMAP_DSI
		resource_put(dsi_rhandle);
#endif
#endif
}
module_exit(omap2_dispout_exit);


/*---------------------------------------------------------------------------*/
/* Framebuffer related */

/* We're intializing the virtual framebuffer dimensions to
 * (H4_XRES, H4_YRES*3) in order to support triple buffering.  The
 * onscreen framebuffer can be flipped via the panning ioctl by specifying
 * offsets of (0, 0), (0, H4_YRES), or (0, 2*H4_YRES).
 */

static struct fb_var_screeninfo h4_lcd_var = {
	.xres		= H4_LCD_XRES,
	.yres		= H4_LCD_YRES,
#ifdef CONFIG_FB_XRES_ALIGN_TO_32BYTES
	.xres_virtual	= ((H4_LCD_XRES) + 31)& ~31,
#else
	.xres_virtual	= H4_LCD_XRES,
#endif
	.yres_virtual	= H4_LCD_YRES*3,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 16,
	.grayscale	= 0,
	.red		= {11, 5, 0},
	.green		= { 5, 6, 0},
	.blue		= { 0, 5, 0},
	.transp		= { 0, 0, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= H4_LCD_PIXCLOCK_MAX,/* picoseconds */
#ifdef CONFIG_FB_OMAP_LCD_WVGA
	.left_margin	= 4, 		/*  pixclocks */
	.right_margin	= 6,		/*  pixclocks */
	.upper_margin	= 4,		/* line clocks */
	.lower_margin	= 3,		/* line clocks */
	.hsync_len	= 1,		/* pixclocks */
	.vsync_len	= 1,		/* line clocks */
	.sync		= 1,		/* hsync & vsync polarity */
#else
#ifdef CONFIG_FB_OMAP_LCD_VGA
	.left_margin	= 80,		/* pixclocks */
	.right_margin	= 90,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 3,		/* pixclocks */
	.vsync_len	= 2,		/* line clocks */
	.sync		= 1,		/* hsync & vsync polarity */
#else
#ifdef CONFIG_OMAP3430_ES2
	.left_margin	= 39,		/* pixclocks */
	.right_margin	= 45,		/* pixclocks */
	.upper_margin	= 1,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 3,		/* pixclocks */
	.vsync_len	= 2,		/* line clocks */
	.sync		= 1,		/* hsync & vsync polarity */
#else
	.left_margin	= 40,		/* pixclocks */
	.right_margin	= 4,		/* pixclocks */
	.upper_margin	= 8,		/* line clocks */
	.lower_margin	= 2,		/* line clocks */
	.hsync_len	= 4,		/* pixclocks */
	.vsync_len	= 2,		/* line clocks */
	.sync		= 0,		/* hsync & vsync polarity */
#endif
#endif
#endif
	.vmode		= FB_VMODE_NONINTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo ntsc_tv_var = {
/* NTSC frame size is 720 * 480,
 * but due to overscan, about 640 x 430 is visible
 */
	.xres = 640,
	.yres = 430,
	.xres_virtual	= 720,
	.yres_virtual	= 480 * 3,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 16,
	.grayscale	= 0,
	.red		= {11, 5, 0},
	.green		= { 5, 6, 0},
	.blue		= { 0, 5, 0},
	.transp		= { 0, 0, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 0,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};

static struct fb_var_screeninfo pal_tv_var = {
/* PAL frame size is 720 * 546,
 * but due to overscan, about 640 x 520 is visible
 */
	.xres = 640,
	.yres = 480,
	.xres_virtual	= 720,
	.yres_virtual	= 576 * 3,
	.xoffset	= 0,
	.yoffset	= 0,
	.bits_per_pixel	= 16,
	.grayscale	= 0,
	.red		= {11, 5, 0},
	.green		= { 5, 6, 0},
	.blue		= { 0, 5, 0},
	.transp		= { 0, 0, 0},
	.nonstd		= 0,
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.accel_flags	= 0,
	.pixclock	= 0,		/* picoseconds */
	.left_margin	= 0,		/* pixclocks */
	.right_margin	= 0,		/* pixclocks */
	.upper_margin	= 0,		/* line clocks */
	.lower_margin	= 0,		/* line clocks */
	.hsync_len	= 0,		/* pixclocks */
	.vsync_len	= 0,		/* line clocks */
	.sync		= 0,
	.vmode		= FB_VMODE_INTERLACED,
	.rotate		= 0,
	.reserved[0]	= 0,
};


void
get_panel_default_var(struct fb_var_screeninfo *var, int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		memcpy((struct fb_var_screeninfo *) var,
		       &h4_lcd_var, sizeof(*var));
	}
	else if (output_dev == OMAP2_OUTPUT_TV) {
		int tv = omap2_disp_get_tvstandard();
		if(tv == PAL_BDGHI ||
			 tv == PAL_NC    ||
			 tv == PAL_N     ||
			 tv == PAL_M     ||
			 tv == PAL_60){
			memcpy((struct fb_var_screeninfo *) var,
				&pal_tv_var, sizeof(*var));
		}else {
			memcpy((struct fb_var_screeninfo *) var,
				&ntsc_tv_var, sizeof(*var));
		}
	}
}

u32
get_panel_pixclock_max(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		return H4_LCD_PIXCLOCK_MAX;
	}
	else if (output_dev == OMAP2_OUTPUT_TV) {
		return ~0;
	}

	return -EINVAL;
}

u32
get_panel_pixclock_min(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		return H4_LCD_PIXCLOCK_MIN;
	}
	else if (output_dev == OMAP2_OUTPUT_TV) {
		return 0;
	}

	return -EINVAL;
}

EXPORT_SYMBOL(get_panel_default_var);
EXPORT_SYMBOL(get_panel_pixclock_max);
EXPORT_SYMBOL(get_panel_pixclock_min);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
