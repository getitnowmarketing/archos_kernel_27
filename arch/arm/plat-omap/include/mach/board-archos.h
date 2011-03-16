/*
 * linux/include/asm-arm/arch-omap/board-archosg6.h
 *
 * Hardware definitions for TI ARCHOS G6 board.
 *
 * Copyright (C) 2008 Niklas Schroeter, Archos S.A.,
 *
 * Derived from mach-omap2/board-omap3evm.h
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __MACH_BOARD_ARCHOS_H
#define __MACH_BOARD_ARCHOS_H

#include <mach/archos-gpio.h>
#include <mach/board.h>
#include <linux/mma7456l.h>
#include <asm/mach-types.h>

#define FLASH_BASE_NOR_ARCHOS	0x10000000  /* NOR flash (256 Meg aligned) */

/* various memory sizes */
#define FLASH_SIZE_NOR_ARCHOS	(SZ_1M*4)

#define OMAP3_WAKEUP (PRCM_WAKEUP_ATMEGA)

struct omap_dss_platform_data;
struct machine_desc;
struct tag;
struct meminfo;

/* max. number of hardware revisions (change if necessary) */
#define MAX_HWREVS	8
/*
 * board config tags, use with omap_get_config()
 * to distribute board configuration to init
 * code and drivers.
 */
struct archos_tsp_conf
{
	struct archos_gpio	irq_gpio;
	struct archos_gpio	pwr_gpio;
	u16 x_plate_ohms;
	u16 pressure_max;
};

struct archos_tsp_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_tsp_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_TSP	0x4e01

struct archos_vbus_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_gpio rev[MAX_HWREVS];
};
#define ARCHOS_TAG_VBUS0 0x4e02

struct archos_pwm_conf
{
	int timer;
	int mux_cfg;
};

struct archos_spi_conf
{
	struct archos_gpio spi_clk;
	struct archos_gpio spi_data;
	struct archos_gpio spi_cs;
};

struct archos_disp_conf
{
	struct archos_gpio cpldreset;
	struct archos_gpio disp_select;
	struct archos_gpio lcd_pwon;
	struct archos_gpio lcd_rst;
	struct archos_gpio bkl_pwon;
	struct archos_gpio lcd_pci;
	struct archos_gpio hdmi_dac;
	struct archos_gpio hdmi_it;
	struct archos_gpio cpld_aux;
	struct archos_pwm_conf bkl_pwm;
	struct archos_pwm_conf vcom_pwm;
	struct archos_spi_conf spi;
};

struct archos_display_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_disp_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_DISPLAY	0x4e03

struct archos_keys_conf
{
	struct archos_gpio power;
	struct archos_gpio vol_up;
	struct archos_gpio vol_down;
	struct archos_gpio cam_first;
	struct archos_gpio cam_full;
};
struct archos_keys_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_keys_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_KEYS	0x4e04

struct archos_usb_conf
{
	struct archos_gpio	usb_id;
	struct archos_gpio	enable_usb_ehci;
	struct archos_gpio	enable_usb_musb;
};

struct archos_usb_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_usb_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_USB		0x4e05

struct archos_charge_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_gpio rev[MAX_HWREVS];
};
#define ARCHOS_TAG_CHARGE 0x4e07

struct archos_usbhdd_conf
{
	struct archos_gpio hub_power;
	struct archos_gpio hdd_power;
};

struct archos_usbhdd_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_usbhdd_conf rev[MAX_HWREVS];
};
#define ARCHOS_TAG_USBHDD 0x4e08

struct archos_audio_conf
{
	struct archos_gpio spdif;
	struct archos_gpio hp_on;
	struct archos_gpio headphone_plugged;
};

struct archos_audio_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_audio_conf rev[MAX_HWREVS];
};
#define ARCHOS_TAG_AUDIO 0x4e09

struct archos_wifi_bt_config {
	int nrev; /* number of hardware revisions */
	struct archos_mmc_wifi_bt_dev_conf {
		struct archos_gpio wifi_power;
		struct archos_gpio wifi_irq;
		struct archos_gpio bt_power;
	} rev[MAX_HWREVS];
};

#define ARCHOS_TAG_WIFI_BT 0x4e0a
struct archos_sata_conf
{
	struct archos_gpio hdd_power;
	struct archos_gpio sata_power;
	struct archos_gpio sata_ready;
};
struct archos_sata_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_sata_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_SATA	0x4e0b

struct archos_sd_conf
{
	struct archos_gpio sd_power;
	struct archos_gpio sd_detect;
};
struct archos_sd_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_sd_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_SD	0x4e0c

struct archos_gps_conf
{
	struct archos_gpio gps_enable;
	struct archos_gpio gps_reset;
	struct archos_gpio gps_int;
};
struct archos_gps_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_gps_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_GPS	0x4e0d

struct archos_irblaster_conf
{
	struct archos_pwm_conf irblaster_pwm;
	struct archos_gpio irblaster_pwm_disable;
	struct archos_pwm_conf irblaster_ctrl_timer;
};

struct archos_irblaster_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_irblaster_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_IRBLASTER	0x4e0e

struct archos_irremote_conf
{
	struct archos_pwm_conf irremote_timer;
	struct archos_gpio irremote_timer_disable;
};

struct archos_irremote_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_irremote_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_IRREMOTE	0x4e0f

struct archos_uart3_conf
{
	int uart3_rx_mux;
	int uart3_tx_mux;
	struct archos_gpio gpio_uart3_rx;
	struct archos_gpio gpio_uart3_tx;
};

struct archos_uart3_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_uart3_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_UART3	0x4e10

struct archos_accel_conf
{
	struct archos_gpio accel_int1;
	struct archos_gpio accel_int2;
};

struct archos_accel_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_accel_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_ACCEL	0x4e11

struct archos_atmega_config {
	const char *name;
	struct archos_gpio irq;
};
#define ARCHOS_TAG_ATMEGA	0x4e12

struct archos_modem_idcc_conf
{
	struct archos_gpio pwron;
	struct archos_gpio wwan_reset;
	struct archos_gpio wwan_disable;
	struct archos_gpio master_rdy;
	struct archos_gpio slave_rdy;
};

struct archos_modem_idcc_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_modem_idcc_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_MODEM_IDCC	0x4e13

struct archos_modem_huawei_conf
{
	struct archos_gpio pwron;
	struct archos_gpio wwan_reset;
	struct archos_gpio wwan_disable;
	struct archos_gpio wake_module;
	struct archos_gpio wake_omap;
	struct archos_gpio enable_usb2;
	struct archos_gpio gps_enable;
};

struct archos_modem_huawei_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_modem_huawei_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_MODEM_HUAWEI	0x4e14

struct archos_vibrator_conf
{
	struct archos_gpio vibrator;
};

struct archos_vibrator_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_vibrator_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_VIBRATOR	0x4e15

struct archos_compass_conf
{
	struct archos_gpio reset;
};

struct archos_compass_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_compass_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_COMPASS	0x4e16

struct archos_camera_conf
{
	struct archos_gpio reset;
};

struct archos_camera_config
{
	int	nrev;	/* number of hardware revisions */
	struct archos_camera_conf rev[MAX_HWREVS]; /* config data indexed by revision */
};
#define ARCHOS_TAG_CAMERA	0x4e17

struct archos_core_platform_data {
	struct gpio_keys_platform_data *gpio_keys;
	int *codes;
	int ncodes;
};


struct archos_hdmi_conf {
	struct archos_gpio hdmi_irq;
	struct archos_gpio hdmi_dac;
	struct archos_gpio disp_sel;
};

struct archos_extdac_conf {
	struct archos_gpio hdmi_dac;
	struct archos_gpio disp_sel;
	struct archos_gpio cpld_aux;
};

extern unsigned int hardware_rev;

extern void usbsata_power(int on_off);
extern void usbhdd_power(int on_off);

extern int archos_set_usb_id(int on_off);

extern int __init archos_accel_init(struct mma7456l_pdata *pdata);

extern int __init archos_usb2sata_init(void); 
extern int __init ads7846_dev_init(void);
extern int __init archos_keys_init(void);
extern void __init fixup_archos(struct machine_desc *,
		struct tag *, char **, struct meminfo *);

extern void archos_power_off(void);

extern int __init panel_samsung_wvga_48_init(struct omap_dss_platform_data *);
extern int __init panel_tpo_wvga_48_init(struct omap_dss_platform_data *);
extern int __init panel_cpt_wvga_48_init(struct omap_dss_platform_data *);
extern int __init archos_tvout_venc_init(struct omap_dss_platform_data *);
extern int __init archos_tvout_hdmi_init(struct omap_dss_platform_data *);
extern int __init archos_tvout_extdac_init(struct omap_dss_platform_data *);

extern int __init archos_wifi_bt_init(void);

extern void archos_wifi_set_power(int enable);
extern void archos_wifi_set_carddetect(int detect);
extern int archos_wifi_get_irqnr(void);
extern void archos_bt_set_power(int enable);
extern int __init archos_usbhdd_init(void);
extern void __init archos_flash_init(void);

extern int __init archosg6_init(void);
extern int __init archosg7_init(void);

extern int __init archos_atmega_init(void);

extern int __init archos_vibrator_init(void);

extern int __init archos_compass_init(void);

extern int __init archos_camera_init(void);

static inline int machine_has_micro_sd(void) 
{
#ifdef CONFIG_MACH_ARCHOS
	return (machine_is_archos_a5s() || machine_is_archos_a5sg() ||
		 machine_is_archos_a5sc() || machine_is_archos_a5st() ||
		 machine_is_archos_a5sgw() || machine_is_archos_a5gcam());
#else
	return 0;
#endif
}
#endif /*  __MACH_BOARD_ARCHOS_H */

