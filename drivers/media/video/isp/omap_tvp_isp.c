/*
 *  omap_tvp_isp.c -- Archos Glue : Omap3 <--> TVP514x driver
 *
 * Copyright 2009 Archos
 * Author: Jean-Christophe RONA
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
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <media/v4l2-int-device.h>
#include <media/tvp514x.h>

/* include V4L2 camera driver related header file */
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>

#define OMAP_TVP_ISP_DEV_NAME	"omap-tvp-isp"

#define BOARD_I2C_BUSNUM		3

/* I2C address of chips present in board (0xb8 >> 1) */
#define TVP5146_I2C_ADDR		(0x5c)

/* Bit defines for Bus Control 1 register */
#define TVP5146_EN_SHIFT		(0x0000u)
#define TVP5146_EN_MASK			(1u << TVP5146_EN_SHIFT)

#define CAMERA_SENSOR_EN_SHIFT		(0x0008u)
#define CAMERA_SENSOR_EN_MASK		(1u << CAMERA_SENSOR_EN_SHIFT)

struct i2c_client *tvp5146_client;

#define CONFIG_HELMETCAM

#ifdef CONFIG_HELMETCAM
struct i2c_client *helmet_client;

#define HELMET_I2C_ADDR		(0x2e)
#endif

static struct omap34xxcam_hw_config decoder_hwc = {
	.dev_index = 0,
	.dev_minor = 0,
	.dev_type = OMAP34XXCAM_SLAVE_SENSOR,
	.u.sensor.xclk = OMAP34XXCAM_XCLK_NONE,
	.u.sensor.sensor_isp = 1,
};

static struct isp_interface_config tvp5146_if_config = {
	.ccdc_par_ser = ISP_PARLL_YUV_BT,
	.dataline_shift = 0x2,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.vdint0_timing = 0x0,
	.vdint1_timing = 0x0,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.u.par.par_bridge = 0x0,
	.u.par.par_clk_pol = 0x0,
};

static struct v4l2_ifparm ifparm = {
	.if_type = V4L2_IF_TYPE_BT656,
	.u = {
	      .bt656 = {
			.frame_start_on_rising_vs = 1,
			.bt_sync_correct = 0,
			.swap = 0,
			.latch_clk_inv = 0,
			.nobt_hs_inv = 0,	/* active high */
			.nobt_vs_inv = 0,	/* active high */
			.mode = V4L2_IF_TYPE_BT656_MODE_BT_8BIT,
			.clock_min = TVP514X_XCLK_BT656,
			.clock_max = TVP514X_XCLK_BT656,
			},
	      },
};

/**
 * @brief tvp5146_ifparm - Returns the TVP5146 decoder interface parameters
 *
 * @param p - pointer to v4l2_ifparm structure
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_ifparm(struct v4l2_ifparm *p)
{
	if (p == NULL)
		return -EINVAL;

	*p = ifparm;
	return 0;
}

/**
 * @brief tvp5146_set_prv_data - Returns tvp5146 omap34xx driver private data
 *
 * @param priv - pointer to omap34xxcam_hw_config structure
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	if (priv == NULL)
		return -EINVAL;

	hwc->u.sensor.sensor_isp = decoder_hwc.u.sensor.sensor_isp;
	hwc->u.sensor.xclk = decoder_hwc.u.sensor.xclk;
	hwc->dev_index = decoder_hwc.dev_index;
	hwc->dev_minor = decoder_hwc.dev_minor;
	hwc->dev_type = decoder_hwc.dev_type;
	return 0;
}

/**
 * @brief tvp5146_power_set - Power-on or power-off TVP5146 device
 *
 * @param power - enum, Power on/off, resume/standby
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_power_set(enum v4l2_power power)
{
	switch (power) {
	case V4L2_POWER_OFF:
		if (isp_free_interface(ISP_PARLL_YUV_BT))
			return -ENODEV;
		break;

	case V4L2_POWER_STANDBY:
		break;

	case V4L2_POWER_ON:
		if (isp_request_interface(ISP_PARLL_YUV_BT))
			return -ENODEV;

		isp_configure_interface(&tvp5146_if_config);
		break;

	default:
		return -ENODEV;
		break;
	}
	return 0;
}

static struct tvp514x_platform_data tvp5146_pdata = {
	.master = "omap34xxcam",
	.power_set = tvp5146_power_set,
	.priv_data_set = tvp5146_set_prv_data,
	.ifparm = tvp5146_ifparm,

	/* Some interface dependent params */
	.clk_polarity = 0, /* data clocked out on falling edge */
	.hs_polarity = 0, /* 0 - Active low, 1- Active high */
	.vs_polarity = 0, /* 0 - Active low, 1- Active high */
};

static struct i2c_board_info __initdata tvp5146_i2c_board_info = {
	I2C_BOARD_INFO("tvp5146m2", TVP5146_I2C_ADDR),
	.platform_data = &tvp5146_pdata,
};

#ifdef CONFIG_HELMETCAM
static struct i2c_board_info __initdata helmet_i2c_board_info = {
	I2C_BOARD_INFO("helmetcam", HELMET_I2C_ADDR),
	.platform_data = &tvp5146_pdata,
};
#endif

static int __init omap_tvp_isp_probe(struct platform_device *pdev)
{
	struct i2c_adapter *adap;

#if 0
	pdata = (struct omap_pwm_timer*) pdev->dev.platform_data;
	if (pdata == NULL) {
		printk(KERN_ERR "omap_tvp_isp_probe: No platform data !\n");
		return -ENOMEM;
	}
#endif

	adap = i2c_get_adapter(BOARD_I2C_BUSNUM);
	if ( !(tvp5146_client = i2c_new_device(adap, &tvp5146_i2c_board_info)) ) {
		printk(KERN_DEBUG "omap_tvp_isp_probe: TVP514x I2C Registration failed \n");
		return -ENODEV;
	}

#ifdef CONFIG_HELMETCAM
	if ( !(helmet_client = i2c_new_device(adap, &helmet_i2c_board_info)) ) {
		printk( "omap_tvp_isp_probe: HELMET I2C Registration failed \n");
		return -ENODEV;
	}
#endif
	return 0;
}

static int omap_tvp_isp_remove(struct platform_device *pdev)
{
	i2c_unregister_device(tvp5146_client);
#ifdef CONFIG_HELMETCAM
	i2c_unregister_device(helmet_client);
#endif
	return 0;
}

static struct platform_driver omap_tvp_isp_driver = {
	.probe		= omap_tvp_isp_probe,
	.remove		= omap_tvp_isp_remove,
	.driver		= {
		.name	= OMAP_TVP_ISP_DEV_NAME,
	},
};

static int __devinit omap_tvp_isp_init_module(void)
{
	return platform_driver_register(&omap_tvp_isp_driver);
}

static void __exit omap_tvp_isp_exit_module(void)
{
	platform_driver_unregister(&omap_tvp_isp_driver);
}


module_init( omap_tvp_isp_init_module );
module_exit( omap_tvp_isp_exit_module );

/* Module information */
MODULE_AUTHOR("Jean-Christophe RONA, rona@archos.com");
MODULE_DESCRIPTION("Archos Glue : Omap3 <--> TVP514x driver");
MODULE_LICENSE("GPL");
