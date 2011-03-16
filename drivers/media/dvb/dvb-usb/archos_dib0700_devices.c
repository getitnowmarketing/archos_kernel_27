/* Linux driver for devices based on the DiBcom DiB0700 USB bridge
 *
 *	This program is free software; you can redistribute it and/or modify it
 *	under the terms of the GNU General Public License as published by the Free
 *	Software Foundation, version 2.
 *
 *  Copyright (C) 2005-7 DiBcom, SA
 */
#include "dib0700.h"


/* stripped down version for dib7070p only */

#include "dib7000p.h"
#include "dib0070.h"

static int force_lna_activation;
module_param(force_lna_activation, int, 0644);
MODULE_PARM_DESC(force_lna_activation, "force the activation of Low-Noise-Amplifyer(s) (LNA), "
		"if applicable for the device (default: 0=automatic/off).");

struct dib0700_adapter_state {
	int (*set_param_save) (struct dvb_frontend *, struct dvb_frontend_parameters *);
};


/* DIB7070 generic */
static struct dibx000_agc_config dib7070_agc_config = {
	BAND_UHF | BAND_VHF | BAND_LBAND | BAND_SBAND,
	/* P_agc_use_sd_mod1=0, P_agc_use_sd_mod2=0, P_agc_freq_pwm_div=5, P_agc_inv_pwm1=0, P_agc_inv_pwm2=0,
	 * P_agc_inh_dc_rv_est=0, P_agc_time_est=3, P_agc_freeze=0, P_agc_nb_est=5, P_agc_write=0 */
	(0 << 15) | (0 << 14) | (5 << 11) | (0 << 10) | (0 << 9) | (0 << 8) | (3 << 5) | (0 << 4) | (5 << 1) | (0 << 0), // setup

	600, // inv_gain
	10,  // time_stabiliz

	0,  // alpha_level
	118,  // thlock

	0,     // wbd_inv
	3530,  // wbd_ref
	1,     // wbd_sel
	5,     // wbd_alpha

	65535,  // agc1_max
		0,  // agc1_min

	65535,  // agc2_max
	0,      // agc2_min

	0,      // agc1_pt1
	40,     // agc1_pt2
	183,    // agc1_pt3
	206,    // agc1_slope1
	255,    // agc1_slope2
	72,     // agc2_pt1
	152,    // agc2_pt2
	88,     // agc2_slope1
	90,     // agc2_slope2

	17,  // alpha_mant
	27,  // alpha_exp
	23,  // beta_mant
	51,  // beta_exp

	0,  // perform_agc_softsplit
};

static int dib7070_tuner_reset(struct dvb_frontend *fe, int onoff)
{
	return dib7000p_set_gpio(fe, 8, 0, !onoff);
}

static int dib7070_tuner_sleep(struct dvb_frontend *fe, int onoff)
{
	return dib7000p_set_gpio(fe, 9, 0, onoff);
}

static int dib7070_update_lna(struct dvb_frontend *fe, u16 agc_global)
{
//printk("dib7070_update_lna: %p, %u, pid: %d\n", fe, agc_global);
	// enable LNA on archos dvb-t module
	return dib7000p_set_gpio(fe, 1, 0, 0);
}

static struct dib0070_config dib7070p_dib0070_config[2] = {
	{
		.i2c_address = DEFAULT_DIB0070_I2C_ADDRESS,
		.reset = dib7070_tuner_reset,
		.sleep = dib7070_tuner_sleep,
		.clock_khz = 12000,
		.clock_pad_drive = 4
	}, {
		.i2c_address = DEFAULT_DIB0070_I2C_ADDRESS,
		.reset = dib7070_tuner_reset,
		.sleep = dib7070_tuner_sleep,
		.clock_khz = 12000,

	}
};

static int dib7070_set_param_override(struct dvb_frontend *fe, struct dvb_frontend_parameters *fep)
{
	struct dvb_usb_adapter *adap = fe->dvb->priv;
	struct dib0700_adapter_state *state = adap->priv;

	u16 offset;
	u8 band = BAND_OF_FREQUENCY(fep->frequency/1000);
	switch (band) {
		case BAND_VHF: offset = 950; break;
		case BAND_UHF:
		default: offset = 550; break;
	}
	deb_info("WBD for DiB7000P: %d\n", offset + dib0070_wbd_offset(fe));
	dib7000p_set_wbd_ref(fe, offset + dib0070_wbd_offset(fe));
	return state->set_param_save(fe, fep);
}

static int dib7070p_tuner_attach(struct dvb_usb_adapter *adap)
{
	struct dib0700_adapter_state *st = adap->priv;
	struct i2c_adapter *tun_i2c = dib7000p_get_i2c_master(adap->fe, DIBX000_I2C_INTERFACE_TUNER, 1);

	if (adap->id == 0) {
		if (dvb_attach(dib0070_attach, adap->fe, tun_i2c, &dib7070p_dib0070_config[0]) == NULL)
			return -ENODEV;
	} else {
		if (dvb_attach(dib0070_attach, adap->fe, tun_i2c, &dib7070p_dib0070_config[1]) == NULL)
			return -ENODEV;
	}

	st->set_param_save = adap->fe->ops.tuner_ops.set_params;
	adap->fe->ops.tuner_ops.set_params = dib7070_set_param_override;
	return 0;
}

static int dib7070_2_set_param_override(struct dvb_frontend *fe, struct dvb_frontend_parameters *fep)
{
	struct dvb_usb_adapter *adap = fe->dvb->priv;
	struct dib0700_adapter_state *state = adap->priv;

	u16 offset;
	u8 band = BAND_OF_FREQUENCY(fep->frequency/1000);

	switch (band) {
		case BAND_VHF: offset = 950; break;
		case BAND_UHF:
		default: offset = 550; break;
	}
	deb_info("WBD for DiB7000P_2: %d\n", offset + dib0070_wbd_offset(fe));
	dib7000p_set_wbd_ref(fe, offset + dib0070_wbd_offset(fe));
	return state[1].set_param_save(fe, fep);
}

static struct dibx000_bandwidth_config dib7070_bw_config_12_mhz = {
	60000, 15000, // internal, sampling
	1, 20, 3, 1, 0, // pll_cfg: prediv, ratio, range, reset, bypass
	0, 0, 1, 1, 2, // misc: refdiv, bypclk_div, IO_CLK_en_core, ADClkSrc, modulo
	(3 << 14) | (1 << 12) | (524 << 0), // sad_cfg: refsel, sel, freq_15k
	(0 << 25) | 0, // ifreq = 0.000000 MHz
	20452225, // timf
	12000000, // xtal_hz
};

/* STK7070PD */
static struct dib7000p_config stk7070pd_dib7000p_config[2] = {
	{
		.output_mpeg2_in_188_bytes = 1,

		.agc_config_count = 1,
		.agc = &dib7070_agc_config,
		.bw  = &dib7070_bw_config_12_mhz,
		.tuner_is_baseband = 1,
		.spur_protect = 1,

		.gpio_dir = DIB7000P_GPIO_DEFAULT_DIRECTIONS,
		.gpio_val = DIB7000P_GPIO_DEFAULT_VALUES,
		.gpio_pwm_pos = DIB7000P_GPIO_DEFAULT_PWM_POS,

		.update_lna = dib7070_update_lna,

		.hostbus_diversity = 1,
	}, {
		.output_mpeg2_in_188_bytes = 1,

		.agc_config_count = 1,
		.agc = &dib7070_agc_config,
		.bw  = &dib7070_bw_config_12_mhz,
		.tuner_is_baseband = 1,
		.spur_protect = 1,

		.gpio_dir = DIB7000P_GPIO_DEFAULT_DIRECTIONS,
		.gpio_val = DIB7000P_GPIO_DEFAULT_VALUES,
		.gpio_pwm_pos = DIB7000P_GPIO_DEFAULT_PWM_POS,

		.update_lna = dib7070_update_lna,

		.hostbus_diversity = 1,
	}
};

static struct dvb_usb_adapter *find_diversity_master(struct dvb_usb_adapter *adap)
{
	int i;
	struct dvb_usb_device *dev = adap->dev;
	
	for (i=0; i<dev->num_adapters_initialized; i++)
	{
		if (dev->adapter[i].props.caps & DVB_USB_ADAP_DIVERSITY_MASTER)
			return &dev->adapter[i];
	}
	
	return NULL;
}

static int stk7070pd_frontend_attach0(struct dvb_usb_adapter *adap)
{
	dib0700_set_gpio(adap->dev, GPIO6, GPIO_OUT, 1);
	msleep(10);
	dib0700_set_gpio(adap->dev, GPIO9, GPIO_OUT, 1);
	dib0700_set_gpio(adap->dev, GPIO4, GPIO_OUT, 1);
	dib0700_set_gpio(adap->dev, GPIO7, GPIO_OUT, 1);
	dib0700_set_gpio(adap->dev, GPIO10, GPIO_OUT, 0);

	dib0700_ctrl_clock(adap->dev, 72, 1);

	msleep(10);
	dib0700_set_gpio(adap->dev, GPIO10, GPIO_OUT, 1);
	msleep(10);
	dib0700_set_gpio(adap->dev, GPIO0, GPIO_OUT, 1);

	dib7000p_i2c_enumeration(&adap->dev->i2c_adap, 2, 18, stk7070pd_dib7000p_config);

	adap->fe = dvb_attach(dib7000p_attach, &adap->dev->i2c_adap, 0x80, &stk7070pd_dib7000p_config[0]);

	// add master to list of himself
	if (stk7070pd_dib7000p_config[0].hostbus_diversity) {
		if (dib7000p_attach_slave(adap->fe, adap->fe) < 0) {
			deb_info("attach master failed\n");		
		}
	}

	return adap->fe == NULL ? -ENODEV : 0;
}

static int stk7070pd_frontend_attach1(struct dvb_usb_adapter *adap)
{
	adap->fe = dvb_attach(dib7000p_attach, &adap->dev->i2c_adap, 0x82, &stk7070pd_dib7000p_config[1]);

	// add slave to list of master
	if (stk7070pd_dib7000p_config[1].hostbus_diversity)
	{
		struct dvb_usb_adapter *master = find_diversity_master(adap);
	
		if (master != NULL) {
			if (dib7000p_attach_slave(master->fe, adap->fe) < 0) {
				deb_info("attach slave to master failed\n");		
			}
			// now attach master to slave
			if (dib7000p_attach_slave(adap->fe, master->fe) < 0) {
				deb_info("attach master to slave failed\n");		
			}
			// now attach slave to itself
			if (dib7000p_attach_slave(adap->fe, adap->fe) < 0) {
				deb_info("attach slave to slave failed\n");		
			}
		}
	}

	return adap->fe == NULL ? -ENODEV : 0;
}

/* Archos Gen6 DVB-T stick */
static struct dib7000p_config archosgen6_dib7000p_config[2] = {
	{
		.output_mpeg2_in_188_bytes = 1,

		.agc_config_count = 1,
		.agc = &dib7070_agc_config,
		.bw  = &dib7070_bw_config_12_mhz,
		.tuner_is_baseband = 1,
		.spur_protect = 1,

		.gpio_dir = DIB7000P_GPIO_DEFAULT_DIRECTIONS,
		.gpio_val = DIB7000P_GPIO_DEFAULT_VALUES,
		.gpio_pwm_pos = DIB7000P_GPIO_DEFAULT_PWM_POS,

		.hostbus_diversity = 1,
	}, {
		.output_mpeg2_in_188_bytes = 1,

		.agc_config_count = 1,
		.agc = &dib7070_agc_config,
		.bw  = &dib7070_bw_config_12_mhz,
		.tuner_is_baseband = 1,
		.spur_protect = 1,

		.gpio_dir = DIB7000P_GPIO_DEFAULT_DIRECTIONS,
		.gpio_val = DIB7000P_GPIO_DEFAULT_VALUES,
		.gpio_pwm_pos = DIB7000P_GPIO_DEFAULT_PWM_POS,

		.hostbus_diversity = 1,
	}
};

/* DVB-USB and USB stuff follows */
struct usb_device_id dib0700_usb_id_table[] = {
/* 0 */	{ USB_DEVICE(USB_VID_DIBCOM,    USB_PID_DIBCOM_STK7700P) },
	{ USB_DEVICE(USB_VID_DIBCOM,    USB_PID_DIBCOM_STK7700P_PC) },
	{ USB_DEVICE(USB_VID_HAUPPAUGE, USB_PID_HAUPPAUGE_NOVA_T_500) },
	{ USB_DEVICE(USB_VID_HAUPPAUGE, USB_PID_HAUPPAUGE_NOVA_T_500_2) },
	{ USB_DEVICE(USB_VID_HAUPPAUGE, USB_PID_HAUPPAUGE_NOVA_T_STICK) },
/* 5 */	{ USB_DEVICE(USB_VID_AVERMEDIA, USB_PID_AVERMEDIA_VOLAR) },
	{ USB_DEVICE(USB_VID_COMPRO,    USB_PID_COMPRO_VIDEOMATE_U500) },
	{ USB_DEVICE(USB_VID_UNIWILL,   USB_PID_UNIWILL_STK7700P) },
	{ USB_DEVICE(USB_VID_LEADTEK,   USB_PID_WINFAST_DTV_DONGLE_STK7700P) },
	{ USB_DEVICE(USB_VID_HAUPPAUGE, USB_PID_HAUPPAUGE_NOVA_T_STICK_2) },
/* 10 */{ USB_DEVICE(USB_VID_AVERMEDIA, USB_PID_AVERMEDIA_VOLAR_2) },
	{ USB_DEVICE(USB_VID_PINNACLE,  USB_PID_PINNACLE_PCTV2000E) },
	{ USB_DEVICE(USB_VID_TERRATEC,
			USB_PID_TERRATEC_CINERGY_DT_XS_DIVERSITY) },
	{ USB_DEVICE(USB_VID_HAUPPAUGE, USB_PID_HAUPPAUGE_NOVA_TD_STICK) },
	{ USB_DEVICE(USB_VID_DIBCOM,    USB_PID_DIBCOM_STK7700D) },
/* 15 */{ USB_DEVICE(USB_VID_DIBCOM,    USB_PID_DIBCOM_STK7070P) },
	{ USB_DEVICE(USB_VID_PINNACLE,  USB_PID_PINNACLE_PCTV_DVB_T_FLASH) },
	{ USB_DEVICE(USB_VID_DIBCOM,    USB_PID_DIBCOM_STK7070PD) },
	{ USB_DEVICE(USB_VID_PINNACLE,
			USB_PID_PINNACLE_PCTV_DUAL_DIVERSITY_DVB_T) },
	{ USB_DEVICE(USB_VID_COMPRO,    USB_PID_COMPRO_VIDEOMATE_U500_PC) },
/* 20 */{ USB_DEVICE(USB_VID_AVERMEDIA, USB_PID_AVERMEDIA_EXPRESS) },
	{ USB_DEVICE(USB_VID_GIGABYTE,  USB_PID_GIGABYTE_U7000) },
	{ USB_DEVICE(USB_VID_ULTIMA_ELECTRONIC, USB_PID_ARTEC_T14BR) },
	{ USB_DEVICE(USB_VID_ASUS,      USB_PID_ASUS_U3000) },
	{ USB_DEVICE(USB_VID_ASUS,      USB_PID_ASUS_U3100) },
/* 25 */{ USB_DEVICE(USB_VID_HAUPPAUGE, USB_PID_HAUPPAUGE_NOVA_T_STICK_3) },
	{ USB_DEVICE(USB_VID_HAUPPAUGE, USB_PID_HAUPPAUGE_MYTV_T) },
	{ USB_DEVICE(USB_VID_TERRATEC,  USB_PID_TERRATEC_CINERGY_HT_USB_XE) },
	{ USB_DEVICE(USB_VID_PINNACLE,	USB_PID_PINNACLE_EXPRESSCARD_320CX) },
	{ USB_DEVICE(USB_VID_PINNACLE,	USB_PID_PINNACLE_PCTV72E) },
/* 30 */{ USB_DEVICE(USB_VID_PINNACLE,	USB_PID_PINNACLE_PCTV73E) },
	{ USB_DEVICE(USB_VID_YUAN,	USB_PID_YUAN_EC372S) },
	{ USB_DEVICE(USB_VID_TERRATEC,	USB_PID_TERRATEC_CINERGY_HT_EXPRESS) },
	{ USB_DEVICE(USB_VID_TERRATEC,	USB_PID_TERRATEC_CINERGY_T_XXS) },
	{ USB_DEVICE(USB_VID_LEADTEK,   USB_PID_WINFAST_DTV_DONGLE_STK7700P_2) },
/* 35 */{ USB_DEVICE(USB_VID_HAUPPAUGE, USB_PID_HAUPPAUGE_NOVA_TD_STICK_52009) },
	{ USB_DEVICE(USB_VID_HAUPPAUGE, USB_PID_HAUPPAUGE_NOVA_T_500_3) },
	{ USB_DEVICE(USB_VID_GIGABYTE,  USB_PID_GIGABYTE_U8000) },
	{ USB_DEVICE(USB_VID_YUAN,      USB_PID_YUAN_STK7700PH) },
	{ USB_DEVICE(USB_VID_ASUS,	USB_PID_ASUS_U3000H) },
/* 40 */{ USB_DEVICE(USB_VID_PINNACLE,  USB_PID_PINNACLE_PCTV801E) },
	{ USB_DEVICE(USB_VID_PINNACLE,  USB_PID_PINNACLE_PCTV801E_SE) },
	{ USB_DEVICE(USB_VID_UNIHAN,	USB_PID_UNIHAN_UPT6007DRD_S) },
	{ 0 }		/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, dib0700_usb_id_table);

#define DIB0700_DEFAULT_DEVICE_PROPERTIES \
	.caps              = DVB_USB_IS_AN_I2C_ADAPTER, \
	.usb_ctrl          = DEVICE_SPECIFIC, \
	.firmware          = "dvb-usb-dib0700-1.20.fw", \
	.download_firmware = dib0700_download_firmware, \
	.no_reconnect      = 1, \
	.size_of_priv      = sizeof(struct dib0700_state), \
	.i2c_algo          = &dib0700_i2c_algo, \
	.identify_state    = dib0700_identify_state

#define DIB0700_DEFAULT_STREAMING_CONFIG(ep) \
	.streaming_ctrl   = dib0700_streaming_ctrl, \
	.stream = { \
		.type = USB_BULK, \
		.count = 4, \
		.endpoint = ep, \
		.u = { \
			.bulk = { \
				.buffersize = 39480, \
			} \
		} \
	}

struct dvb_usb_device_properties dib0700_devices[] = {
	{
		DIB0700_DEFAULT_DEVICE_PROPERTIES,

		.num_adapters = 2,
		.adapter = {
			{
				.frontend_attach  = stk7070pd_frontend_attach0,
				.tuner_attach     = dib7070p_tuner_attach,

				DIB0700_DEFAULT_STREAMING_CONFIG(0x02),

				.caps             = DVB_USB_ADAP_DIVERSITY_MASTER,

				.size_of_priv     = sizeof(struct dib0700_adapter_state)*2,
			}, {
				.frontend_attach  = stk7070pd_frontend_attach1,
				.tuner_attach     = dib7070p_tuner_attach,

				DIB0700_DEFAULT_STREAMING_CONFIG(0x03),

				.caps             = DVB_USB_ADAP_DIVERSITY_SLAVE,

				.size_of_priv     = sizeof(struct dib0700_adapter_state)*2,
			}
		},

		.num_device_descs = 5,
		.devices = {
			{   "DiBcom STK7070PD reference design",
				{ &dib0700_usb_id_table[17], NULL },
				{ NULL },
			},
			{   "Pinnacle PCTV Dual DVB-T Diversity Stick",
				{ &dib0700_usb_id_table[18], NULL },
				{ NULL },
			},
			{   "Hauppauge Nova-TD Stick (52009)",
				{ &dib0700_usb_id_table[35], NULL },
				{ NULL },
			},
			{   "Hauppauge Nova-TD-500 (84xxx)",
				{ &dib0700_usb_id_table[36], NULL },
				{ NULL },
			},
			{   "Archos DVB-T Gen7 Stick",
				{ &dib0700_usb_id_table[42], NULL },
				{ NULL },
			}
		}
	},
};

int dib0700_device_count = ARRAY_SIZE(dib0700_devices);
