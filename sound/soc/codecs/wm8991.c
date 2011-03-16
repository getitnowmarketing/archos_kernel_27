/*
 * wm8991.c  --  WM8991 ALSA Soc Audio driver
 *
 * Copyright 2007, 2008 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>

#include "wm8991.h"

#define AUDIO_NAME "wm8991"
#define WM8991_VERSION "0.01"

/* codec private data */
struct wm8991_priv {
	unsigned int sysclk;
	unsigned int pcmclk;
};

/*
 * wm8991 register cache
 * We can't read the WM8991 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8991_reg[] = WM8991_REGISTER_DEFAULTS;

/*
 * read wm8991 register cache
 */
static inline unsigned int wm8991_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg < 1 || reg > (ARRAY_SIZE(wm8991_reg) + 1))
		return -1;
	return cache[reg - 1];
}

/*
 * write wm8991 register cache
 */
static inline void wm8991_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg < 1 || reg > 0x3f)
		return;
	cache[reg - 1] = value;
}

/*
 * write to the wm8991 register space
 */
static int wm8991_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[3];

	data[0] = reg & 0xFF;
	data[1] = (value >> 8) & 0xFF;
	data[2] = value & 0xFF;

	wm8991_write_reg_cache(codec, reg, value);
	if (codec->hw_write(codec->control_data, data, 3) == 2)
		return 0;
	else
		return -EIO;
}

#define wm8991_reset(c) wm8991_write(c, WM8991_RESET, 0)

static const unsigned int rec_mix_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 7, TLV_DB_LINEAR_ITEM(-1500, 600),
};

static const unsigned int in_pga_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 0x1F, TLV_DB_LINEAR_ITEM(-1650, 3000),
};

static const unsigned int out_mix_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 7, TLV_DB_LINEAR_ITEM(0, -2100),
};

static const unsigned int out_pga_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 127, TLV_DB_LINEAR_ITEM(-7300, 600),
};

static const unsigned int out_omix_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 7, TLV_DB_LINEAR_ITEM(-600, 0),
};

static const unsigned int out_dac_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 255, TLV_DB_LINEAR_ITEM(-7163, 0),
};

static const unsigned int in_adc_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 255, TLV_DB_LINEAR_ITEM(-7163, 1763),
};

static const unsigned int out_sidetone_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 31, TLV_DB_LINEAR_ITEM(-3600, 0),
};

static int wm899x_outpga_put_volsw_vu(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & 0xff;
	int ret;
	u16 val;

	ret = snd_soc_put_volsw(kcontrol, ucontrol);
	if (ret < 0)
		return ret;

	/* now hit the volume update bits (always bit 8) */
	val = wm8991_read_reg_cache(codec, reg);
	return wm8991_write(codec, reg, val | 0x0100);
}

#define SOC_WM899X_OUTPGA_SINGLE_R_TLV(xname, reg, shift, max, invert,\
					 tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw, .put = wm899x_outpga_put_volsw_vu, \
	.private_value = SOC_SINGLE_VALUE(reg, shift, max, invert) }


static const char *wm8991_digital_sidetone[] =
	{"None", "Left ADC", "Right ADC", "Reserved"};

static const struct soc_enum wm8991_left_digital_sidetone_enum =
SOC_ENUM_SINGLE(WM8991_DIGITAL_SIDE_TONE,
	WM8991_ADC_TO_DACL_SHIFT,
	WM8991_ADC_TO_DACL_MASK,
	wm8991_digital_sidetone);

static const struct soc_enum wm8991_right_digital_sidetone_enum =
SOC_ENUM_SINGLE(WM8991_DIGITAL_SIDE_TONE,
	WM8991_ADC_TO_DACR_SHIFT,
	WM8991_ADC_TO_DACR_MASK,
	wm8991_digital_sidetone);

static const char *wm8991_adcmode[] =
	{"Hi-fi mode", "Voice mode 1", "Voice mode 2", "Voice mode 3"};

static const struct soc_enum wm8991_right_adcmode_enum =
SOC_ENUM_SINGLE(WM8991_ADC_CTRL,
	WM8991_ADC_HPF_CUT_SHIFT,
	WM8991_ADC_HPF_CUT_MASK,
	wm8991_adcmode);

static const struct snd_kcontrol_new wm8991_snd_controls[] = {
/* INMIXL */
SOC_SINGLE("LIN12 PGA Boost", WM8991_INPUT_MIXER3, WM8991_L12MNBST_BIT, 1, 0),
SOC_SINGLE("LIN34 PGA Boost", WM8991_INPUT_MIXER3, WM8991_L34MNBST_BIT, 1, 0),
/* INMIXR */
SOC_SINGLE("RIN12 PGA Boost", WM8991_INPUT_MIXER3, WM8991_R12MNBST_BIT, 1, 0),
SOC_SINGLE("RIN34 PGA Boost", WM8991_INPUT_MIXER3, WM8991_R34MNBST_BIT, 1, 0),

/* LOMIX */
SOC_SINGLE_TLV("LOMIX LIN3 Bypass Volume", WM8991_OUTPUT_MIXER3,
	WM8991_LLI3LOVOL_SHIFT, WM8991_LLI3LOVOL_MASK, 1, out_mix_tlv),
SOC_SINGLE_TLV("LOMIX RIN12 PGA Bypass Volume", WM8991_OUTPUT_MIXER3,
	WM8991_LR12LOVOL_SHIFT, WM8991_LR12LOVOL_MASK, 1, out_mix_tlv),
SOC_SINGLE_TLV("LOMIX LIN12 PGA Bypass Volume", WM8991_OUTPUT_MIXER3,
	WM8991_LL12LOVOL_SHIFT, WM8991_LL12LOVOL_MASK, 1, out_mix_tlv),
SOC_SINGLE_TLV("LOMIX RIN3 Bypass Volume", WM8991_OUTPUT_MIXER5,
	WM8991_LRI3LOVOL_SHIFT, WM8991_LRI3LOVOL_MASK, 1, out_mix_tlv),
SOC_SINGLE_TLV("LOMIX AINRMUX Bypass Volume", WM8991_OUTPUT_MIXER5,
	WM8991_LRBLOVOL_SHIFT, WM8991_LRBLOVOL_MASK, 1, out_mix_tlv),
SOC_SINGLE_TLV("LOMIX AINLMUX Bypass Volume", WM8991_OUTPUT_MIXER5,
	WM8991_LRBLOVOL_SHIFT, WM8991_LRBLOVOL_MASK, 1, out_mix_tlv),

/* ROMIX */
SOC_SINGLE_TLV("ROMIX RIN3 Bypass Volume", WM8991_OUTPUT_MIXER4,
	WM8991_RRI3ROVOL_SHIFT, WM8991_RRI3ROVOL_MASK, 1, out_mix_tlv),
SOC_SINGLE_TLV("ROMIX LIN12 PGA Bypass Volume", WM8991_OUTPUT_MIXER4,
	WM8991_RL12ROVOL_SHIFT, WM8991_RL12ROVOL_MASK, 1, out_mix_tlv),
SOC_SINGLE_TLV("ROMIX RIN12 PGA Bypass Volume", WM8991_OUTPUT_MIXER4,
	WM8991_RR12ROVOL_SHIFT, WM8991_RR12ROVOL_MASK, 1, out_mix_tlv),
SOC_SINGLE_TLV("ROMIX LIN3 Bypass Volume", WM8991_OUTPUT_MIXER6,
	WM8991_RLI3ROVOL_SHIFT, WM8991_RLI3ROVOL_MASK, 1, out_mix_tlv),
SOC_SINGLE_TLV("ROMIX AINLMUX Bypass Volume", WM8991_OUTPUT_MIXER6,
	WM8991_RLBROVOL_SHIFT, WM8991_RLBROVOL_MASK, 1, out_mix_tlv),
SOC_SINGLE_TLV("ROMIX AINRMUX Bypass Volume", WM8991_OUTPUT_MIXER6,
	WM8991_RRBROVOL_SHIFT, WM8991_RRBROVOL_MASK, 1, out_mix_tlv),

/* LOUT */
SOC_WM899X_OUTPGA_SINGLE_R_TLV("LOUT Volume", WM8991_LEFT_OUTPUT_VOLUME,
	WM8991_LOUTVOL_SHIFT, WM8991_LOUTVOL_MASK, 0, out_pga_tlv),
SOC_SINGLE("LOUT ZC", WM8991_LEFT_OUTPUT_VOLUME, WM8991_LOZC_BIT, 1, 0),

/* ROUT */
SOC_WM899X_OUTPGA_SINGLE_R_TLV("ROUT Volume", WM8991_RIGHT_OUTPUT_VOLUME,
	WM8991_ROUTVOL_SHIFT, WM8991_ROUTVOL_MASK, 0, out_pga_tlv),
SOC_SINGLE("ROUT ZC", WM8991_RIGHT_OUTPUT_VOLUME, WM8991_ROZC_BIT, 1, 0),

/* LOPGA */
SOC_WM899X_OUTPGA_SINGLE_R_TLV("LOPGA Volume", WM8991_LEFT_OPGA_VOLUME,
	WM8991_LOPGAVOL_SHIFT, WM8991_LOPGAVOL_MASK, 0, out_pga_tlv),
SOC_SINGLE("LOPGA ZC Switch", WM8991_LEFT_OPGA_VOLUME,
	WM8991_LOPGAZC_BIT, 1, 0),

/* ROPGA */
SOC_WM899X_OUTPGA_SINGLE_R_TLV("ROPGA Volume", WM8991_RIGHT_OPGA_VOLUME,
	WM8991_ROPGAVOL_SHIFT, WM8991_ROPGAVOL_MASK, 0, out_pga_tlv),
SOC_SINGLE("ROPGA ZC Switch", WM8991_RIGHT_OPGA_VOLUME,
	WM8991_ROPGAZC_BIT, 1, 0),

SOC_SINGLE("LON Mute Switch", WM8991_LINE_OUTPUTS_VOLUME,
	WM8991_LONMUTE_BIT, 1, 0),
SOC_SINGLE("LOP Mute Switch", WM8991_LINE_OUTPUTS_VOLUME,
	WM8991_LOPMUTE_BIT, 1, 0),
SOC_SINGLE("LOP Attenuation Switch", WM8991_LINE_OUTPUTS_VOLUME,
	WM8991_LOATTN_BIT, 1, 0),
SOC_SINGLE("RON Mute Switch", WM8991_LINE_OUTPUTS_VOLUME,
	WM8991_RONMUTE_BIT, 1, 0),
SOC_SINGLE("ROP Mute Switch", WM8991_LINE_OUTPUTS_VOLUME,
	WM8991_ROPMUTE_BIT, 1, 0),
SOC_SINGLE("ROP Attenuation Switch", WM8991_LINE_OUTPUTS_VOLUME,
	WM8991_ROATTN_BIT, 1, 0),

SOC_SINGLE("OUT3 Mute Switch", WM8991_OUT3_4_VOLUME,
	WM8991_OUT3MUTE_BIT, 1, 0),
SOC_SINGLE("OUT3 Attenuation Switch", WM8991_OUT3_4_VOLUME,
	WM8991_OUT3ATTN_BIT, 1, 0),

SOC_SINGLE("OUT4 Mute Switch", WM8991_OUT3_4_VOLUME,
	WM8991_OUT4MUTE_BIT, 1, 0),
SOC_SINGLE("OUT4 Attenuation Switch", WM8991_OUT3_4_VOLUME,
	WM8991_OUT4ATTN_BIT, 1, 0),

SOC_SINGLE("Speaker Mode Switch", WM8991_CLASSD1,
	WM8991_CDMODE_BIT, 1, 0),

SOC_SINGLE("Speaker Output Attenuation Volume", WM8991_SPEAKER_VOLUME,
	WM8991_SPKVOL_SHIFT, WM8991_SPKVOL_MASK, 0),
SOC_SINGLE("Speaker DC Boost Volume", WM8991_CLASSD3,
	WM8991_DCGAIN_SHIFT, WM8991_DCGAIN_MASK, 0),
SOC_SINGLE("Speaker AC Boost Volume", WM8991_CLASSD3,
	WM8991_ACGAIN_SHIFT, WM8991_ACGAIN_MASK, 0),

SOC_WM899X_OUTPGA_SINGLE_R_TLV("Left DAC Digital Volume",
	WM8991_LEFT_DAC_DIGITAL_VOLUME,
	WM8991_DACL_VOL_SHIFT,
	WM8991_DACL_VOL_MASK,
	0,
	out_dac_tlv),

SOC_WM899X_OUTPGA_SINGLE_R_TLV("Right DAC Digital Volume",
	WM8991_RIGHT_DAC_DIGITAL_VOLUME,
	WM8991_DACR_VOL_SHIFT,
	WM8991_DACR_VOL_MASK,
	0,
	out_dac_tlv),

SOC_ENUM("Left Digital Sidetone", wm8991_left_digital_sidetone_enum),
SOC_ENUM("Right Digital Sidetone", wm8991_right_digital_sidetone_enum),

SOC_SINGLE_TLV("Left Digital Sidetone Volume", WM8991_DIGITAL_SIDE_TONE,
	WM8991_ADCL_DAC_SVOL_SHIFT, WM8991_ADCL_DAC_SVOL_MASK, 0,
	out_sidetone_tlv),
SOC_SINGLE_TLV("Right Digital Sidetone Volume", WM8991_DIGITAL_SIDE_TONE,
	WM8991_ADCR_DAC_SVOL_SHIFT, WM8991_ADCR_DAC_SVOL_MASK, 0,
	out_sidetone_tlv),

SOC_SINGLE("ADC Digital High Pass Filter Switch", WM8991_ADC_CTRL,
	WM8991_ADC_HPF_ENA_BIT, 1, 0),

SOC_ENUM("ADC HPF Mode", wm8991_right_adcmode_enum),

SOC_WM899X_OUTPGA_SINGLE_R_TLV("Left ADC Digital Volume",
	WM8991_LEFT_ADC_DIGITAL_VOLUME,
	WM8991_ADCL_VOL_SHIFT,
	WM8991_ADCL_VOL_MASK,
	0,
	in_adc_tlv),

SOC_WM899X_OUTPGA_SINGLE_R_TLV("Right ADC Digital Volume",
	WM8991_RIGHT_ADC_DIGITAL_VOLUME,
	WM8991_ADCR_VOL_SHIFT,
	WM8991_ADCR_VOL_MASK,
	0,
	in_adc_tlv),

SOC_WM899X_OUTPGA_SINGLE_R_TLV("LIN12 Volume",
	WM8991_LEFT_LINE_INPUT_1_2_VOLUME,
	WM8991_LIN12VOL_SHIFT,
	WM8991_LIN12VOL_MASK,
	0,
	in_pga_tlv),

SOC_SINGLE("LIN12 ZC Switch", WM8991_LEFT_LINE_INPUT_1_2_VOLUME,
	WM8991_LI12ZC_BIT, 1, 0),

SOC_SINGLE("LIN12 Mute Switch", WM8991_LEFT_LINE_INPUT_1_2_VOLUME,
	WM8991_LI12MUTE_BIT, 1, 0),

SOC_WM899X_OUTPGA_SINGLE_R_TLV("LIN34 Volume",
	WM8991_LEFT_LINE_INPUT_3_4_VOLUME,
	WM8991_LIN34VOL_SHIFT,
	WM8991_LIN34VOL_MASK,
	0,
	in_pga_tlv),

SOC_SINGLE("LIN34 ZC Switch", WM8991_LEFT_LINE_INPUT_3_4_VOLUME,
	WM8991_LI34ZC_BIT, 1, 0),

SOC_SINGLE("LIN34 Mute Switch", WM8991_LEFT_LINE_INPUT_3_4_VOLUME,
	WM8991_LI34MUTE_BIT, 1, 0),

SOC_WM899X_OUTPGA_SINGLE_R_TLV("RIN12 Volume",
	WM8991_RIGHT_LINE_INPUT_1_2_VOLUME,
	WM8991_RIN12VOL_SHIFT,
	WM8991_RIN12VOL_MASK,
	0,
	in_pga_tlv),

SOC_SINGLE("RIN12 ZC Switch", WM8991_RIGHT_LINE_INPUT_1_2_VOLUME,
	WM8991_RI12ZC_BIT, 1, 0),

SOC_SINGLE("RIN12 Mute Switch", WM8991_RIGHT_LINE_INPUT_1_2_VOLUME,
	WM8991_RI12MUTE_BIT, 1, 0),

SOC_WM899X_OUTPGA_SINGLE_R_TLV("RIN34 Volume",
	WM8991_RIGHT_LINE_INPUT_3_4_VOLUME,
	WM8991_RIN34VOL_SHIFT,
	WM8991_RIN34VOL_MASK,
	0,
	in_pga_tlv),

SOC_SINGLE("RIN34 ZC Switch", WM8991_RIGHT_LINE_INPUT_3_4_VOLUME,
	WM8991_RI34ZC_BIT, 1, 0),

SOC_SINGLE("RIN34 Mute Switch", WM8991_RIGHT_LINE_INPUT_3_4_VOLUME,
	WM8991_RI34MUTE_BIT, 1, 0),

};

/* add non dapm controls */
static int wm8991_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8991_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm8991_snd_controls[i], codec,
					NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

/*
 * _DAPM_ Controls
 */

static int inmixer_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	u16 reg, fakepower;

	reg = wm8991_read_reg_cache(w->codec, WM8991_POWER_MANAGEMENT_2);
	fakepower = wm8991_read_reg_cache(w->codec, WM8991_INTDRIVBITS);

	if (fakepower & ((1 << WM8991_INMIXL_PWR_BIT) |
		(1 << WM8991_AINLMUX_PWR_BIT))) {
		reg |= WM8991_AINL_ENA;
	} else {
		reg &= ~WM8991_AINL_ENA;
	}

	if (fakepower & ((1 << WM8991_INMIXR_PWR_BIT) |
		(1 << WM8991_AINRMUX_PWR_BIT))) {
		reg |= WM8991_AINR_ENA;
	} else {
		reg &= ~WM8991_AINL_ENA;
	}
	wm8991_write(w->codec, WM8991_POWER_MANAGEMENT_2, reg);

	return 0;
}

static int outmixer_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	u32 reg_shift = kcontrol->private_value & 0xfff;
	int ret = 0;
	u16 reg;

	switch (reg_shift) {
	case WM8991_SPEAKER_MIXER | (WM8991_LDSPK_BIT << 8):
		reg = wm8991_read_reg_cache(w->codec, WM8991_OUTPUT_MIXER1);
		if (reg & WM8991_LDLO) {
			printk(KERN_WARNING
			"Cannot set as Output Mixer 1 LDLO Set\n");
			ret = -1;
		}
		break;

	case WM8991_SPEAKER_MIXER | (WM8991_RDSPK_BIT << 8):
		reg = wm8991_read_reg_cache(w->codec, WM8991_OUTPUT_MIXER2);
		if (reg & WM8991_RDRO) {
			printk(KERN_WARNING
			"Cannot set as Output Mixer 2 RDRO Set\n");
			ret = -1;
		}
		break;

	case WM8991_OUTPUT_MIXER1 | (WM8991_LDLO_BIT << 8):
		reg = wm8991_read_reg_cache(w->codec, WM8991_SPEAKER_MIXER);
		if (reg & WM8991_LDSPK) {
			printk(KERN_WARNING
			"Cannot set as Speaker Mixer LDSPK Set\n");
			ret = -1;
		}
		break;

	case WM8991_OUTPUT_MIXER2 | (WM8991_RDRO_BIT << 8):
		reg = wm8991_read_reg_cache(w->codec, WM8991_SPEAKER_MIXER);
		if (reg & WM8991_RDSPK) {
			printk(KERN_WARNING
			"Cannot set as Speaker Mixer RDSPK Set\n");
			ret = -1;
		}
		break;

	default:
		BUG();
	}

	return ret;
}

/* INMIX dB values */
static const unsigned int in_mix_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 7, TLV_DB_LINEAR_ITEM(-1200, 600),
};

/* Left In PGA Connections */
static const struct snd_kcontrol_new wm8991_dapm_lin12_pga_controls[] = {
SOC_DAPM_SINGLE("LIN1 Switch", WM8991_INPUT_MIXER2, WM8991_LMN1_BIT, 1, 0),
SOC_DAPM_SINGLE("LIN2 Switch", WM8991_INPUT_MIXER2, WM8991_LMP2_BIT, 1, 0),
};

static const struct snd_kcontrol_new wm8991_dapm_lin34_pga_controls[] = {
SOC_DAPM_SINGLE("LIN3 Switch", WM8991_INPUT_MIXER2, WM8991_LMN3_BIT, 1, 0),
SOC_DAPM_SINGLE("LIN4 Switch", WM8991_INPUT_MIXER2, WM8991_LMP4_BIT, 1, 0),
};

/* Right In PGA Connections */
static const struct snd_kcontrol_new wm8991_dapm_rin12_pga_controls[] = {
SOC_DAPM_SINGLE("RIN1 Switch", WM8991_INPUT_MIXER2, WM8991_RMN1_BIT, 1, 0),
SOC_DAPM_SINGLE("RIN2 Switch", WM8991_INPUT_MIXER2, WM8991_RMP2_BIT, 1, 0),
};

static const struct snd_kcontrol_new wm8991_dapm_rin34_pga_controls[] = {
SOC_DAPM_SINGLE("RIN3 Switch", WM8991_INPUT_MIXER2, WM8991_RMN3_BIT, 1, 0),
SOC_DAPM_SINGLE("RIN4 Switch", WM8991_INPUT_MIXER2, WM8991_RMP4_BIT, 1, 0),
};

/* INMIXL */
static const struct snd_kcontrol_new wm8991_dapm_inmixl_controls[] = {
SOC_DAPM_SINGLE_TLV("Record Left Volume", WM8991_INPUT_MIXER3,
	WM8991_LDBVOL_SHIFT, WM8991_LDBVOL_MASK, 0, in_mix_tlv),
SOC_DAPM_SINGLE_TLV("LIN2 Volume", WM8991_INPUT_MIXER5, WM8991_LI2BVOL_SHIFT,
	7, 0, in_mix_tlv),
SOC_DAPM_SINGLE("LINPGA12 Switch", WM8991_INPUT_MIXER3, WM8991_L12MNB_BIT,
		1, 0),
SOC_DAPM_SINGLE("LINPGA34 Switch", WM8991_INPUT_MIXER3, WM8991_L34MNB_BIT,
		1, 0),
};

/* INMIXR */
static const struct snd_kcontrol_new wm8991_dapm_inmixr_controls[] = {
SOC_DAPM_SINGLE_TLV("Record Right Volume", WM8991_INPUT_MIXER4,
	WM8991_RDBVOL_SHIFT, WM8991_RDBVOL_MASK, 0, in_mix_tlv),
SOC_DAPM_SINGLE_TLV("RIN2 Volume", WM8991_INPUT_MIXER6, WM8991_RI2BVOL_SHIFT,
	7, 0, in_mix_tlv),
SOC_DAPM_SINGLE("RINPGA12 Switch", WM8991_INPUT_MIXER3, WM8991_L12MNB_BIT,
	1, 0),
SOC_DAPM_SINGLE("RINPGA34 Switch", WM8991_INPUT_MIXER3, WM8991_L34MNB_BIT,
	1, 0),
};

/* AINLMUX */
static const char *wm8991_ainlmux[] =
	{"INMIXL Mix", "RXVOICE Mix", "DIFFINL Mix"};

static const struct soc_enum wm8991_ainlmux_enum =
SOC_ENUM_SINGLE(WM8991_INPUT_MIXER1, WM8991_AINLMODE_SHIFT,
	ARRAY_SIZE(wm8991_ainlmux), wm8991_ainlmux);

static const struct snd_kcontrol_new wm8991_dapm_ainlmux_controls =
SOC_DAPM_ENUM("Route", wm8991_ainlmux_enum);

/* DIFFINL */

/* AINRMUX */
static const char *wm8991_ainrmux[] =
	{"INMIXR Mix", "RXVOICE Mix", "DIFFINR Mix"};

static const struct soc_enum wm8991_ainrmux_enum =
SOC_ENUM_SINGLE(WM8991_INPUT_MIXER1, WM8991_AINRMODE_SHIFT,
	ARRAY_SIZE(wm8991_ainrmux), wm8991_ainrmux);

static const struct snd_kcontrol_new wm8991_dapm_ainrmux_controls =
SOC_DAPM_ENUM("Route", wm8991_ainrmux_enum);

/* RXVOICE */
static const struct snd_kcontrol_new wm8991_dapm_rxvoice_controls[] = {
SOC_DAPM_SINGLE_TLV("LIN4/RXN", WM8991_INPUT_MIXER5, WM8991_LR4BVOL_SHIFT,
			WM8991_LR4BVOL_MASK, 0, in_mix_tlv),
SOC_DAPM_SINGLE_TLV("RIN4/RXP", WM8991_INPUT_MIXER6, WM8991_RL4BVOL_SHIFT,
			WM8991_RL4BVOL_MASK, 0, in_mix_tlv),
};

/* LOMIX */
static const struct snd_kcontrol_new wm8991_dapm_lomix_controls[] = {
SOC_DAPM_SINGLE("LOMIX Right ADC Bypass Switch", WM8991_OUTPUT_MIXER1,
	WM8991_LRBLO_BIT, 1, 0),
SOC_DAPM_SINGLE("LOMIX Left ADC Bypass Switch", WM8991_OUTPUT_MIXER1,
	WM8991_LLBLO_BIT, 1, 0),
SOC_DAPM_SINGLE("LOMIX RIN3 Bypass Switch", WM8991_OUTPUT_MIXER1,
	WM8991_LRI3LO_BIT, 1, 0),
SOC_DAPM_SINGLE("LOMIX LIN3 Bypass Switch", WM8991_OUTPUT_MIXER1,
	WM8991_LLI3LO_BIT, 1, 0),
SOC_DAPM_SINGLE("LOMIX RIN12 PGA Bypass Switch", WM8991_OUTPUT_MIXER1,
	WM8991_LR12LO_BIT, 1, 0),
SOC_DAPM_SINGLE("LOMIX LIN12 PGA Bypass Switch", WM8991_OUTPUT_MIXER1,
	WM8991_LL12LO_BIT, 1, 0),
SOC_DAPM_SINGLE("LOMIX Left DAC Switch", WM8991_OUTPUT_MIXER1,
	WM8991_LDLO_BIT, 1, 0),
};

/* ROMIX */
static const struct snd_kcontrol_new wm8991_dapm_romix_controls[] = {
SOC_DAPM_SINGLE("ROMIX Left ADC Bypass Switch", WM8991_OUTPUT_MIXER2,
	WM8991_RLBRO_BIT, 1, 0),
SOC_DAPM_SINGLE("ROMIX Right ADC Bypass Switch", WM8991_OUTPUT_MIXER2,
	WM8991_RRBRO_BIT, 1, 0),
SOC_DAPM_SINGLE("ROMIX LIN3 Bypass Switch", WM8991_OUTPUT_MIXER2,
	WM8991_RLI3RO_BIT, 1, 0),
SOC_DAPM_SINGLE("ROMIX RIN3 Bypass Switch", WM8991_OUTPUT_MIXER2,
	WM8991_RRI3RO_BIT, 1, 0),
SOC_DAPM_SINGLE("ROMIX LIN12 PGA Bypass Switch", WM8991_OUTPUT_MIXER2,
	WM8991_RL12RO_BIT, 1, 0),
SOC_DAPM_SINGLE("ROMIX RIN12 PGA Bypass Switch", WM8991_OUTPUT_MIXER2,
	WM8991_RR12RO_BIT, 1, 0),
SOC_DAPM_SINGLE("ROMIX Right DAC Switch", WM8991_OUTPUT_MIXER2,
	WM8991_RDRO_BIT, 1, 0),
};

/* LONMIX */
static const struct snd_kcontrol_new wm8991_dapm_lonmix_controls[] = {
SOC_DAPM_SINGLE("LONMIX Left Mixer PGA Switch", WM8991_LINE_MIXER1,
	WM8991_LLOPGALON_BIT, 1, 0),
SOC_DAPM_SINGLE("LONMIX Right Mixer PGA Switch", WM8991_LINE_MIXER1,
	WM8991_LROPGALON_BIT, 1, 0),
SOC_DAPM_SINGLE("LONMIX Inverted LOP Switch", WM8991_LINE_MIXER1,
	WM8991_LOPLON_BIT, 1, 0),
};

/* LOPMIX */
static const struct snd_kcontrol_new wm8991_dapm_lopmix_controls[] = {
SOC_DAPM_SINGLE("LOPMIX Right Mic Bypass Switch", WM8991_LINE_MIXER1,
	WM8991_LR12LOP_BIT, 1, 0),
SOC_DAPM_SINGLE("LOPMIX Left Mic Bypass Switch", WM8991_LINE_MIXER1,
	WM8991_LL12LOP_BIT, 1, 0),
SOC_DAPM_SINGLE("LOPMIX Left Mixer PGA Switch", WM8991_LINE_MIXER1,
	WM8991_LLOPGALOP_BIT, 1, 0),
};

/* RONMIX */
static const struct snd_kcontrol_new wm8991_dapm_ronmix_controls[] = {
SOC_DAPM_SINGLE("RONMIX Right Mixer PGA Switch", WM8991_LINE_MIXER2,
	WM8991_RROPGARON_BIT, 1, 0),
SOC_DAPM_SINGLE("RONMIX Left Mixer PGA Switch", WM8991_LINE_MIXER2,
	WM8991_RLOPGARON_BIT, 1, 0),
SOC_DAPM_SINGLE("RONMIX Inverted ROP Switch", WM8991_LINE_MIXER2,
	WM8991_ROPRON_BIT, 1, 0),
};

/* ROPMIX */
static const struct snd_kcontrol_new wm8991_dapm_ropmix_controls[] = {
SOC_DAPM_SINGLE("ROPMIX Left Mic Bypass Switch", WM8991_LINE_MIXER2,
	WM8991_RL12ROP_BIT, 1, 0),
SOC_DAPM_SINGLE("ROPMIX Right Mic Bypass Switch", WM8991_LINE_MIXER2,
	WM8991_RR12ROP_BIT, 1, 0),
SOC_DAPM_SINGLE("ROPMIX Right Mixer PGA Switch", WM8991_LINE_MIXER2,
	WM8991_RROPGAROP_BIT, 1, 0),
};

/* OUT3MIX */
static const struct snd_kcontrol_new wm8991_dapm_out3mix_controls[] = {
SOC_DAPM_SINGLE("OUT3MIX LIN4/RXP Bypass Switch", WM8991_OUT3_4_MIXER,
	WM8991_LI4O3_BIT, 1, 0),
SOC_DAPM_SINGLE("OUT3MIX Left Out PGA Switch", WM8991_OUT3_4_MIXER,
	WM8991_LPGAO3_BIT, 1, 0),
};

/* OUT4MIX */
static const struct snd_kcontrol_new wm8991_dapm_out4mix_controls[] = {
SOC_DAPM_SINGLE("OUT4MIX Right Out PGA Switch", WM8991_OUT3_4_MIXER,
	WM8991_RPGAO4_BIT, 1, 0),
SOC_DAPM_SINGLE("OUT4MIX RIN4/RXP Bypass Switch", WM8991_OUT3_4_MIXER,
	WM8991_RI4O4_BIT, 1, 0),
};

/* SPKMIX */
static const struct snd_kcontrol_new wm8991_dapm_spkmix_controls[] = {
SOC_DAPM_SINGLE("SPKMIX LIN2 Bypass Switch", WM8991_SPEAKER_MIXER,
	WM8991_LI2SPK_BIT, 1, 0),
SOC_DAPM_SINGLE("SPKMIX LADC Bypass Switch", WM8991_SPEAKER_MIXER,
	WM8991_LB2SPK_BIT, 1, 0),
SOC_DAPM_SINGLE("SPKMIX Left Mixer PGA Switch", WM8991_SPEAKER_MIXER,
	WM8991_LOPGASPK_BIT, 1, 0),
SOC_DAPM_SINGLE("SPKMIX Left DAC Switch", WM8991_SPEAKER_MIXER,
	WM8991_LDSPK_BIT, 1, 0),
SOC_DAPM_SINGLE("SPKMIX Right DAC Switch", WM8991_SPEAKER_MIXER,
	WM8991_RDSPK_BIT, 1, 0),
SOC_DAPM_SINGLE("SPKMIX Right Mixer PGA Switch", WM8991_SPEAKER_MIXER,
	WM8991_ROPGASPK_BIT, 1, 0),
SOC_DAPM_SINGLE("SPKMIX RADC Bypass Switch", WM8991_SPEAKER_MIXER,
	WM8991_RL12ROP_BIT, 1, 0),
SOC_DAPM_SINGLE("SPKMIX RIN2 Bypass Switch", WM8991_SPEAKER_MIXER,
	WM8991_RI2SPK_BIT, 1, 0),
};

static const struct snd_soc_dapm_widget wm8991_dapm_widgets[] = {
/* Input Side */
/* Input Lines */
SND_SOC_DAPM_INPUT("LIN1"),
SND_SOC_DAPM_INPUT("LIN2"),
SND_SOC_DAPM_INPUT("LIN3"),
SND_SOC_DAPM_INPUT("LIN4/RXN"),
SND_SOC_DAPM_INPUT("RIN3"),
SND_SOC_DAPM_INPUT("RIN4/RXP"),
SND_SOC_DAPM_INPUT("RIN1"),
SND_SOC_DAPM_INPUT("RIN2"),
SND_SOC_DAPM_INPUT("Internal ADC Source"),

/* DACs */
SND_SOC_DAPM_ADC("Left ADC", "Left Capture", WM8991_POWER_MANAGEMENT_2,
	WM8991_ADCL_ENA_BIT, 0),
SND_SOC_DAPM_ADC("Right ADC", "Right Capture", WM8991_POWER_MANAGEMENT_2,
	WM8991_ADCR_ENA_BIT, 0),

/* Input PGAs */
SND_SOC_DAPM_MIXER("LIN12 PGA", WM8991_POWER_MANAGEMENT_2, WM8991_LIN12_ENA_BIT,
	0, &wm8991_dapm_lin12_pga_controls[0],
	ARRAY_SIZE(wm8991_dapm_lin12_pga_controls)),
SND_SOC_DAPM_MIXER("LIN34 PGA", WM8991_POWER_MANAGEMENT_2, WM8991_LIN34_ENA_BIT,
	0, &wm8991_dapm_lin34_pga_controls[0],
	ARRAY_SIZE(wm8991_dapm_lin34_pga_controls)),
SND_SOC_DAPM_MIXER("RIN12 PGA", WM8991_POWER_MANAGEMENT_2, WM8991_RIN12_ENA_BIT,
	0, &wm8991_dapm_rin12_pga_controls[0],
	ARRAY_SIZE(wm8991_dapm_rin12_pga_controls)),
SND_SOC_DAPM_MIXER("RIN34 PGA", WM8991_POWER_MANAGEMENT_2, WM8991_RIN34_ENA_BIT,
	0, &wm8991_dapm_rin34_pga_controls[0],
	ARRAY_SIZE(wm8991_dapm_rin34_pga_controls)),

/* INMIXL */
SND_SOC_DAPM_MIXER_E("INMIXL", WM8991_INTDRIVBITS, WM8991_INMIXL_PWR_BIT, 0,
	&wm8991_dapm_inmixl_controls[0],
	ARRAY_SIZE(wm8991_dapm_inmixl_controls),
	inmixer_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

/* AINLMUX */
SND_SOC_DAPM_MUX_E("AILNMUX", WM8991_INTDRIVBITS, WM8991_AINLMUX_PWR_BIT, 0,
	&wm8991_dapm_ainlmux_controls, inmixer_event,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

/* INMIXR */
SND_SOC_DAPM_MIXER_E("INMIXR", WM8991_INTDRIVBITS, WM8991_INMIXR_PWR_BIT, 0,
	&wm8991_dapm_inmixr_controls[0],
	ARRAY_SIZE(wm8991_dapm_inmixr_controls),
	inmixer_event, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

/* AINRMUX */
SND_SOC_DAPM_MUX_E("AIRNMUX", WM8991_INTDRIVBITS, WM8991_AINRMUX_PWR_BIT, 0,
	&wm8991_dapm_ainrmux_controls, inmixer_event,
	SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

/* Output Side */
/* DACs */
SND_SOC_DAPM_DAC("Left DAC", "Left Playback", WM8991_POWER_MANAGEMENT_3,
	WM8991_DACL_ENA_BIT, 0),
SND_SOC_DAPM_DAC("Right DAC", "Right Playback", WM8991_POWER_MANAGEMENT_3,
	WM8991_DACR_ENA_BIT, 0),

/* LOMIX */
SND_SOC_DAPM_MIXER_E("LOMIX", WM8991_POWER_MANAGEMENT_3, WM8991_LOMIX_ENA_BIT,
	0, &wm8991_dapm_lomix_controls[0],
	ARRAY_SIZE(wm8991_dapm_lomix_controls),
	outmixer_event, SND_SOC_DAPM_PRE_REG),

/* LONMIX */
SND_SOC_DAPM_MIXER("LONMIX", WM8991_POWER_MANAGEMENT_3, WM8991_LON_ENA_BIT, 0,
	&wm8991_dapm_lonmix_controls[0],
	ARRAY_SIZE(wm8991_dapm_lonmix_controls)),

/* LOPMIX */
SND_SOC_DAPM_MIXER("LOPMIX", WM8991_POWER_MANAGEMENT_3, WM8991_LOP_ENA_BIT, 0,
	&wm8991_dapm_lopmix_controls[0],
	ARRAY_SIZE(wm8991_dapm_lopmix_controls)),

/* OUT3MIX */
SND_SOC_DAPM_MIXER("OUT3MIX", WM8991_POWER_MANAGEMENT_1, WM8991_OUT3_ENA_BIT, 0,
	&wm8991_dapm_out3mix_controls[0],
	ARRAY_SIZE(wm8991_dapm_out3mix_controls)),

/* SPKMIX */
SND_SOC_DAPM_MIXER_E("SPKMIX", WM8991_POWER_MANAGEMENT_1, WM8991_SPK_ENA_BIT, 0,
	&wm8991_dapm_spkmix_controls[0],
	ARRAY_SIZE(wm8991_dapm_spkmix_controls), outmixer_event,
	SND_SOC_DAPM_PRE_REG),

/* OUT4MIX */
SND_SOC_DAPM_MIXER("OUT4MIX", WM8991_POWER_MANAGEMENT_1, WM8991_OUT4_ENA_BIT, 0,
	&wm8991_dapm_out4mix_controls[0],
	ARRAY_SIZE(wm8991_dapm_out4mix_controls)),

/* ROPMIX */
SND_SOC_DAPM_MIXER("ROPMIX", WM8991_POWER_MANAGEMENT_3, WM8991_ROP_ENA_BIT, 0,
	&wm8991_dapm_ropmix_controls[0],
	ARRAY_SIZE(wm8991_dapm_ropmix_controls)),

/* RONMIX */
SND_SOC_DAPM_MIXER("RONMIX", WM8991_POWER_MANAGEMENT_3, WM8991_RON_ENA_BIT, 0,
	&wm8991_dapm_ronmix_controls[0],
	ARRAY_SIZE(wm8991_dapm_ronmix_controls)),

/* ROMIX */
SND_SOC_DAPM_MIXER_E("ROMIX", WM8991_POWER_MANAGEMENT_3, WM8991_ROMIX_ENA_BIT,
	0, &wm8991_dapm_romix_controls[0],
	ARRAY_SIZE(wm8991_dapm_romix_controls),
	outmixer_event, SND_SOC_DAPM_PRE_REG),

/* LOUT PGA */
SND_SOC_DAPM_PGA("LOUT PGA", WM8991_POWER_MANAGEMENT_1, WM8991_LOUT_ENA_BIT, 0,
	NULL, 0),

/* ROUT PGA */
SND_SOC_DAPM_PGA("ROUT PGA", WM8991_POWER_MANAGEMENT_1, WM8991_ROUT_ENA_BIT, 0,
	NULL, 0),

/* LOPGA */
SND_SOC_DAPM_PGA("LOPGA", WM8991_POWER_MANAGEMENT_3, WM8991_LOPGA_ENA_BIT, 0,
	NULL, 0),

/* ROPGA */
SND_SOC_DAPM_PGA("ROPGA", WM8991_POWER_MANAGEMENT_3, WM8991_ROPGA_ENA_BIT, 0,
	NULL, 0),

/* MICBIAS */
SND_SOC_DAPM_MICBIAS("MICBIAS", WM8991_POWER_MANAGEMENT_1,
	WM8991_MICBIAS_ENA_BIT, 0),

SND_SOC_DAPM_OUTPUT("LON"),
SND_SOC_DAPM_OUTPUT("LOP"),
SND_SOC_DAPM_OUTPUT("OUT3"),
SND_SOC_DAPM_OUTPUT("LOUT"),
SND_SOC_DAPM_OUTPUT("SPKN"),
SND_SOC_DAPM_OUTPUT("SPKP"),
SND_SOC_DAPM_OUTPUT("ROUT"),
SND_SOC_DAPM_OUTPUT("OUT4"),
SND_SOC_DAPM_OUTPUT("ROP"),
SND_SOC_DAPM_OUTPUT("RON"),

SND_SOC_DAPM_OUTPUT("Internal DAC Sink"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Make DACs turn on when playing even if not mixed into any outputs */
	{"Internal DAC Sink", NULL, "Left DAC"},
	{"Internal DAC Sink", NULL, "Right DAC"},

	/* Make ADCs turn on when recording even if not mixed from any inputs */
	{"Left ADC", NULL, "Internal ADC Source"},
	{"Right ADC", NULL, "Internal ADC Source"},

	/* Input Side */
	/* LIN12 PGA */
	{"LIN12 PGA", "LIN1 Switch", "LIN1"},
	{"LIN12 PGA", "LIN2 Switch", "LIN2"},
	/* LIN34 PGA */
	{"LIN34 PGA", "LIN3 Switch", "LIN3"},
	{"LIN34 PGA", "LIN4 Switch", "LIN4"},
	/* INMIXL */
	{"INMIXL", "Record Left Volume", "LOMIX"},
	{"INMIXL", "LIN2 Volume", "LIN2"},
	{"INMIXL", "LINPGA12 Switch", "LIN12 PGA"},
	{"INMIXL", "LINPGA34 Switch", "LIN34 PGA"},
	/* AILNMUX */
	{"AILNMUX", "INMIXL Mix", "INMIXL"},
	{"AILNMUX", "DIFFINL Mix", "LIN12PGA"},
	{"AILNMUX", "DIFFINL Mix", "LIN34PGA"},
	{"AILNMUX", "RXVOICE Mix", "LIN4/RXN"},
	{"AILNMUX", "RXVOICE Mix", "RIN4/RXP"},
	/* ADC */
	{"Left ADC", NULL, "AILNMUX"},

	/* RIN12 PGA */
	{"RIN12 PGA", "RIN1 Switch", "RIN1"},
	{"RIN12 PGA", "RIN2 Switch", "RIN2"},
	/* RIN34 PGA */
	{"RIN34 PGA", "RIN3 Switch", "RIN3"},
	{"RIN34 PGA", "RIN4 Switch", "RIN4"},
	/* INMIXL */
	{"INMIXR", "Record Right Volume", "ROMIX"},
	{"INMIXR", "RIN2 Volume", "RIN2"},
	{"INMIXR", "RINPGA12 Switch", "RIN12 PGA"},
	{"INMIXR", "RINPGA34 Switch", "RIN34 PGA"},
	/* AIRNMUX */
	{"AIRNMUX", "INMIXR Mix", "INMIXR"},
	{"AIRNMUX", "DIFFINR Mix", "RIN12PGA"},
	{"AIRNMUX", "DIFFINR Mix", "RIN34PGA"},
	{"AIRNMUX", "RXVOICE Mix", "RIN4/RXN"},
	{"AIRNMUX", "RXVOICE Mix", "RIN4/RXP"},
	/* ADC */
	{"Right ADC", NULL, "AIRNMUX"},

	/* LOMIX */
	{"LOMIX", "LOMIX RIN3 Bypass Switch", "RIN3"},
	{"LOMIX", "LOMIX LIN3 Bypass Switch", "LIN3"},
	{"LOMIX", "LOMIX LIN12 PGA Bypass Switch", "LIN12 PGA"},
	{"LOMIX", "LOMIX RIN12 PGA Bypass Switch", "RIN12 PGA"},
	{"LOMIX", "LOMIX Right ADC Bypass Switch", "AINRMUX"},
	{"LOMIX", "LOMIX Left ADC Bypass Switch", "AINLMUX"},
	{"LOMIX", "LOMIX Left DAC Switch", "Left DAC"},

	/* ROMIX */
	{"ROMIX", "ROMIX RIN3 Bypass Switch", "RIN3"},
	{"ROMIX", "ROMIX LIN3 Bypass Switch", "LIN3"},
	{"ROMIX", "ROMIX LIN12 PGA Bypass Switch", "LIN12 PGA"},
	{"ROMIX", "ROMIX RIN12 PGA Bypass Switch", "RIN12 PGA"},
	{"ROMIX", "ROMIX Right ADC Bypass Switch", "AINRMUX"},
	{"ROMIX", "ROMIX Left ADC Bypass Switch", "AINLMUX"},
	{"ROMIX", "ROMIX Right DAC Switch", "Right DAC"},

	/* SPKMIX */
	{"SPKMIX", "SPKMIX LIN2 Bypass Switch", "LIN2"},
	{"SPKMIX", "SPKMIX RIN2 Bypass Switch", "RIN2"},
	{"SPKMIX", "SPKMIX LADC Bypass Switch", "AINLMUX"},
	{"SPKMIX", "SPKMIX RADC Bypass Switch", "AINRMUX"},
	{"SPKMIX", "SPKMIX Left Mixer PGA Switch", "LOPGA"},
	{"SPKMIX", "SPKMIX Right Mixer PGA Switch", "ROPGA"},
	{"SPKMIX", "SPKMIX Right DAC Switch", "Right DAC"},
	{"SPKMIX", "SPKMIX Left DAC Switch", "Right DAC"},

	/* LONMIX */
	{"LONMIX", "LONMIX Left Mixer PGA Switch", "LOPGA"},
	{"LONMIX", "LONMIX Right Mixer PGA Switch", "ROPGA"},
	{"LONMIX", "LONMIX Inverted LOP Switch", "LOPMIX"},

	/* LOPMIX */
	{"LOPMIX", "LOPMIX Right Mic Bypass Switch", "RIN12 PGA"},
	{"LOPMIX", "LOPMIX Left Mic Bypass Switch", "LIN12 PGA"},
	{"LOPMIX", "LOPMIX Left Mixer PGA Switch", "LOPGA"},

	/* OUT3MIX */
	{"OUT3MIX", "OUT3MIX LIN4/RXP Bypass Switch", "LIN4/RXP"},
	{"OUT3MIX", "OUT3MIX Left Out PGA Switch", "LOPGA"},

	/* OUT4MIX */
	{"OUT4MIX", "OUT4MIX Right Out PGA Switch", "ROPGA"},
	{"OUT4MIX", "OUT4MIX RIN4/RXP Bypass Switch", "RIN4/RXP"},

	/* RONMIX */
	{"RONMIX", "RONMIX Right Mixer PGA Switch", "ROPGA"},
	{"RONMIX", "RONMIX Left Mixer PGA Switch", "LOPGA"},
	{"RONMIX", "RONMIX Inverted ROP Switch", "ROPMIX"},

	/* ROPMIX */
	{"ROPMIX", "ROPMIX Left Mic Bypass Switch", "LIN12 PGA"},
	{"ROPMIX", "ROPMIX Right Mic Bypass Switch", "RIN12 PGA"},
	{"ROPMIX", "ROPMIX Right Mixer PGA Switch", "ROPGA"},

	/* Out Mixer PGAs */
	{"LOPGA", NULL, "LOMIX"},
	{"ROPGA", NULL, "ROMIX"},

	{"LOUT PGA", NULL, "LOMIX"},
	{"ROUT PGA", NULL, "ROMIX"},

	/* Output Pins */
	{"LON", NULL, "LONMIX"},
	{"LOP", NULL, "LOPMIX"},
	{"OUT", NULL, "OUT3MIX"},
	{"LOUT", NULL, "LOUT PGA"},
	{"SPKN", NULL, "SPKMIX"},
	{"ROUT", NULL, "ROUT PGA"},
	{"OUT4", NULL, "OUT4MIX"},
	{"ROP", NULL, "ROPMIX"},
	{"RON", NULL, "RONMIX"},
};

static int wm8991_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, wm8991_dapm_widgets,
				  ARRAY_SIZE(wm8991_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

/* PLL divisors */
struct _pll_div {
	u32 div2;
	u32 n;
	u32 k;
};

/* The size in bits of the pll divide multiplied by 10
 * to allow rounding later */
#define FIXED_PLL_SIZE ((1 << 16) * 10)

static void pll_factors(struct _pll_div *pll_div, unsigned int target,
	unsigned int source)
{
	u64 Kpart;
	unsigned int K, Ndiv, Nmod;


	Ndiv = target / source;
	if (Ndiv < 6) {
		source >>= 1;
		pll_div->div2 = 1;
		Ndiv = target / source;
	} else
		pll_div->div2 = 0;

	if ((Ndiv < 6) || (Ndiv > 12))
		printk(KERN_WARNING
		"WM8991 N value outwith recommended range! N = %d\n", Ndiv);

	pll_div->n = Ndiv;
	Nmod = target % source;
	Kpart = FIXED_PLL_SIZE * (long long)Nmod;

	do_div(Kpart, source);

	K = Kpart & 0xFFFFFFFF;

	/* Check if we need to round */
	if ((K % 10) >= 5)
		K += 5;

	/* Move down to proper range now rounding is done */
	K /= 10;

	pll_div->k = K;
}

static int wm8991_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	u16 reg;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct _pll_div pll_div;

	if (freq_in && freq_out) {
		pll_factors(&pll_div, freq_out * 4, freq_in);

		/* Turn on PLL */
		reg = wm8991_read_reg_cache(codec, WM8991_POWER_MANAGEMENT_2);
		reg |= WM8991_PLL_ENA;
		wm8991_write(codec, WM8991_POWER_MANAGEMENT_2, reg);

		/* sysclk comes from PLL */
		reg = wm8991_read_reg_cache(codec, WM8991_CLOCKING_2);
		wm8991_write(codec, WM8991_CLOCKING_2, reg | WM8991_SYSCLK_SRC);

		/* set up N , fractional mode and pre-divisor if neccessary */
		wm8991_write(codec, WM8991_PLL1, pll_div.n | WM8991_SDM |
			(pll_div.div2 ? WM8991_PRESCALE : 0));
		wm8991_write(codec, WM8991_PLL2, (u8)(pll_div.k>>8));
		wm8991_write(codec, WM8991_PLL3, (u8)(pll_div.k & 0xFF));
	} else {
		/* Turn on PLL */
		reg = wm8991_read_reg_cache(codec, WM8991_POWER_MANAGEMENT_2);
		reg &= ~WM8991_PLL_ENA;
		wm8991_write(codec, WM8991_POWER_MANAGEMENT_2, reg);
	}
	return 0;
}

/*
 * Clock after PLL and dividers
 */
static int wm8991_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct wm8991_priv *wm8991 = codec->private_data;

	wm8991->sysclk = freq;
	return 0;
}

/*
 * Set's ADC and Voice DAC format.
 */
static int wm8991_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 audio1, audio3;

	audio1 = wm8991_read_reg_cache(codec, WM8991_AUDIO_INTERFACE_1);
	audio3 = wm8991_read_reg_cache(codec, WM8991_AUDIO_INTERFACE_3);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		audio3 &= ~WM8991_AIF_MSTR1;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		audio3 |= WM8991_AIF_MSTR1;
		break;
	default:
		return -EINVAL;
	}

	audio1 &= ~WM8991_AIF_FMT_MASK;

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		audio1 |= WM8991_AIF_TMF_I2S;
		audio1 &= ~WM8991_AIF_LRCLK_INV;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		audio1 |= WM8991_AIF_TMF_RIGHTJ;
		audio1 &= ~WM8991_AIF_LRCLK_INV;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		audio1 |= WM8991_AIF_TMF_LEFTJ;
		audio1 &= ~WM8991_AIF_LRCLK_INV;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		audio1 |= WM8991_AIF_TMF_DSP;
		audio1 &= ~WM8991_AIF_LRCLK_INV;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		audio1 |= WM8991_AIF_TMF_DSP | WM8991_AIF_LRCLK_INV;
		break;
	default:
		return -EINVAL;
	}

	wm8991_write(codec, WM8991_AUDIO_INTERFACE_1, audio1);
	wm8991_write(codec, WM8991_AUDIO_INTERFACE_3, audio3);
	return 0;
}

static int wm8991_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	switch (div_id) {
	case WM8991_MCLK_DIV:
		reg = wm8991_read_reg_cache(codec, WM8991_CLOCKING_2) &
			~WM8991_MCLK_DIV_MASK;
		wm8991_write(codec, WM8991_CLOCKING_2, reg | div);
		break;
	case WM8991_DACCLK_DIV:
		reg = wm8991_read_reg_cache(codec, WM8991_CLOCKING_2) &
			~WM8991_DAC_CLKDIV_MASK;
		wm8991_write(codec, WM8991_CLOCKING_2, reg | div);
		break;
	case WM8991_ADCCLK_DIV:
		reg = wm8991_read_reg_cache(codec, WM8991_CLOCKING_2) &
			~WM8991_ADC_CLKDIV_MASK;
		wm8991_write(codec, WM8991_CLOCKING_2, reg | div);
		break;
	case WM8991_BCLK_DIV:
		reg = wm8991_read_reg_cache(codec, WM8991_CLOCKING_1) &
			~WM8991_BCLK_DIV_MASK;
		wm8991_write(codec, WM8991_CLOCKING_1, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * Set PCM DAI bit size and sample rate.
 */
static int wm8991_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u16 audio1 = wm8991_read_reg_cache(codec, WM8991_AUDIO_INTERFACE_1);

	audio1 &= ~WM8991_AIF_WL_MASK;
	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		audio1 |= WM8991_AIF_WL_20BITS;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		audio1 |= WM8991_AIF_WL_24BITS;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		audio1 |= WM8991_AIF_WL_32BITS;
		break;
	}

	wm8991_write(codec, WM8991_AUDIO_INTERFACE_1, audio1);
	return 0;
}

static int wm8991_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

static int wm8991_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	return 0;
}

#define WM8991_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define WM8991_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

/*
 * The WM8991 supports 2 different and mutually exclusive DAI
 * configurations.
 *
 * 1. ADC/DAC on Primary Interface
 * 2. ADC on Primary Interface/DAC on secondary
 */
struct snd_soc_dai wm8991_dai = {
/* ADC/DAC on primary */
	.name = "WM8991 ADC/DAC Primary",
	.id = 1,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8991_RATES,
		.formats = WM8991_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8991_RATES,
		.formats = WM8991_FORMATS,},
	.ops = {
		 .hw_params = wm8991_hw_params,
		 .digital_mute = wm8991_mute,
		 .set_fmt = wm8991_set_dai_fmt,
		 .set_clkdiv = wm8991_set_dai_clkdiv,
		 .set_pll = wm8991_set_dai_pll,
		 .set_sysclk = wm8991_set_dai_sysclk,
	},
};
EXPORT_SYMBOL_GPL(wm8991_dai);

static int wm8991_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	/* we only need to suspend if we are a valid card */
	if (!codec->card)
		return 0;

	wm8991_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int wm8991_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* we only need to resume if we are a valid card */
	if (!codec->card)
		return 0;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8991_reg); i++) {
		if (i + 1 == WM8991_RESET)
			continue;
		data[0] = ((i + 1) << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->hw_write(codec->control_data, data, 2);
	}

	wm8991_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

/*
 * initialise the WM8991 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int wm8991_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	u16 reg;
	int ret = 0;

	codec->name = "WM8991";
	codec->owner = THIS_MODULE;
	codec->read = wm8991_read_reg_cache;
	codec->write = wm8991_write;
	codec->set_bias_level = wm8991_set_bias_level;
	codec->dai = &wm8991_dai;
	codec->num_dai = 2;
	codec->reg_cache_size = ARRAY_SIZE(wm8991_reg);
	codec->reg_cache = kmemdup(wm8991_reg, sizeof(wm8991_reg), GFP_KERNEL);

	if (codec->reg_cache == NULL)
		return -ENOMEM;

	wm8991_reset(codec);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "wm8991: failed to create pcms\n");
		goto pcm_err;
	}

	/* charge output caps */
	wm8991_set_bias_level(codec, SNDRV_CTL_POWER_D2);
	codec->bias_level = SND_SOC_BIAS_STANDBY;

	reg = wm8991_read_reg_cache(codec, WM8991_AUDIO_INTERFACE_4);
	wm8991_write(codec, WM8991_AUDIO_INTERFACE_4, reg | WM8991_ALRCGPIO1);

	reg = wm8991_read_reg_cache(codec, WM8991_GPIO1_GPIO2) &
		~WM8991_GPIO1_SEL_MASK;
	wm8991_write(codec, WM8991_GPIO1_GPIO2, reg | 1);

	reg = wm8991_read_reg_cache(codec, WM8991_POWER_MANAGEMENT_1);
	wm8991_write(codec, WM8991_POWER_MANAGEMENT_1, reg | WM8991_VREF_ENA|
		WM8991_VMID_MODE_MASK);

	reg = wm8991_read_reg_cache(codec, WM8991_POWER_MANAGEMENT_2);
	wm8991_write(codec, WM8991_POWER_MANAGEMENT_2, reg | WM8991_OPCLK_ENA);

	wm8991_write(codec, WM8991_DAC_CTRL, 0);
	wm8991_write(codec, WM8991_LEFT_OUTPUT_VOLUME, 0x50 | (1<<8));
	wm8991_write(codec, WM8991_RIGHT_OUTPUT_VOLUME, 0x50 | (1<<8));

	wm8991_add_controls(codec);
	wm8991_add_widgets(codec);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "wm8991: failed to register card\n");
		goto card_err;
	}
	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

/* If the i2c layer weren't so broken, we could pass this kind of data
   around */
static struct snd_soc_device *wm8991_socdev;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

/*
 * WM891 2 wire address is determined by GPIO5
 * state during powerup.
 *    low  = 0x34
 *    high = 0x36
 */
static unsigned short normal_i2c[] = { 0, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver wm8991_i2c_driver;
static struct i2c_client client_template;

static int wm8991_codec_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_device *socdev = wm8991_socdev;
	struct wm8991_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *i2c;
	int ret;

	if (addr != setup->i2c_address)
		return -ENODEV;

	client_template.adapter = adap;
	client_template.addr = addr;

	i2c =  kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL) {
		kfree(codec);
		return -ENOMEM;
	}
	i2c_set_clientdata(i2c, codec);
	codec->control_data = i2c;

	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		pr_err("failed to attach codec at addr %x\n", addr);
		goto err;
	}

	ret = wm8991_init(socdev);
	if (ret < 0) {
		pr_err("failed to initialise WM8991\n");
		goto err;
	}
	return ret;

err:
	kfree(codec);
	kfree(i2c);
	return ret;
}

static int wm8991_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	i2c_detach_client(client);
	kfree(codec->reg_cache);
	kfree(client);
	return 0;
}

static int wm8991_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, wm8991_codec_probe);
}

/* corgi i2c codec control layer */
static struct i2c_driver wm8991_i2c_driver = {
	.driver = {
		.name = "WM8991 I2C Codec",
		.owner = THIS_MODULE,
	},
	.attach_adapter = wm8991_i2c_attach,
	.detach_client =  wm8991_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "WM8991",
	.driver = &wm8991_i2c_driver,
};
#endif

static int wm8991_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct wm8991_setup_data *setup;
	struct snd_soc_codec *codec;
	struct wm8991_priv *wm8991;
	int ret = 0;

	pr_info("WM8991 Audio Codec %s", WM8991_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	wm8991 = kzalloc(sizeof(struct wm8991_priv), GFP_KERNEL);
	if (wm8991 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = wm8991;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	wm8991_socdev = socdev;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		normal_i2c[0] = setup->i2c_address;
		codec->hw_write = (hw_write_t)i2c_master_send;
		ret = i2c_add_driver(&wm8991_i2c_driver);
		if (ret != 0)
			printk(KERN_ERR "can't add i2c driver");
	}
#else
		/* Add other interfaces here */
#endif
	return ret;
}

/* power down chip */
static int wm8991_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		wm8991_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&wm8991_i2c_driver);
#endif
	kfree(codec->private_data);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wm8991 = {
	.probe = 	wm8991_probe,
	.remove = 	wm8991_remove,
	.suspend = 	wm8991_suspend,
	.resume =	wm8991_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_wm8991);

static int __init wm8991_modinit(void)
{
	return snd_soc_register_dai(&wm8991_dai);
}
module_init(wm8991_modinit);

static void __exit wm8991_exit(void)
{
	snd_soc_unregister_dai(&wm8991_dai);
}
module_exit(wm8991_exit);

MODULE_DESCRIPTION("ASoC WM8991 driver");
MODULE_AUTHOR("Graeme Gregory");
MODULE_LICENSE("GPL");
