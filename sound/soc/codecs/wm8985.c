/*
 * wm8985.c  --  WM8985 ALSA Soc Audio driver
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Copyright 2009 Archos SA.
 *
 * Authors:
 * Mike Arthur			<linux@wolfsonmicro.com>
 * Jean-Christophe RONA		<rona@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#include "wm8985.h"

#define AUDIO_NAME "wm8985"
#define WM8985_VERSION "0.3"

/* Define this if you want to enable DAPM (Dynamic Power Management) */
//#define DYNAMIC_POWER

/* Define this if you need some kcontrols from
	non-dynamic mode in dynamic mode (they will appear twice) */
#define COMPATIBILITY

/* codec private data */
struct wm8985_priv {
	/* inclk is the input clock, (not the generated one when the WM8985 is Master) */
	unsigned int inclk;
	/* Volumes will be saved here when needed */
	u16 vol_hp_l;
	u16 vol_hp_r;
	u16 vol_spk_l;
	u16 vol_spk_r;
};

static void wm8985_init_default_pre_power_levels(struct snd_soc_codec *codec);
static void wm8985_init_default_post_power_levels(struct snd_soc_codec *codec);

/*
 * wm8985 register cache
 * We can't read the WM8985 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8985_reg[WM8985_CACHEREGNUM] = {
//  0       1       2       3       4       5       6       7       8       9
    0x0000, 0x0000, 0x0000, 0x0000, 0x0050, 0x0000, 0x0140, 0x0080, 0x0000, 0x0000,
    0x0000, 0x00ff, 0x00ff, 0x0000, 0x0100, 0x00ff, 0x00ff, 0x0000, 0x012c, 0x002c,
    0x002c, 0x002c, 0x002c, 0x0008, 0x0032, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0038, 0x000b, 0x0032, 0x0000, 0x0008, 0x000c, 0x0093, 0x00e9,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0003, 0x0010, 0x0010, 0x0100, 0x0100, 0x0002,
    0x0001, 0x0001, 0x0039, 0x0039, 0x0039, 0x0039, 0x0001, 0x0001, 0x0000, 0x0000,
    0x0000, 0x0000
};

/*
 * read wm8985 register cache
 */
static inline unsigned int wm8985_read_reg_cache(struct snd_soc_codec  *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == WM8985_RESET)
		return 0;
	if (reg >= WM8985_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write wm8985 register cache
 */
static inline void wm8985_write_reg_cache(struct snd_soc_codec  *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WM8985_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the WM8985 register space
 */
static int wm8985_write(struct snd_soc_codec  *codec, unsigned int reg,
	unsigned int value)
{
	int data[2];
	//printk("wm8985_write: reg = 0x%02X, val = 0x%03X\n", reg, value);
	/* data is
	 *   D15..D9 WM8985 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	wm8985_write_reg_cache (codec, reg, value);

	/*
	if (codec->hw_write(codec->control_data, data, 2) == 2) {
		return 0;
	}
	else
		return -1;
	*/

	return i2c_smbus_write_byte_data(codec->control_data, data[0], data[1]);
}

static int wm8985_field_write(struct snd_soc_codec *codec, int reg, unsigned int data, unsigned int mask)
{
	unsigned int next_data;

	next_data = wm8985_read_reg_cache(codec,reg);
	next_data &= (~mask);
	next_data |= data;

	return wm8985_write(codec,reg,next_data);
}

/* Functions needed to power up chip blocs */
void wm8985_power_up_inputs(struct snd_soc_codec *codec, int input_blocs, int on)
{
#if !defined (DYNAMIC_POWER)
	if (input_blocs & INP_BLOC_BOOST) {
		if (on)
			wm8985_field_write(codec, WM8985_POWER2,(BOOSTENR|BOOSTENL),
				(BOOSTENRM|BOOSTENLM));
		else
			wm8985_field_write(codec, WM8985_POWER2,0,(BOOSTENRM|BOOSTENRM));
	}

	if (input_blocs & INP_BLOC_PGA) {
		if (on)
			wm8985_field_write(codec, WM8985_POWER2,(INPPGAENR|INPPGAENL),
				(INPPGAENRM|INPPGAENLM));
		else
			wm8985_field_write(codec, WM8985_POWER2,0,(INPPGAENRM|INPPGAENLM));
	}

	if (input_blocs & INP_BLOC_MIC) {
		if (on)
			wm8985_field_write(codec, WM8985_POWER1,MIC_BIASEN,MIC_BIASEN_M);
		else
			wm8985_field_write(codec, WM8985_POWER1,0,MIC_BIASEN_M);
	}
#endif /* DYNAMIC_POWER */
}
EXPORT_SYMBOL_GPL(wm8985_power_up_inputs);

void wm8985_power_up_outn_mixers(struct snd_soc_codec *codec, int mixers, int on)
{
#if !defined (DYNAMIC_POWER)
	if (mixers & MIXER_OUT3) {
		if (on)
			wm8985_field_write(codec, WM8985_POWER1,OUT3MIXEN,OUT3MIXEN);
		else
			wm8985_field_write(codec, WM8985_POWER1,0,OUT3MIXEN);
	}

	if (mixers & MIXER_OUT4) {
		if (on)
			wm8985_field_write(codec, WM8985_POWER1,OUT4MIXEN,OUT4MIXEN);
		else
			wm8985_field_write(codec, WM8985_POWER1,0,OUT4MIXEN);
	}
#endif /* DYNAMIC_POWER */
}
EXPORT_SYMBOL_GPL(wm8985_power_up_outn_mixers);

void wm8985_select_outputs(struct snd_soc_codec *codec, int outputs, int on)
{
#if !defined (DYNAMIC_POWER)
	if (outputs & OUTPUT_SPK) {
		if (on)
			wm8985_field_write(codec, WM8985_POWER3,(ROUT2EN|LOUT2EN),(ROUT2ENM|LOUT2ENM));
		else
			wm8985_field_write(codec, WM8985_POWER3,0,(ROUT2ENM|LOUT2ENM));
	}

	if (outputs & OUTPUT_HP) {
		if (on)
			wm8985_field_write(codec, WM8985_POWER2,(ROUT1EN|LOUT1EN),(ROUT1ENM|LOUT1ENM));
		else
			wm8985_field_write(codec, WM8985_POWER2,0,(ROUT1ENM|LOUT1ENM));
	}

	if (outputs & OUTPUT_OUT3) {
		if (on)
			wm8985_field_write(codec, WM8985_POWER3,OUT3EN,OUT3ENM);
		else
			wm8985_field_write(codec, WM8985_POWER3,0,OUT3ENM);
	}

	if (outputs & OUTPUT_OUT4) {
		if (on)
			wm8985_field_write(codec, WM8985_POWER3,OUT4EN,OUT4ENM);
		else
			wm8985_field_write(codec, WM8985_POWER3,0,OUT4ENM);
	}
#endif /* DYNAMIC_POWER */
}
EXPORT_SYMBOL_GPL(wm8985_select_outputs);

void wm8985_connect_pga(struct snd_soc_codec *codec, int on)
{
	if (on) {
		wm8985_field_write(codec, WM8985_INPPGAL,0,INPPGAMUTELM);
		wm8985_field_write(codec, WM8985_INPPGAR,0,INPPGAMUTERM);
	} else {
		wm8985_field_write(codec, WM8985_INPPGAL,INPPGAMUTEL,INPPGAMUTELM);
		wm8985_field_write(codec, WM8985_INPPGAR,INPPGAMUTER,INPPGAMUTERM);
	}
}
EXPORT_SYMBOL_GPL(wm8985_connect_pga);

void wm8985_select_pga_inputs(struct snd_soc_codec *codec, int inputs, int on)
{
	if (inputs & INPUT_LIP) {
		if (on)
			wm8985_field_write(codec, WM8985_INPUT,LIP_2INPPGA,LIP_2INPPGA);
		else
			wm8985_field_write(codec, WM8985_INPUT,0,LIP_2INPPGA);
	}

	if (inputs & INPUT_RIP) {
		if (on)
			wm8985_field_write(codec, WM8985_INPUT,RIP_2INPPGA,RIP_2INPPGA);
		else
			wm8985_field_write(codec, WM8985_INPUT,0,RIP_2INPPGA);
	}

	if (inputs & INPUT_LIN) {
		if (on)
			wm8985_field_write(codec, WM8985_INPUT,LIN_2INPPGA,LIN_2INPPGA);
		else
			wm8985_field_write(codec, WM8985_INPUT,0,LIN_2INPPGA);
	}

	if (inputs & INPUT_RIN) {
		if (on)
			wm8985_field_write(codec, WM8985_INPUT,RIN_2INPPGA,RIN_2INPPGA);
		else
			wm8985_field_write(codec, WM8985_INPUT,0,RIN_2INPPGA);
	}

	if (inputs & INPUT_LINE) {
		if (on)
			wm8985_field_write(codec, WM8985_INPUT,
				(L2_2INPPGA|R2_2INPPGA),(L2_2INPPGA|R2_2INPPGA));
		else
			wm8985_field_write(codec, WM8985_INPUT,0,(L2_2INPPGA|R2_2INPPGA));
	}
}
EXPORT_SYMBOL_GPL(wm8985_select_pga_inputs);

static int wm8985_put_volsw_2r_vu(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int ret;
	unsigned int reg = mc->reg;
	u16 val;

	ret = snd_soc_put_volsw_2r(kcontrol, ucontrol);
	if (ret < 0)
		return ret;

	/* now hit the volume update bits (always bit 8) */
	val = wm8985_read_reg_cache(codec, reg);
	wm8985_write(codec, reg, val | OUTVU);
	return 1;
}

static int wm8985_get_volsw_2r_2s(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	unsigned int mask = (1<<fls(max))-1;
	unsigned int invert = mc->invert;

	ucontrol->value.integer.value[0] =
		(snd_soc_read(codec, reg) >> shift) & mask;
	ucontrol->value.integer.value[1] =
		(snd_soc_read(codec, reg2) >> rshift) & mask;
	if (invert) {
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];
		ucontrol->value.integer.value[1] =
			max - ucontrol->value.integer.value[1];
	}

	return 0;
}

static int wm8985_put_volsw_2r_2s(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	int err;
	unsigned short val, val2, val_mask, val_mask2;

	val_mask = mask << shift;
	val_mask2 = mask << rshift;
	val = (ucontrol->value.integer.value[0] & mask);
	val2 = (ucontrol->value.integer.value[1] & mask);

	if (invert) {
		val = max - val;
		val2 = max - val2;
	}

	val = val << shift;
	val2 = val2 << rshift;

	err = snd_soc_update_bits(codec, reg, val_mask, val);
	if (err < 0)
		return err;

	err = snd_soc_update_bits(codec, reg2, val_mask2, val2);
	return err;
}

#define wm8985_reset(c)	wm8985_write(c, WM8985_RESET, 0)

/* Allows to set the update bit when a vol is updated */
#define SOC_WM8985_DOUBLE_R(xname, reg_left, reg_right, xshift, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw_2r, \
	.get = snd_soc_get_volsw_2r, .put = wm8985_put_volsw_2r_vu, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, .rreg = reg_right, .shift = xshift, \
		.max = xmax, .invert = xinvert} }

/* Allows to use 2 regs and 2 shifts */
#define SOC_WM8985_DOUBLE_R_S(xname, reg_left, reg_right, shift_left, \
	shift_right, xmax, xinvert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw_2r, \
	.get = wm8985_get_volsw_2r_2s, .put = wm8985_put_volsw_2r_2s, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, .rreg = reg_right, .shift = shift_left, \
		.rshift = shift_right, .max = xmax, .invert = xinvert} }

static const char *wm8985_companding[] = {"Off", "NC", "u-law", "A-law" };
static const char *wm8985_deemp[] = {"None", "32kHz", "44.1kHz", "48kHz" };
static const char *wm8985_lowpower[] = {"On", "Off" };
static const char *wm8985_eqmode[] = {"Capture", "Playback" };
static const char *wm8985_bw[] = {"Narrow", "Wide" };
static const char *wm8985_eq1[] = {"80Hz", "105Hz", "135Hz", "175Hz" };
static const char *wm8985_eq2[] = {"230Hz", "300Hz", "385Hz", "500Hz" };
static const char *wm8985_eq3[] = {"650Hz", "850Hz", "1.1kHz", "1.4kHz" };
static const char *wm8985_eq4[] = {"1.8kHz", "2.4kHz", "3.2kHz", "4.1kHz" };
static const char *wm8985_eq5[] = {"5.3kHz", "6.9kHz", "9kHz", "11.7kHz" };
static const char *wm8985_alc[] =
    {"Off", "Right", "Left", "Both" };
static const char *wm8985_alcmode[] =
    {"ALC", "Limiter" };
static const char *wm8985_out42boost[] = {"Left", "Right" };
static const char *wm8985_limiter[] = {"Off", "On" };

static const struct soc_enum wm8985_enum[] = {
	SOC_ENUM_SINGLE(WM8985_COMP, 1, 4, wm8985_companding), /* adc */
	SOC_ENUM_SINGLE(WM8985_COMP, 3, 4, wm8985_companding), /* dac */
	SOC_ENUM_SINGLE(WM8985_DAC,  4, 4, wm8985_deemp),
	SOC_ENUM_SINGLE(WM8985_EQ1,  8, 2, wm8985_eqmode),

	SOC_ENUM_SINGLE(WM8985_EQ1,  5, 4, wm8985_eq1),
	SOC_ENUM_SINGLE(WM8985_EQ2,  8, 2, wm8985_bw),
	SOC_ENUM_SINGLE(WM8985_EQ2,  5, 4, wm8985_eq2),
	SOC_ENUM_SINGLE(WM8985_EQ3,  8, 2, wm8985_bw),

	SOC_ENUM_SINGLE(WM8985_EQ3,  5, 4, wm8985_eq3),
	SOC_ENUM_SINGLE(WM8985_EQ4,  8, 2, wm8985_bw),
	SOC_ENUM_SINGLE(WM8985_EQ4,  5, 4, wm8985_eq4),

	SOC_ENUM_SINGLE(WM8985_EQ5,  5, 4, wm8985_eq5),
	SOC_ENUM_SINGLE(WM8985_ALC1,  0, 4, wm8985_alc),
	SOC_ENUM_SINGLE(WM8985_ALC3,  8, 2, wm8985_alcmode),
	SOC_ENUM_SINGLE(WM8985_OUT4ADC, 5, 2, wm8985_out42boost),
	SOC_ENUM_SINGLE(WM8985_DACLIM1, 8, 2, wm8985_limiter),
	SOC_ENUM_SINGLE(WM8985_ADD, 8, 2, wm8985_lowpower),
};

/* Syntax : <Source> [Direction : Playback | Capture]
	<Function : Switch | Volume | Route> */
static const struct snd_kcontrol_new wm8985_snd_controls[] = {
SOC_SINGLE("Digital Loopback Switch", WM8985_COMP, 0, 1, 0),

SOC_ENUM("ADC Companding Capture Volume", wm8985_enum[0]),
SOC_ENUM("DAC Companding Playback Volume", wm8985_enum[1]),

/* Not used by us... */
//SOC_SINGLE("Jack Detection Enable", WM8985_JACK1, 6, 1, 0),

SOC_DOUBLE("DAC Inversion Switch", WM8985_DAC, 0, 1, 1, 0),
SOC_WM8985_DOUBLE_R("DAC Playback Volume", WM8985_DACVOLL, WM8985_DACVOLR, 0, 255, 0),

SOC_SINGLE("High Pass Filter Capture Switch", WM8985_ADC, 8, 1, 0),
SOC_SINGLE("High Pass Cut Off Capture Volume", WM8985_ADC, 4, 7, 0),

SOC_DOUBLE("ADC Inversion Switch", WM8985_ADC, 0, 1, 1, 0),
SOC_WM8985_DOUBLE_R("ADC Capture Volume", WM8985_ADCVOLL, WM8985_ADCVOLR, 0, 255, 0),

SOC_SINGLE("3D Enhancement Playback Volume", WM8985_3D, 0, 15, 0),

SOC_ENUM("Low Power Mode", wm8985_enum[16]),

SOC_ENUM("Equaliser Function", wm8985_enum[3]),
SOC_ENUM("EQ1 Cut Off", wm8985_enum[4]),
SOC_SINGLE("EQ1 Playback Volume", WM8985_EQ1, 0, 24, 1),

SOC_ENUM("EQ2 Bandwith", wm8985_enum[5]),
SOC_ENUM("EQ2 Center", wm8985_enum[6]),
SOC_SINGLE("EQ2 Playback Volume", WM8985_EQ2, 0, 24, 1),

SOC_ENUM("EQ3 Bandwith", wm8985_enum[7]),
SOC_ENUM("EQ3 Center", wm8985_enum[8]),
SOC_SINGLE("EQ3 Playback Volume", WM8985_EQ3, 0, 24, 1),

SOC_ENUM("EQ4 Bandwith", wm8985_enum[9]),
SOC_ENUM("EQ4 Center", wm8985_enum[10]),
SOC_SINGLE("EQ4 Playback Volume", WM8985_EQ4, 0, 24, 1),

SOC_ENUM("EQ5 Cut Off", wm8985_enum[11]),
SOC_SINGLE("EQ5 Playback Volume", WM8985_EQ5, 0, 24, 1),

SOC_ENUM("DAC Limiter Playback Switch", wm8985_enum[15]),
SOC_SINGLE("DAC Limiter Decay Playback Volume", WM8985_DACLIM1, 4, 15, 0),
SOC_SINGLE("DAC Limiter Attack Playback Volume", WM8985_DACLIM1, 0, 15, 0),
SOC_SINGLE("DAC Limiter Threshold Playback Volume", WM8985_DACLIM2, 4, 7, 0),
SOC_SINGLE("DAC Limiter Boost Playback Volume", WM8985_DACLIM2, 0, 15, 0),

SOC_ENUM("ALC Function Capture Volume", wm8985_enum[12]),
SOC_SINGLE("ALC Max Gain Capture Volume", WM8985_ALC1, 3, 7, 0),
SOC_SINGLE("ALC Min Gain Capture Volume", WM8985_ALC1, 0, 7, 0),
SOC_SINGLE("ALC ZC Capture Switch", WM8985_ALC2, 8, 1, 0),
SOC_SINGLE("ALC Hold Capture Volume", WM8985_ALC2, 4, 7, 0),
SOC_SINGLE("ALC Target Capture Volume", WM8985_ALC2, 0, 15, 0),
SOC_ENUM("ALC Mode Capture Switch", wm8985_enum[13]),
SOC_SINGLE("ALC Decay Capture Volume", WM8985_ALC3, 4, 15, 0),
SOC_SINGLE("ALC Attack Capture Volume", WM8985_ALC3, 0, 15, 0),
SOC_SINGLE("ALC Noise Gate Capture Switch", WM8985_NGATE, 3, 1, 0),
SOC_SINGLE("ALC Noise Gate Threshold Capture Volume", WM8985_NGATE, 0, 7, 0),

SOC_DOUBLE_R("Boost Stage Playback Volume", WM8985_MIXL, WM8985_MIXR,
	2, 3, 0),
SOC_DOUBLE_R("Aux Playback Volume", WM8985_MIXL, WM8985_MIXR,
	6, 3, 0),

SOC_DOUBLE_R("Headphone ZC Playback Switch", WM8985_HPVOLL, WM8985_HPVOLR,
	7, 1, 0),
SOC_DOUBLE_R("Headphone Playback Switch", WM8985_HPVOLL, WM8985_HPVOLR,
	6, 1, 1),
SOC_WM8985_DOUBLE_R("Headphone Playback Volume", WM8985_HPVOLL, WM8985_HPVOLR,
	0, 57, 0),

SOC_DOUBLE_R("Speaker ZC Playback Switch", WM8985_SPKVOLL, WM8985_SPKVOLR,
	7, 1, 0),
SOC_DOUBLE_R("Speaker Playback Switch", WM8985_SPKVOLL, WM8985_SPKVOLR,
	6, 1, 1),
SOC_WM8985_DOUBLE_R("Speaker Playback Volume", WM8985_SPKVOLL, WM8985_SPKVOLR,
	0,63, 0),

SOC_SINGLE("Out4 Playback Switch", WM8985_OUT4MIX, 6, 1, 1),

SOC_SINGLE("Out3 Playback Switch", WM8985_OUT3MIX, 6, 1, 1),

SOC_DOUBLE_R("PGA Capture Boost(+20dB) Capture Switch", WM8985_ADCBOOSTL, WM8985_ADCBOOSTR,
	8, 1, 0),

SOC_DOUBLE_R("PGA Capture ZC Capture Switch", WM8985_INPPGAL, WM8985_INPPGAR,
	7, 1, 0),
SOC_WM8985_DOUBLE_R("PGA Capture Capture Volume", WM8985_INPPGAL, WM8985_INPPGAR,
	0, 63, 0),

#if !defined (DYNAMIC_POWER) || defined (COMPATIBILITY)
/* Some kcontrols from DAPM stuff are needed if DAPM is disabled */
SOC_DOUBLE_R("Aux Capture Capture Volume", WM8985_ADCBOOSTL, WM8985_ADCBOOSTR,
	0, 7, 0),
SOC_DOUBLE_R("Line Capture Capture Volume", WM8985_ADCBOOSTL, WM8985_ADCBOOSTR,
	4, 7, 0),
SOC_SINGLE("Out4 Capture Capture Volume", WM8985_OUT4ADC, 6, 7, 0),
SOC_ENUM("Out4 Capture Destination Capture Switch", wm8985_enum[14]),

SOC_SINGLE("Right Boost Stage To Out4 Playback Switch", WM8985_OUT4MIX, 2, 1, 0),
SOC_SINGLE("Left Boost Stage To Out3 Playback Switch", WM8985_OUT3MIX, 2, 1, 0),
SOC_SINGLE("Out3 To Out4 Playback Switch", WM8985_OUT4MIX, 7, 1, 0),
SOC_SINGLE("Out4 To Out3 Playback Switch", WM8985_OUT3MIX, 7, 1, 0),
SOC_SINGLE("Right DAC To Out4 Playback Switch", WM8985_OUT4MIX, 0, 1, 0),

SOC_DOUBLE_R("Aux Playback Switch", WM8985_MIXL, WM8985_MIXR, 5, 1, 0),
SOC_DOUBLE_R("Boost Stage Playback Switch", WM8985_MIXL, WM8985_MIXR, 1, 1, 0),
SOC_DOUBLE_R("PGA Capture Capture Switch", WM8985_INPPGAL, WM8985_INPPGAR, 6, 1, 1),

#endif /* DYNAMIC_POWER */

};

/* add non dapm controls */
static int wm8985_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8985_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm8985_snd_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

/* Left Playback Mixer */
static const struct snd_kcontrol_new wm8985_left_mixer_controls[] = {
SOC_DAPM_SINGLE("Left PCM Playback Switch", WM8985_MIXL, 0, 1, 0),
SOC_DAPM_SINGLE("Right PCM Playback Switch", WM8985_OUTPUT, 5, 1, 0),
SOC_DAPM_SINGLE("Left Input Playback Switch", WM8985_MIXL, 1, 1, 0),
SOC_DAPM_SINGLE("Right Input Playback Switch", WM8985_BEEP, 7, 1, 0),
SOC_DAPM_SINGLE("Left Aux Playback Switch", WM8985_MIXL, 5, 1, 0),
};

/* Right Playback Mixer */
static const struct snd_kcontrol_new wm8985_right_mixer_controls[] = {
SOC_DAPM_SINGLE("Left PCM Playback Switch", WM8985_OUTPUT, 6, 1, 0),
SOC_DAPM_SINGLE("Right PCM Playback Switch", WM8985_MIXR, 0, 1, 0),
SOC_DAPM_SINGLE("Left Input Playback Switch", WM8985_MIXR, 1, 1, 0),
SOC_DAPM_SINGLE("Right Input Playback Switch", WM8985_BEEP, 8, 1, 0),
SOC_DAPM_SINGLE("Right Aux Playback Switch", WM8985_MIXR, 5, 1, 0),
};

/* Out 3 Mixer */
static const struct snd_kcontrol_new wm8985_out3_mixer_controls[] = {
SOC_DAPM_SINGLE("Left PCM Playback Switch", WM8985_OUT3MIX, 0, 1, 0),
SOC_DAPM_SINGLE("Left Mixer Playback Switch", WM8985_OUT3MIX, 1, 1, 0),
SOC_DAPM_SINGLE("Left Input Playback Switch", WM8985_OUT3MIX, 2, 1, 0),
SOC_DAPM_SINGLE("Out4 Mixer Playback Switch", WM8985_OUT3MIX, 3, 1, 0),
};

/* Out 4 Mixer */
static const struct snd_kcontrol_new wm8985_out4_mixer_controls[] = {
SOC_DAPM_SINGLE("Out3 Mixer Playback Switch", WM8985_OUT4MIX, 7, 1, 0),
SOC_DAPM_SINGLE("Right PCM Playback Switch", WM8985_OUT4MIX, 0, 1, 0),
SOC_DAPM_SINGLE("Right Mixer Playback Switch", WM8985_OUT4MIX, 1, 1, 0),
SOC_DAPM_SINGLE("Right Input Playback Switch", WM8985_OUT4MIX, 2, 1, 0),
SOC_DAPM_SINGLE("Left PCM Playback Switch", WM8985_OUT4MIX, 3, 1, 0),
SOC_DAPM_SINGLE("Left Mixer Playback Switch", WM8985_OUT4MIX, 4, 1, 0),
};

/* Left Boost Mixer */
static const struct snd_kcontrol_new wm8985_left_boost_mixer_controls[] = {
SOC_DAPM_SINGLE("Out4 Mixer Capture Switch", WM8985_OUT4ADC, 5, 1, 0),
SOC_DAPM_SINGLE("Left Aux Capture Volume", WM8985_ADCBOOSTL, 0, 7, 0),
SOC_DAPM_SINGLE("Left PGA Capture Switch", WM8985_INPPGAL, 6, 1, 1),
SOC_DAPM_SINGLE("Left Line Capture Volume", WM8985_ADCBOOSTL, 4, 7, 0),
};

/* Right Boost Mixer */
static const struct snd_kcontrol_new wm8985_right_boost_mixer_controls[] = {
SOC_DAPM_SINGLE("Out4 Mixer Capture Switch", WM8985_OUT4ADC, 5, 1, 1),
SOC_DAPM_SINGLE("Right Aux Capture Volume", WM8985_ADCBOOSTR, 0, 7, 0),
SOC_DAPM_SINGLE("Right PGA Capture Switch", WM8985_INPPGAR, 6, 1, 1),
SOC_DAPM_SINGLE("Right Line Capture Volume", WM8985_ADCBOOSTR, 4, 7, 0),
};

/* Out 4 to Boost Volume */
static const struct snd_kcontrol_new wm8985_out42boost_volume_controls[] = {
SOC_DAPM_SINGLE("Right Boost Mixer Out4 Mixer Capture Volume", WM8985_OUT4ADC, 6, 7, 0),
};

/* Left Input PGA */
static const struct snd_kcontrol_new wm8985_left_input_pga_controls[] = {
SOC_DAPM_SINGLE("Left Input N Capture Switch", WM8985_INPUT, 1, 1, 0),
SOC_DAPM_SINGLE("Left Input P Capture Switch", WM8985_INPUT, 0, 1, 0),
SOC_DAPM_SINGLE("Left Line Capture Switch", WM8985_INPUT, 2, 1, 0),
};
/* Right Input PGA */
static const struct snd_kcontrol_new wm8985_right_input_pga_controls[] = {
SOC_DAPM_SINGLE("Right Input N Capture Switch", WM8985_INPUT, 5, 1, 0),
SOC_DAPM_SINGLE("Right Input P Capture Switch", WM8985_INPUT, 4, 1, 0),
SOC_DAPM_SINGLE("Right Line Capture Switch", WM8985_INPUT, 6, 1, 0),
};

static const struct snd_soc_dapm_widget wm8985_dapm_widgets[] = {
#ifdef DYNAMIC_POWER
SND_SOC_DAPM_MIXER("Left Output Mixer", WM8985_POWER3, 2, 0,
	wm8985_left_mixer_controls,
	ARRAY_SIZE(wm8985_left_mixer_controls)),
SND_SOC_DAPM_MIXER("Right Output Mixer", WM8985_POWER3, 3, 0,
	wm8985_right_mixer_controls,
	ARRAY_SIZE(wm8985_right_mixer_controls)),

SND_SOC_DAPM_DAC("Left DAC", "Playback", WM8985_POWER3, 0, 0),
SND_SOC_DAPM_DAC("Right DAC", "Playback", WM8985_POWER3, 1, 0),
SND_SOC_DAPM_ADC("Left ADC", "Capture", WM8985_POWER2, 0, 0),
SND_SOC_DAPM_ADC("Right ADC", "Capture", WM8985_POWER2, 1, 0),

SND_SOC_DAPM_PGA("Left Out 1", WM8985_POWER2, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Out 1", WM8985_POWER2, 8, 0, NULL, 0),
SND_SOC_DAPM_PGA("Left Out 2", WM8985_POWER3, 5, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Out 2", WM8985_POWER3, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("Out 3", WM8985_POWER3, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("Out 4", WM8985_POWER3, 8, 0, NULL, 0),

SND_SOC_DAPM_MIXER("Left In PGA", WM8985_POWER2, 2, 0,
	wm8985_left_input_pga_controls,
	ARRAY_SIZE(wm8985_left_input_pga_controls)),
SND_SOC_DAPM_MIXER("Right In PGA", WM8985_POWER2, 3, 0,
	wm8985_right_input_pga_controls,
	ARRAY_SIZE(wm8985_right_input_pga_controls)),

SND_SOC_DAPM_MIXER("Left Boost Mixer", WM8985_POWER2, 4, 0,
	wm8985_left_boost_mixer_controls,
	ARRAY_SIZE(wm8985_left_boost_mixer_controls)),
SND_SOC_DAPM_MIXER("Right Boost Mixer", WM8985_POWER2, 5, 0,
	wm8985_right_boost_mixer_controls,
	ARRAY_SIZE(wm8985_right_boost_mixer_controls)),

SND_SOC_DAPM_MIXER("Out 3 Mixer", WM8985_POWER1, 6, 0,
	wm8985_out3_mixer_controls,
	ARRAY_SIZE(wm8985_out3_mixer_controls)),
SND_SOC_DAPM_MIXER("Out 4 Mixer", WM8985_POWER1, 7, 0,
	wm8985_out4_mixer_controls,
	ARRAY_SIZE(wm8985_out4_mixer_controls)),

SND_SOC_DAPM_MIXER_NAMED_CTL("Out 4 To Boost Volume", SND_SOC_NOPM, 0, 0,
	wm8985_out42boost_volume_controls,
	ARRAY_SIZE(wm8985_out42boost_volume_controls)),

SND_SOC_DAPM_MICBIAS("Mic Bias", WM8985_POWER1, 4, 0),

SND_SOC_DAPM_INPUT("LINN"),
SND_SOC_DAPM_INPUT("RINN"),
SND_SOC_DAPM_INPUT("LINP"),
SND_SOC_DAPM_INPUT("RINP"),
SND_SOC_DAPM_INPUT("LAUX"),
SND_SOC_DAPM_INPUT("RAUX"),
SND_SOC_DAPM_INPUT("LLINE"),
SND_SOC_DAPM_INPUT("RLINE"),
SND_SOC_DAPM_OUTPUT("LOUT1"),
SND_SOC_DAPM_OUTPUT("ROUT1"),
SND_SOC_DAPM_OUTPUT("LOUT2"),
SND_SOC_DAPM_OUTPUT("ROUT2"),
SND_SOC_DAPM_OUTPUT("OUT3"),
SND_SOC_DAPM_OUTPUT("OUT4"),
#endif /* DYNAMIC_POWER */
};

static const struct snd_soc_dapm_route audio_map[] = {
#ifdef DYNAMIC_POWER
	/* Left output mixer */
	{"Left Output Mixer", "Left PCM Playback Switch", "Left DAC"},
	{"Left Output Mixer", "Right PCM Playback Switch", "Right DAC"},
	{"Left Output Mixer", "Left Input Playback Switch", "Left Boost Mixer"},
	{"Left Output Mixer", "Right Input Playback Switch", "Right Boost Mixer"},
	{"Left Output Mixer", "Left Aux Playback Switch", "LAUX"},

	/* Right output mixer */
	{"Right Output Mixer", "Left PCM Playback Switch", "Left DAC"},
	{"Right Output Mixer", "Right PCM Playback Switch", "Right DAC"},
	{"Right Output Mixer", "Left Input Playback Switch", "Left Boost Mixer"},
	{"Right Output Mixer", "Right Input Playback Switch", "Right Boost Mixer"},
	{"Right Output Mixer", "Right Aux Playback Switch", "RAUX"},

	/* Out3 output mixer */
	{"Out 3 Mixer", "Left PCM Playback Switch", "Left DAC"},
	{"Out 3 Mixer", "Left Mixer Playback Switch", "Left Output Mixer"},
	{"Out 3 Mixer", "Left Input Playback Switch", "Left Boost Mixer"},
	{"Out 3 Mixer", "Out4 Mixer Playback Switch", "Out 4 Mixer"},

	/* Out4 output mixer */
	{"Out 4 Mixer", "Out3 Mixer Playback Switch", "Out 3 Mixer"},
	{"Out 4 Mixer", "Right PCM Playback Switch", "Right DAC"},
	{"Out 4 Mixer", "Right Mixer Playback Switch", "Right Output Mixer"},
	{"Out 4 Mixer", "Right Input Playback Switch", "Right Boost Mixer"},
	{"Out 4 Mixer", "Left PCM Playback Switch", "Left DAC"},
	{"Out 4 Mixer", "Left Mixer Playback Switch", "Left Output Mixer"},

	/* Outputs */
	{"Left Out 1", NULL, "Left Output Mixer"},
	{"Right Out 1", NULL, "Right Output Mixer"},
	{"LOUT1", NULL, "Left Out 1"},
	{"ROUT1", NULL, "Right Out 1"},

	{"Left Out 2", NULL, "Left Output Mixer"},
	{"Right Out 2", NULL, "Right Output Mixer"},
	{"LOUT2", NULL, "Left Out 2"},
	{"ROUT2", NULL, "Right Out 2"},

	{"Out 3", NULL, "Out 3 Mixer"},
	{"OUT3", NULL, "Out 3"},
	{"Out 4", NULL, "Out 4 Mixer"},
	{"OUT4", NULL, "Out 4"},

	/* Left Boost Mixer */
	{"Left Boost Mixer", "Out4 Mixer Capture Switch", "Out 4 To Boost Volume"},
	{"Left Boost Mixer", "Left Aux Capture Volume", "LAUX"},
	{"Left Boost Mixer", "Left PGA Capture Switch", "Left In PGA"},
	{"Left Boost Mixer", "Left Line Capture Volume", "LLINE"},
	{"Left Boost Mixer", NULL, "Left ADC"},

	/* Right Boost Mixer */
	{"Right Boost Mixer", "Out4 Mixer Capture Switch", "Out 4 To Boost Volume"},
	{"Right Boost Mixer", "Right Aux Capture Volume", "RAUX"},
	{"Right Boost Mixer", "Right PGA Capture Switch", "Right In PGA"},
	{"Right Boost Mixer", "Right Line Capture Volume", "RLINE"},
	{"Right Boost Mixer", NULL, "Right ADC"},

	/* Out 4 to Boost Volume */
	{"Out 4 To Boost Volume", "Right Boost Mixer Out4 Mixer Capture Volume", "Out 4 Mixer"},

	/* Left Input PGA */
	{"Left In PGA", "Left Input N Capture Switch", "LINN"},
	{"Left In PGA", "Left Input P Capture Switch", "LINP"},
	{"Left In PGA", "Left Line Capture Switch", "LLINE"},

	/* Right Input PGA */
	{"Right In PGA", "Right Input N Capture Switch", "RINN"},
	{"Right In PGA", "Right Input P Capture Switch", "RINP"},
	{"Right In PGA", "Right Line Capture Switch", "RLINE"},
#endif /* DYNAMIC_POWER */

};

static int wm8985_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, wm8985_dapm_widgets,
				  ARRAY_SIZE(wm8985_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

struct pll_ {
	unsigned int in_hz, out_hz;
	unsigned int pre:4; /* prescale - 1 */
	unsigned int n:4;
	unsigned int k;
};

static struct pll_ pll[] = {
	{12000000,  8192000, 0, 10, 0xf433e2},
	{12000000, 11289600, 0,  7, 0x86c226},
	{12000000, 12288000, 0,  8, 0x3126e8},
	{13000000, 11289600, 0,  6, 0xf28bd4},
	{13000000, 12288000, 0,  7, 0x8fd525},
	{12288000, 11289600, 0,  7, 0x59999a},
	{11289600, 12288000, 0,  8, 0x80dee9},
	{19200000,  8192000, 1,  6, 0xd3a06d},
	{19200000, 11289600, 1,  9, 0x6872af},
	{19200000, 12288000, 1, 10, 0x3d70a3},
	/* TODO: liam - add more entries */
};

static int wm8985_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	int i;
	u16 reg;

	if(freq_in == 0 || freq_out == 0) {
		reg = wm8985_read_reg_cache(codec, WM8985_POWER1);
		wm8985_write(codec, WM8985_POWER1, reg & (0x1df));
		return 0;
	}

	for(i = 0; i < ARRAY_SIZE(pll); i++) {
		if (freq_in == pll[i].in_hz && freq_out == pll[i].out_hz) {
			//printk("wm8985_set_dai_pll : in = %d, out = %d, pre = %d, n = %d, k = %X\n",
			//	pll[i].in_hz, pll[i].out_hz, pll[i].pre, pll[i].n, pll[i].k);
			wm8985_write(codec, WM8985_PLLN, (pll[i].pre << 4) | pll[i].n);
			wm8985_write(codec, WM8985_PLLK1, pll[i].k >> 18);
			wm8985_write(codec, WM8985_PLLK2, (pll[i].k >> 9) & 0x1ff);
			wm8985_write(codec, WM8985_PLLK3, pll[i].k & 0x1ff);
			reg = wm8985_read_reg_cache(codec, WM8985_POWER1);
			wm8985_write(codec, WM8985_POWER1, reg | 0x20);
			return 0;
		}
	}
	return -EINVAL;
}

/* Set the the clocks, could be needed (not for now) */
static int wm8985_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct wm8985_priv *wm8985 = codec->private_data;

	//printk("wm8985_set_dai_sysclk : inclk at %d\n", freq);
	switch (freq) {
	case 12000000:
		if (clk_id == WM8985_ID_MCLK) {
			wm8985->inclk = freq;
			return 0;
		}
		break;
	case 19200000:
		if (clk_id == WM8985_ID_MCLK) {
			wm8985->inclk = freq;
			return 0;
		}
		break;
	default :
		printk(KERN_ERR "wm8985_set_dai_sysclk : sysclock value not supported\n");
	}
	return -EINVAL;
}

static int wm8985_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = wm8985_read_reg_cache(codec, WM8985_IFACE) & 0x3;
	u16 clk = wm8985_read_reg_cache(codec, WM8985_CLOCK) & 0xfffe;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		clk |= 0x0001;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0010;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0008;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x00018;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:	// DSP mode A + BCKL Normal
		break;
	case SND_SOC_DAIFMT_IB_IF:	// DSP mode B + BCKL Normal
		iface |= 0x0180;
		break;
	case SND_SOC_DAIFMT_IB_NF:	// DSP mode A + BCKL Inverted
		iface |= 0x0100;
		break;
	case SND_SOC_DAIFMT_NB_IF:	// DSP mode B + BCKL Inverted
		iface |= 0x0080;
		break;
	default:
		return -EINVAL;
	}

	wm8985_write(codec, WM8985_IFACE, iface);
	wm8985_write(codec, WM8985_CLOCK, clk);
	return 0;
}

static int wm8985_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u16 iface = wm8985_read_reg_cache(codec, WM8985_IFACE) & 0xff9e;
	u16 adn = wm8985_read_reg_cache(codec, WM8985_ADD) & 0x1f1;
	u16 output = wm8985_read_reg_cache(codec, WM8985_OUTPUT) & 0x1bf;
	//printk("wm8985_hw_params : requested format = %d, rate = %d\n",
	//	params_format(params), params_rate(params));

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_IEC958_SUBFRAME_LE:	// SPDIF will not really use the WM8985
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0020;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0040;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x0060;
		break;
	}

	/* filter coefficient */
	switch (params_rate(params)) {
	case 8000:
		adn |= 0x5 << 1;
		break;
	case 11025:
		adn |= 0x4 << 1;
		break;
	case 12000:
		adn |= 0x4 << 1;
		break;
	case 16000:
		adn |= 0x3 << 1;
		break;
	case 22050:
		adn |= 0x2 << 1;
		break;
	case 24000:
		adn |= 0x2 << 1;
		break;
	case 32000:
		adn |= 0x1 << 1;
		break;
	case 44100:
		/* adn |= 0x0 << 1; */
		break;
	case 48000:
		/* adn |= 0x0 << 1; */
		break;
	}

	/* Mono/Sereo mode */
	switch (params_channels(params)) {
	case 1:
		iface |= 0x1;				// Mono
		output |= 0x1 << 6;			// Left DAC to Right OUT
		break;
	case 2:
		iface |= 0x3 << 1;			// Swap right and left channels (32 bits DMA needs this)
		/* output |= 0x0 << 6; */
		break;
	}	

	/* set iface */
	wm8985_write(codec, WM8985_OUTPUT, output);
	wm8985_write(codec, WM8985_IFACE, iface);
	wm8985_write(codec, WM8985_ADD, adn);
	return 0;
}

static int wm8985_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;
//printk("wm8985_set_dai_clkdiv\n");
	switch (div_id) {
	case WM8985_MCLKDIV:
		reg = wm8985_read_reg_cache(codec, WM8985_CLOCK) & 0x11f;
		wm8985_write(codec, WM8985_CLOCK, reg | div);
		break;
	case WM8985_BCLKDIV:
		reg = wm8985_read_reg_cache(codec, WM8985_CLOCK) & 0x1e1;
		wm8985_write(codec, WM8985_CLOCK, reg | div);
		break;
	case WM8985_OPCLKDIV:
		reg = wm8985_read_reg_cache(codec, WM8985_GPIO) & 0x1cf;
		wm8985_write(codec, WM8985_GPIO, reg | div);
		break;
	case WM8985_DACOSR:
		reg = wm8985_read_reg_cache(codec, WM8985_DAC) & 0x1f7;
		wm8985_write(codec, WM8985_DAC, reg | div);
		break;
	case WM8985_ADCOSR:
		reg = wm8985_read_reg_cache(codec, WM8985_ADC) & 0x1f7;
		wm8985_write(codec, WM8985_ADC, reg | div);
		break;
	case WM8985_MCLKSEL:
		reg = wm8985_read_reg_cache(codec, WM8985_CLOCK) & 0x0ff;
		wm8985_write(codec, WM8985_CLOCK, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wm8985_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = wm8985_read_reg_cache(codec, WM8985_DAC) & 0xffbf;

	if(mute)
		wm8985_write(codec, WM8985_DAC, mute_reg | 0x40);
	else
		wm8985_write(codec, WM8985_DAC, mute_reg);

	return 0;
}

/* If DYNAMIC_POWER is defined, power on begins in SND_SOC_BIAS_PREPARE
	(when is stream is about to be played), then ASoC enables some needed
	mix, DAC, ADC, outputs and inputs. The end of the power on procedure is
	in SND_SOC_BIAS_ON. DYNAMIC_POWER causes pops, but reduce consumption.
	If DYNAMIC_POWER is not defined, power on is done in SND_SOC_BIAS_STANDBY
	(only once, when the codec is initialized), and mix, ADC, DAC... are enabled
	manually, so only one pop, but no DAPM */
static int wm8985_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	u16 reg;
	struct wm8985_priv *wm8985 = codec->private_data;

	switch (level) {
	case SND_SOC_BIAS_ON:
		//printk("wm8985_set_bias_level: SND_SOC_BIAS_ON\n");
#ifdef DYNAMIC_POWER
		/* Disable Vmid independent */
		wm8985_field_write(codec, WM8985_OUT4ADC,POBCTRL_DIS,POBCTRLM);
	
		/* wait 500 ms */
		msleep(500);
#endif /* DYNAMIC_POWER */

		/* Volumes have to be set again to be effective */
		/* LOUT1 and ROUT1 */
		wm8985_write(codec, WM8985_HPVOLL,(wm8985->vol_hp_l|OUTVU));
		wm8985_write(codec, WM8985_HPVOLR,(wm8985->vol_hp_r|OUTVU));

		/* LOUT2 and ROUT2 */
		wm8985_write(codec, WM8985_SPKVOLL,(wm8985->vol_spk_l|OUTVU));
		wm8985_write(codec, WM8985_SPKVOLR,(wm8985->vol_spk_r|OUTVU));

		break;
	case SND_SOC_BIAS_PREPARE:
		//printk("wm8985_set_bias_level: SND_SOC_BIAS_PREPARE\n");
		if (codec->bias_level == SND_SOC_BIAS_STANDBY) {
#ifdef DYNAMIC_POWER
			/* Power ON is done here (before the stream start) */
			/* If we switch from STANDBY to ON, we have to activate BIAS */
			/* Set low analog bias mode */
			wm8985_write(codec, WM8985_BIASCTRL,BIASCUT);

			/* Enable thermal shutdown */	
			wm8985_write(codec, WM8985_OUTPUT,(TSDEN|TSOPCTRL));

			/* Enable Internal Bias*/
			wm8985_write(codec, WM8985_POWER1,(BIASEN|BUFIOEN));
#endif /* DYNAMIC_POWER */

			/* Mute all outputs and save volumes*/	
			/* LOUT1 and ROUT1 to minimum*/
			wm8985->vol_hp_l = wm8985_read_reg_cache(codec, WM8985_HPVOLL) & 0x0ff;
			wm8985_write(codec, WM8985_HPVOLL,(LHP_MUTE|OUTVU));
			wm8985->vol_hp_r = wm8985_read_reg_cache(codec, WM8985_HPVOLR) & 0x0ff;
			wm8985_write(codec, WM8985_HPVOLR,(RHP_MUTE|OUTVU));
			/* LOUT2 and ROUT2 to minimum*/
			wm8985->vol_spk_l = wm8985_read_reg_cache(codec, WM8985_SPKVOLL) & 0x0ff;
			wm8985_write(codec, WM8985_SPKVOLL,(LHP_MUTE|OUTVU));
			wm8985->vol_spk_r = wm8985_read_reg_cache(codec, WM8985_SPKVOLR) & 0x0ff;
			wm8985_write(codec, WM8985_SPKVOLR,(RHP_MUTE|OUTVU));

#ifdef DYNAMIC_POWER
			/* Enable Vmid independent current bias */
			wm8985_write(codec, WM8985_OUT4ADC,POBCTRL);

			/* Set VROI to 30k */
			wm8985_field_write(codec, WM8985_OUTPUT,VROI,VROI);

			/* Enable VMID with required charge */
			wm8985_field_write(codec, WM8985_POWER1,VMIDSEL_300k,VMIDSELM);

			/* wait 30 ms */
			msleep(30);
#endif /* DYNAMIC_POWER */
		}
		break;
	case SND_SOC_BIAS_STANDBY:
		//printk("wm8985_set_bias_level: SND_SOC_BIAS_STANDBY\n");
#ifndef DYNAMIC_POWER
		/* Power ON is done here (before the stream start) */
		/* If we switch from OFF to STANDBY, we have to activate BIAS */
		if (codec->bias_level == SND_SOC_BIAS_OFF) {

			/* Set low analog bias mode */
			wm8985_write(codec, WM8985_BIASCTRL,BIASCUT);

			/* Enable thermal shutdown */	
			wm8985_write(codec, WM8985_OUTPUT,(TSDEN|TSOPCTRL));

			/* Enable Internal Bias*/
			wm8985_write(codec, WM8985_POWER1,(BIASEN|BUFIOEN));

			/* Mute all outputs and save volumes*/	
			/* LOUT1 and ROUT1 to minimum*/
			wm8985_write(codec, WM8985_HPVOLL,(LHP_MUTE|OUTVU));
			wm8985_write(codec, WM8985_HPVOLR,(RHP_MUTE|OUTVU));
			/* LOUT2 and ROUT2 to minimum*/
			wm8985_write(codec, WM8985_SPKVOLL,(LHP_MUTE|OUTVU));
			wm8985_write(codec, WM8985_SPKVOLR,(RHP_MUTE|OUTVU));

			/* Enable Vmid indpendent current bias */
			wm8985_write(codec, WM8985_OUT4ADC,POBCTRL);

			/* Enable VMID with required charge */
			wm8985_field_write(codec, WM8985_POWER1,VMIDSEL_300k,VMIDSELM);

			/* wait 30 ms */
			msleep(30);

			/* enable ADC and DAC */
			wm8985_write(codec, WM8985_POWER2,(ADCENR|ADCENL));
			wm8985_write(codec, WM8985_POWER3,(DACENR|DACENL|RMIXEN|LMIXEN));

			/* Disable Vmid independent */
			wm8985_field_write(codec, WM8985_OUT4ADC,POBCTRL_DIS,POBCTRLM);
		
			/* wait 500 ms */
			msleep(500);
		}
		break;
#endif /* DYNAMIC_POWER */
	case SND_SOC_BIAS_OFF:
		msleep(2);

		//printk("wm8985_set_bias_level: SND_SOC_BIAS_OFF\n");
		/* Disable thermal shutdown */	
		reg = wm8985_read_reg_cache(codec, WM8985_OUTPUT) & 0x1f9;
		wm8985_write(codec, WM8985_OUTPUT, reg);

		/* Mute all outputs and save volumes*/	
		/* LOUT1 and ROUT1 to minimum*/
		wm8985->vol_hp_l = wm8985_read_reg_cache(codec, WM8985_HPVOLL) & 0x0ff;
		wm8985_write(codec, WM8985_HPVOLL,(LHP_MUTE|OUTVU));
		wm8985->vol_hp_r = wm8985_read_reg_cache(codec, WM8985_HPVOLR) & 0x0ff;
		wm8985_write(codec, WM8985_HPVOLR,(RHP_MUTE|OUTVU));
		/* LOUT2 and ROUT2 to minimum*/
		wm8985->vol_spk_l = wm8985_read_reg_cache(codec, WM8985_SPKVOLL) & 0x0ff;
		wm8985_write(codec, WM8985_SPKVOLL,(LHP_MUTE|OUTVU));
		wm8985->vol_spk_r = wm8985_read_reg_cache(codec, WM8985_SPKVOLR) & 0x0ff;
		wm8985_write(codec, WM8985_SPKVOLR,(RHP_MUTE|OUTVU));

		/* VMID OFF, BIAS OFF, BUFIO OFF */
		reg = wm8985_read_reg_cache(codec, WM8985_POWER1) & 0x1f0;
		wm8985_write(codec, WM8985_POWER1, reg);

		/* Wait for VMID... */
		msleep(500); 

		wm8985_write(codec, WM8985_POWER1, 0x0);
		wm8985_write(codec, WM8985_POWER2, 0x0);
		wm8985_write(codec, WM8985_POWER3, 0x0);
		break;
	}
	codec->bias_level = level;
	return 0;
}

#define WM8985_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)

#define WM8985_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE)

struct snd_soc_dai wm8985_dai = {
	.name = "WM8985 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8985_RATES,
		.formats = WM8985_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8985_RATES,
		.formats = WM8985_FORMATS,},
	.ops = {
		.hw_params = wm8985_hw_params,
		.digital_mute = wm8985_mute,
		.set_fmt = wm8985_set_dai_fmt,
		.set_clkdiv = wm8985_set_dai_clkdiv,
		.set_pll = wm8985_set_dai_pll,
		.set_sysclk = wm8985_set_dai_sysclk,
	},
};
EXPORT_SYMBOL_GPL(wm8985_dai);

static int wm8985_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	unsigned int val;

	static u16 cache_save[WM8985_CACHEREGNUM];
	
	/* save the cache before messing with power registers */
	memcpy(cache_save, codec->reg_cache, sizeof(wm8985_reg));
	
	/* turn off the codec */
	wm8985_set_bias_level(codec, SND_SOC_BIAS_OFF);
	
	/* restore the cache to have a sane state for resume */
	memcpy(codec->reg_cache, cache_save, sizeof(wm8985_reg));
	return 0;
}

static int wm8985_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	struct wm8985_priv *wm8985 = codec->private_data;
	int i;
	unsigned int val;
	u8 data[2];
	u16 *cache = codec->reg_cache;
		
	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8985_reg); i++) {

		/* skip RESET and POWER registers */
		if (i == WM8985_RESET)
			continue;
		if (i == WM8985_POWER1)
			continue;
		
		/* Skip non-existing registers 17, 31, 40 */
		if (i == 17 || i == 31 || i == 40)
			continue;
		/* Skip non-existing registers 58, 59, 60 */		
		if (i > 57 && i < 61) 
			continue;
		wm8985_write(codec, i, cache[i]);
	}

	val = wm8985_read_reg_cache(codec, WM8985_POWER1);
	wm8985_write(codec, WM8985_POWER1, val | 0x20);

	/* Volumes might need to be set again to be effective */
	/* LOUT1 and ROUT1 */
	wm8985_write(codec, WM8985_HPVOLL,(wm8985->vol_hp_l|OUTVU));
	wm8985_write(codec, WM8985_HPVOLR,(wm8985->vol_hp_r|OUTVU));
	/* LOUT2 and ROUT2 */
	wm8985_write(codec, WM8985_SPKVOLL,(wm8985->vol_spk_l|OUTVU));
	wm8985_write(codec, WM8985_SPKVOLR,(wm8985->vol_spk_r|OUTVU));

	return 0;
}

/* Some default register values */
static void wm8985_init_default_pre_power_levels(struct snd_soc_codec *codec)
{
	/* LOUT1 and ROUT1 to minimum*/
	wm8985_write(codec, WM8985_HPVOLL, (LHP_MUTE|OUTVU));
	wm8985_write(codec, WM8985_HPVOLR, (RHP_MUTE|OUTVU));

	/* LOUT2 and ROUT2 to minimum*/
	wm8985_write(codec, WM8985_SPKVOLL, (LHP_MUTE|OUTVU));
	wm8985_write(codec, WM8985_SPKVOLR, (RHP_MUTE|OUTVU));

	/* Input PGA to minimum and mute */
	wm8985_write(codec, WM8985_INPPGAL, (INPPGAMUTEL|INPPGAVU));
	wm8985_write(codec, WM8985_INPPGAR, (INPPGAMUTER|INPPGAVU));

	/* set PGA boost gain to 0 dB */
	/* and disconnect all from boost */
	wm8985_write(codec, WM8985_ADCBOOSTL, 0);
	wm8985_write(codec, WM8985_ADCBOOSTR, 0);

	/* mute and disable out3 mixer */
//  	wm8985_write(WM8985_OUT3_MIXER_CTRL,(OUT3MUTE|LDAC2OUT3_DIS));
  	wm8985_write(codec, WM8985_OUT3MIX,0);
	/* mute and disable out4 mixer */
//  	wm8985_write(WM8985_OUT4_MIXER_CTRL,(OUT4MUTE|RDAC2OUT4_DIS));
  	wm8985_write(codec, WM8985_OUT4MIX,0);

	/* Enable Vmid indpendent current bias */
	wm8985_write(codec, WM8985_OUT4ADC, POBCTRL);

	/* Enable required output */
	/* left mixer default is good: left dac to left out, all input not wired to output */
	wm8985_write(codec, WM8985_MIXL,DACL2LMIX);
	/* right mixer default is good: right dac to right out, all input not wired to output */
	wm8985_write(codec, WM8985_MIXR,DACR2RMIX);
	/* output ctrl default is good */

	/* setup path input: all inputs are disconnected from PGA */
	wm8985_write(codec, WM8985_INPUT,0);

	/* set eq to DAC */
	wm8985_field_write(codec, WM8985_EQ1,EQ3DMODE_DAC,EQ3DMODEM);
}

static void wm8985_init_default_post_power_levels(struct snd_soc_codec *codec)
{
	/* set DAC vol */
	wm8985_write(codec, WM8985_DACVOLL, (DACVU|DACVOL_00DB));
	wm8985_write(codec, WM8985_DACVOLR, (DACVU|DACVOL_00DB));

	/* set DAC best SNR and mute*/
	wm8985_write(codec, WM8985_DAC, (DACOSR128|DAC_SMUTE));

	/* set ADC best SNR and high pass filter*/
	wm8985_write(codec, WM8985_ADC, (HPFEN|ADCOSR));

	/* set slow clk for zero crossing */
	wm8985_write(codec, WM8985_ADD, SLOWCLKEN);

	/* Disable Vmid independent */
	wm8985_field_write(codec, WM8985_OUT4ADC, POBCTRL_DIS,POBCTRLM);

	/* wait 500 ms */
	msleep(500);
}

/*
 * initialise the WM8985 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int wm8985_init(struct snd_soc_device* socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;

	codec->name = "WM8985";
	codec->owner = THIS_MODULE;
	codec->read = wm8985_read_reg_cache;
	codec->write = wm8985_write;
	codec->set_bias_level = wm8985_set_bias_level;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->dai = &wm8985_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(wm8985_reg);
	codec->reg_cache = kmemdup(wm8985_reg, sizeof(wm8985_reg), GFP_KERNEL);

	if (codec->reg_cache == NULL)
		return -ENOMEM;

	wm8985_reset(codec);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if(ret < 0) {
		printk(KERN_ERR "wm8985: failed to create pcms\n");
		goto pcm_err;
	}

	/* set default levels (pre power) */
	wm8985_init_default_pre_power_levels(codec);

	/* power on device */
	wm8985_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* set default levels (post power) */
	wm8985_init_default_post_power_levels(codec);

	wm8985_add_controls(codec);
	wm8985_add_widgets(codec);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
      	printk(KERN_ERR "wm8985: failed to register card\n");
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

static struct snd_soc_device *wm8985_socdev;

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)

/*
 * WM8985 2 wire address is 0x1a
 */

static struct i2c_driver wm8985_i2c_driver;

/* If the i2c layer weren't so broken, we could pass this kind of data
   around */

static int wm8985_i2c_probe(struct i2c_client *client, 
		const struct i2c_device_id *id)
{
	struct snd_soc_device *socdev = wm8985_socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s functinality check failed\n", id->name);
		return -ENODEV;
	}

	i2c_set_clientdata(client, codec);

	codec->control_data = client;

	ret = wm8985_init(socdev);
	if(ret < 0) {
		pr_err("failed to initialise WM8985\n");
		goto err;
	}
	
	printk(KERN_INFO "wm8985_i2c_probe: WM8985 initialized\n");
	return 0;

err:
	kfree(codec);

	return ret;
}

static int __exit wm8985_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	i2c_detach_client(client);
	kfree(codec->reg_cache);
	kfree(client);
	return 0;
}

static const struct i2c_device_id wm8985_i2c_id[] = {
	{"wm87xx", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, wm8985_i2c_id);

static struct i2c_driver wm8985_i2c_driver = {
	.driver = {
		.name = "WM8985 I2C Codec",
		.owner = THIS_MODULE,
	},
	.probe		= wm8985_i2c_probe,
	.remove		= __exit_p(wm8985_i2c_remove),
	.id_table	= wm8985_i2c_id,
	.command =        NULL,
};

#endif

static int wm8985_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct wm8985_setup_data *setup;
	struct snd_soc_codec *codec;
	struct wm8985_priv *wm8985;
	int ret = 0;

	pr_info("WM8985 Audio Codec %s\n", WM8985_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	wm8985 = kzalloc(sizeof(struct wm8985_priv), GFP_KERNEL);
	if (wm8985 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = wm8985;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	wm8985_socdev = socdev;
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&wm8985_i2c_driver);
	if (ret != 0)
		printk(KERN_ERR "can't add i2c driver\n");
#else
	/* Add other interfaces here */
#endif
	return ret;
}

/* power down chip */
static int wm8985_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		wm8985_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	i2c_del_driver(&wm8985_i2c_driver);
#endif
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wm8985 = {
	.probe = 	wm8985_probe,
	.remove = 	wm8985_remove,
	.suspend = 	wm8985_suspend,
	.resume =	wm8985_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_wm8985);

static int __init wm8985_modinit(void)
{
	return snd_soc_register_dai(&wm8985_dai);
}
module_init(wm8985_modinit);

static void __exit wm8985_exit(void)
{
	snd_soc_unregister_dai(&wm8985_dai);
}
module_exit(wm8985_exit);

MODULE_DESCRIPTION("ASoC WM8985 driver");
MODULE_AUTHOR("Mike Arthur");
MODULE_LICENSE("GPL");
