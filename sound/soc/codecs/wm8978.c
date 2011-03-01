/*
 * wm8978.c  --  WM8978 ALSA Soc Audio driver
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 *
 * Authors: Liam Girdwood <lg@opensource.wolfsonmicro.com>
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

#include "wm8978.h"

#define AUDIO_NAME "wm8978"
#define WM8978_VERSION "0.1"

struct snd_soc_codec_device soc_codec_dev_wm8978;

/*
 * wm8978 register cache
 * We can't read the WM8978 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8978_reg[WM8978_CACHEREGNUM] = {
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0050, 0x0000, 0x0140, 0x0000,
    0x0000, 0x0000, 0x0000, 0x00ff,
    0x00ff, 0x0000, 0x0100, 0x00ff,
    0x00ff, 0x0000, 0x012c, 0x002c,
    0x002c, 0x002c, 0x002c, 0x0000,
    0x0032, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0038, 0x000b, 0x0032, 0x0000,
    0x0008, 0x000c, 0x0093, 0x00e9,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0033, 0x0010, 0x0010, 0x0100,
    0x0100, 0x0002, 0x0001, 0x0001,
    0x0039, 0x0039, 0x0039, 0x0039,
    0x0001, 0x0001,
};

/*
 * read wm8978 register cache
 */
static inline unsigned int wm8978_read_reg_cache(struct snd_soc_codec  *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == WM8978_RESET)
		return 0;
	if (reg >= WM8978_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write wm8978 register cache
 */
static inline void wm8978_write_reg_cache(struct snd_soc_codec  *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WM8978_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the WM8978 register space
 */
static int wm8978_write(struct snd_soc_codec  *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	/* data is
	 *   D15..D9 WM8978 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	wm8978_write_reg_cache (codec, reg, value);
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -1;
}

#define wm8978_reset(c)	wm8978_write(c, WM8978_RESET, 0)

static const char *wm8978_companding[] = {"Off", "NC", "u-law", "A-law" };
static const char *wm8978_deemp[] = {"None", "32kHz", "44.1kHz", "48kHz" };
static const char *wm8978_eqmode[] = {"Capture", "Playback" };
static const char *wm8978_bw[] = {"Narrow", "Wide" };
static const char *wm8978_eq1[] = {"80Hz", "105Hz", "135Hz", "175Hz" };
static const char *wm8978_eq2[] = {"230Hz", "300Hz", "385Hz", "500Hz" };
static const char *wm8978_eq3[] = {"650Hz", "850Hz", "1.1kHz", "1.4kHz" };
static const char *wm8978_eq4[] = {"1.8kHz", "2.4kHz", "3.2kHz", "4.1kHz" };
static const char *wm8978_eq5[] = {"5.3kHz", "6.9kHz", "9kHz", "11.7kHz" };
static const char *wm8978_alc[] =
    {"ALC both on", "ALC left only", "ALC right only", "Limiter" };

static const struct soc_enum wm8978_enum[] = {
	SOC_ENUM_SINGLE(WM8978_COMP, 1, 4, wm8978_companding), /* adc */
	SOC_ENUM_SINGLE(WM8978_COMP, 3, 4, wm8978_companding), /* dac */
	SOC_ENUM_SINGLE(WM8978_DAC,  4, 4, wm8978_deemp),
	SOC_ENUM_SINGLE(WM8978_EQ1,  8, 2, wm8978_eqmode),

	SOC_ENUM_SINGLE(WM8978_EQ1,  5, 4, wm8978_eq1),
	SOC_ENUM_SINGLE(WM8978_EQ2,  8, 2, wm8978_bw),
	SOC_ENUM_SINGLE(WM8978_EQ2,  5, 4, wm8978_eq2),
	SOC_ENUM_SINGLE(WM8978_EQ3,  8, 2, wm8978_bw),

	SOC_ENUM_SINGLE(WM8978_EQ3,  5, 4, wm8978_eq3),
	SOC_ENUM_SINGLE(WM8978_EQ4,  8, 2, wm8978_bw),
	SOC_ENUM_SINGLE(WM8978_EQ4,  5, 4, wm8978_eq4),
	SOC_ENUM_SINGLE(WM8978_EQ5,  8, 2, wm8978_bw),

	SOC_ENUM_SINGLE(WM8978_EQ5,  5, 4, wm8978_eq5),
	SOC_ENUM_SINGLE(WM8978_ALC3,  8, 2, wm8978_alc),
};

static const struct snd_kcontrol_new wm8978_snd_controls[] = {
SOC_SINGLE("Digital Loopback Switch", WM8978_COMP, 0, 1, 0),

SOC_ENUM("ADC Companding", wm8978_enum[0]),
SOC_ENUM("DAC Companding", wm8978_enum[1]),

SOC_SINGLE("Jack Detection Enable", WM8978_JACK1, 6, 1, 0),

SOC_SINGLE("DAC Right Inversion Switch", WM8978_DAC, 1, 1, 0),
SOC_SINGLE("DAC Left Inversion Switch", WM8978_DAC, 0, 1, 0),

SOC_SINGLE("Left Playback Volume", WM8978_DACVOLL, 0, 127, 0),
SOC_SINGLE("Right Playback Volume", WM8978_DACVOLR, 0, 127, 0),

SOC_SINGLE("High Pass Filter Switch", WM8978_ADC, 8, 1, 0),
SOC_SINGLE("High Pass Filter Switch", WM8978_ADC, 8, 1, 0),
SOC_SINGLE("High Pass Cut Off", WM8978_ADC, 4, 7, 0),
SOC_SINGLE("Right ADC Inversion Switch", WM8978_ADC, 1, 1, 0),
SOC_SINGLE("Left ADC Inversion Switch", WM8978_ADC, 0, 1, 0),

SOC_SINGLE("Left Capture Volume", WM8978_ADCVOLL,  0, 127, 0),
SOC_SINGLE("Right Capture Volume", WM8978_ADCVOLR,  0, 127, 0),

SOC_ENUM("Equaliser Function", wm8978_enum[3]),
SOC_ENUM("EQ1 Cut Off", wm8978_enum[4]),
SOC_SINGLE("EQ1 Volume", WM8978_EQ1,  0, 31, 1),

SOC_ENUM("Equaliser EQ2 Bandwith", wm8978_enum[5]),
SOC_ENUM("EQ2 Cut Off", wm8978_enum[6]),
SOC_SINGLE("EQ2 Volume", WM8978_EQ2,  0, 31, 1),

SOC_ENUM("Equaliser EQ3 Bandwith", wm8978_enum[7]),
SOC_ENUM("EQ3 Cut Off", wm8978_enum[8]),
SOC_SINGLE("EQ3 Volume", WM8978_EQ3,  0, 31, 1),

SOC_ENUM("Equaliser EQ4 Bandwith", wm8978_enum[9]),
SOC_ENUM("EQ4 Cut Off", wm8978_enum[10]),
SOC_SINGLE("EQ4 Volume", WM8978_EQ4,  0, 31, 1),

SOC_ENUM("Equaliser EQ5 Bandwith", wm8978_enum[11]),
SOC_ENUM("EQ5 Cut Off", wm8978_enum[12]),
SOC_SINGLE("EQ5 Volume", WM8978_EQ5,  0, 31, 1),

SOC_SINGLE("DAC Playback Limiter Switch", WM8978_DACLIM1,  8, 1, 0),
SOC_SINGLE("DAC Playback Limiter Decay", WM8978_DACLIM1,  4, 15, 0),
SOC_SINGLE("DAC Playback Limiter Attack", WM8978_DACLIM1,  0, 15, 0),

SOC_SINGLE("DAC Playback Limiter Threshold", WM8978_DACLIM2,  4, 7, 0),
SOC_SINGLE("DAC Playback Limiter Boost", WM8978_DACLIM2,  0, 15, 0),

SOC_SINGLE("ALC Enable Switch", WM8978_ALC1,  8, 1, 0),
SOC_SINGLE("ALC Capture Max Gain", WM8978_ALC1,  3, 7, 0),
SOC_SINGLE("ALC Capture Min Gain", WM8978_ALC1,  0, 7, 0),

SOC_SINGLE("ALC Capture ZC Switch", WM8978_ALC2,  8, 1, 0),
SOC_SINGLE("ALC Capture Hold", WM8978_ALC2,  4, 7, 0),
SOC_SINGLE("ALC Capture Target", WM8978_ALC2,  0, 15, 0),

SOC_ENUM("ALC Capture Mode", wm8978_enum[13]),
SOC_SINGLE("ALC Capture Decay", WM8978_ALC3,  4, 15, 0),
SOC_SINGLE("ALC Capture Attack", WM8978_ALC3,  0, 15, 0),

SOC_SINGLE("ALC Capture Noise Gate Switch", WM8978_NGATE,  3, 1, 0),
SOC_SINGLE("ALC Capture Noise Gate Threshold", WM8978_NGATE,  0, 7, 0),

SOC_SINGLE("Left Capture PGA ZC Switch", WM8978_INPPGAL,  7, 1, 0),
SOC_SINGLE("Left Capture PGA Volume", WM8978_INPPGAL,  0, 63, 0),

SOC_SINGLE("Right Capture PGA ZC Switch", WM8978_INPPGAR,  7, 1, 0),
SOC_SINGLE("Right Capture PGA Volume", WM8978_INPPGAR,  0, 63, 0),

SOC_SINGLE("Left Headphone Playback ZC Switch", WM8978_HPVOLL,  7, 1, 0),
SOC_SINGLE("Left Headphone Playback Switch", WM8978_HPVOLL,  6, 1, 1),
SOC_SINGLE("Left Headphone Playback Volume", WM8978_HPVOLL,  0, 63, 0),

SOC_SINGLE("Right Headphone Playback ZC Switch", WM8978_HPVOLR,  7, 1, 0),
SOC_SINGLE("Right Headphone Playback Switch", WM8978_HPVOLR,  6, 1, 1),
SOC_SINGLE("Right Headphone Playback Volume", WM8978_HPVOLR,  0, 63, 0),

SOC_SINGLE("Left Speaker Playback ZC Switch", WM8978_SPKVOLL,  7, 1, 0),
SOC_SINGLE("Left Speaker Playback Switch", WM8978_SPKVOLL,  6, 1, 1),
SOC_SINGLE("Left Speaker Playback Volume", WM8978_SPKVOLL,  0, 63, 0),

SOC_SINGLE("Right Speaker Playback ZC Switch", WM8978_SPKVOLR,  7, 1, 0),
SOC_SINGLE("Right Speaker Playback Switch", WM8978_SPKVOLR,  6, 1, 1),
SOC_SINGLE("Right Speaker Playback Volume", WM8978_SPKVOLR,  0, 63, 0),

SOC_DOUBLE_R("Capture Boost(+20dB)", WM8978_ADCBOOSTL, WM8978_ADCBOOSTR,
	8, 1, 0),
};

/* add non dapm controls */
static int wm8978_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8978_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm8978_snd_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

/* Left Output Mixer */
static const struct snd_kcontrol_new wm8978_left_mixer_controls[] = {
SOC_DAPM_SINGLE("Right PCM Playback Switch", WM8978_OUTPUT, 6, 1, 1),
SOC_DAPM_SINGLE("Left PCM Playback Switch", WM8978_MIXL, 0, 1, 1),
SOC_DAPM_SINGLE("Line Bypass Switch", WM8978_MIXL, 1, 1, 0),
SOC_DAPM_SINGLE("Aux Playback Switch", WM8978_MIXL, 5, 1, 0),
};

/* Right Output Mixer */
static const struct snd_kcontrol_new wm8978_right_mixer_controls[] = {
SOC_DAPM_SINGLE("Left PCM Playback Switch", WM8978_OUTPUT, 5, 1, 1),
SOC_DAPM_SINGLE("Right PCM Playback Switch", WM8978_MIXR, 0, 1, 1),
SOC_DAPM_SINGLE("Line Bypass Switch", WM8978_MIXR, 1, 1, 0),
SOC_DAPM_SINGLE("Aux Playback Switch", WM8978_MIXR, 5, 1, 0),
};

/* Left AUX Input boost vol */
static const struct snd_kcontrol_new wm8978_laux_boost_controls =
SOC_DAPM_SINGLE("Left Aux Volume", WM8978_ADCBOOSTL, 0, 3, 0);

/* Right AUX Input boost vol */
static const struct snd_kcontrol_new wm8978_raux_boost_controls =
SOC_DAPM_SINGLE("Right Aux Volume", WM8978_ADCBOOSTR, 0, 3, 0);

/* Left Input boost vol */
static const struct snd_kcontrol_new wm8978_lmic_boost_controls =
SOC_DAPM_SINGLE("Left Input Volume", WM8978_ADCBOOSTL, 4, 3, 0);

/* Right Input boost vol */
static const struct snd_kcontrol_new wm8978_rmic_boost_controls =
SOC_DAPM_SINGLE("Right Input Volume", WM8978_ADCBOOSTR, 4, 3, 0);

/* Left Aux In to PGA */
static const struct snd_kcontrol_new wm8978_laux_capture_boost_controls =
SOC_DAPM_SINGLE("Left Capture Switch", WM8978_ADCBOOSTL,  8, 1, 0);

/* Right  Aux In to PGA */
static const struct snd_kcontrol_new wm8978_raux_capture_boost_controls =
SOC_DAPM_SINGLE("Right Capture Switch", WM8978_ADCBOOSTR,  8, 1, 0);

/* Left Input P In to PGA */
static const struct snd_kcontrol_new wm8978_lmicp_capture_boost_controls =
SOC_DAPM_SINGLE("Left Input P Capture Boost Switch", WM8978_INPUT,  0, 1, 0);

/* Right Input P In to PGA */
static const struct snd_kcontrol_new wm8978_rmicp_capture_boost_controls =
SOC_DAPM_SINGLE("Right Input P Capture Boost Switch", WM8978_INPUT,  4, 1, 0);

/* Left Input N In to PGA */
static const struct snd_kcontrol_new wm8978_lmicn_capture_boost_controls =
SOC_DAPM_SINGLE("Left Input N Capture Boost Switch", WM8978_INPUT,  1, 1, 0);

/* Right Input N In to PGA */
static const struct snd_kcontrol_new wm8978_rmicn_capture_boost_controls =
SOC_DAPM_SINGLE("Right Input N Capture Boost Switch", WM8978_INPUT,  5, 1, 0);

// TODO Widgets
static const struct snd_soc_dapm_widget wm8978_dapm_widgets[] = {
#if 0
//SND_SOC_DAPM_MUTE("Mono Mute", WM8978_MONOMIX, 6, 0),
//SND_SOC_DAPM_MUTE("Speaker Mute", WM8978_SPKMIX, 6, 0),

SND_SOC_DAPM_MIXER("Speaker Mixer", WM8978_POWER3, 2, 0,
	&wm8978_speaker_mixer_controls[0],
	ARRAY_SIZE(wm8978_speaker_mixer_controls)),
SND_SOC_DAPM_MIXER("Mono Mixer", WM8978_POWER3, 3, 0,
	&wm8978_mono_mixer_controls[0],
	ARRAY_SIZE(wm8978_mono_mixer_controls)),
SND_SOC_DAPM_DAC("DAC", "HiFi Playback", WM8978_POWER3, 0, 0),
SND_SOC_DAPM_ADC("ADC", "HiFi Capture", WM8978_POWER3, 0, 0),
SND_SOC_DAPM_PGA("Aux Input", WM8978_POWER1, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("SpkN Out", WM8978_POWER3, 5, 0, NULL, 0),
SND_SOC_DAPM_PGA("SpkP Out", WM8978_POWER3, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mono Out", WM8978_POWER3, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mic PGA", WM8978_POWER2, 2, 0, NULL, 0),

SND_SOC_DAPM_PGA("Aux Boost", SND_SOC_NOPM, 0, 0,
	&wm8978_aux_boost_controls, 1),
SND_SOC_DAPM_PGA("Mic Boost", SND_SOC_NOPM, 0, 0,
	&wm8978_mic_boost_controls, 1),
SND_SOC_DAPM_SWITCH("Capture Boost", SND_SOC_NOPM, 0, 0,
	&wm8978_capture_boost_controls),

SND_SOC_DAPM_MIXER("Boost Mixer", WM8978_POWER2, 4, 0, NULL, 0),

SND_SOC_DAPM_MICBIAS("Mic Bias", WM8978_POWER1, 4, 0),

SND_SOC_DAPM_INPUT("MICN"),
SND_SOC_DAPM_INPUT("MICP"),
SND_SOC_DAPM_INPUT("AUX"),
SND_SOC_DAPM_OUTPUT("MONOOUT"),
SND_SOC_DAPM_OUTPUT("SPKOUTP"),
SND_SOC_DAPM_OUTPUT("SPKOUTN"),
#endif
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Mono output mixer */
	{"Mono Mixer", "PCM Playback Switch", "DAC"},
	{"Mono Mixer", "Aux Playback Switch", "Aux Input"},
	{"Mono Mixer", "Line Bypass Switch", "Boost Mixer"},

	/* Speaker output mixer */
	{"Speaker Mixer", "PCM Playback Switch", "DAC"},
	{"Speaker Mixer", "Aux Playback Switch", "Aux Input"},
	{"Speaker Mixer", "Line Bypass Switch", "Boost Mixer"},

	/* Outputs */
	{"Mono Out", NULL, "Mono Mixer"},
	{"MONOOUT", NULL, "Mono Out"},
	{"SpkN Out", NULL, "Speaker Mixer"},
	{"SpkP Out", NULL, "Speaker Mixer"},
	{"SPKOUTN", NULL, "SpkN Out"},
	{"SPKOUTP", NULL, "SpkP Out"},

	/* Boost Mixer */
	{"Boost Mixer", NULL, "ADC"},
	{"Capture Boost Switch", "Aux Capture Boost Switch", "AUX"},
	{"Aux Boost", "Aux Volume", "Boost Mixer"},
	{"Capture Boost", "Capture Switch", "Boost Mixer"},
	{"Mic Boost", "Mic Volume", "Boost Mixer"},

	/* Inputs */
	{"MICP", NULL, "Mic Boost"},
	{"MICN", NULL, "Mic PGA"},
	{"Mic PGA", NULL, "Capture Boost"},
	{"AUX", NULL, "Aux Input"},
};

static int wm8978_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, wm8978_dapm_widgets,
				  ARRAY_SIZE(wm8978_dapm_widgets));

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
	{12000000, 11289600, 0, 7, 0x86c220},
	{12000000, 12288000, 0, 8, 0x3126e8},
	{13000000, 11289600, 0, 6, 0xf28bd4},
	{13000000, 12288000, 0, 7, 0x8fd525},
	{12288000, 11289600, 0, 7, 0x59999a},
	{11289600, 12288000, 0, 8, 0x80dee9},
	/* TODO: liam - add more entries */
};

static int wm8978_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	int i;
	u16 reg;

	if(freq_in == 0 || freq_out == 0) {
		reg = wm8978_read_reg_cache(codec, WM8978_POWER1);
		wm8978_write(codec, WM8978_POWER1, reg & 0x1df);
		return 0;
	}

	for(i = 0; i < ARRAY_SIZE(pll); i++) {
		if (freq_in == pll[i].in_hz && freq_out == pll[i].out_hz) {
			wm8978_write(codec, WM8978_PLLN, (pll[i].pre << 4) | pll[i].n);
			wm8978_write(codec, WM8978_PLLK1, pll[i].k >> 18);
			wm8978_write(codec, WM8978_PLLK1, (pll[i].k >> 9) && 0x1ff);
			wm8978_write(codec, WM8978_PLLK1, pll[i].k && 0x1ff);
			reg = wm8978_read_reg_cache(codec, WM8978_POWER1);
			wm8978_write(codec, WM8978_POWER1, reg | 0x020);
			return 0;
		}
	}
	return -EINVAL;
}

static int wm8978_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = wm8978_read_reg_cache(codec, WM8978_IFACE) & 0x3;
	u16 clk = wm8978_read_reg_cache(codec, WM8978_CLOCK) & 0xfffe;

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
		iface |= 0x00018;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0180;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0100;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0080;
		break;
	default:
		return -EINVAL;
	}

	wm8978_write(codec, WM8978_IFACE, iface);
	return 0;
}

static int wm8978_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u16 iface = wm8978_read_reg_cache(codec, WM8978_IFACE) & 0xff9f;
	u16 adn = wm8978_read_reg_cache(codec, WM8978_ADD) & 0x1f1;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0020;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0040;
		break;
	}

	/* filter coefficient */
	switch (params_rate(params)) {
	case SNDRV_PCM_RATE_8000:
		adn |= 0x5 << 1;
		break;
	case SNDRV_PCM_RATE_11025:
		adn |= 0x4 << 1;
		break;
	case SNDRV_PCM_RATE_16000:
		adn |= 0x3 << 1;
		break;
	case SNDRV_PCM_RATE_22050:
		adn |= 0x2 << 1;
		break;
	case SNDRV_PCM_RATE_32000:
		adn |= 0x1 << 1;
		break;
	}

	/* set iface */
	wm8978_write(codec, WM8978_IFACE, iface);
	wm8978_write(codec, WM8978_ADD, adn);
	return 0;
}

static int wm8978_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	switch (div_id) {
	case WM8978_MCLKDIV:
		reg = wm8978_read_reg_cache(codec, WM8978_CLOCK) & 0x11f;
		wm8978_write(codec, WM8978_CLOCK, reg | div);
		break;
	case WM8978_BCLKDIV:
		reg = wm8978_read_reg_cache(codec, WM8978_CLOCK) & 0x1c7;
		wm8978_write(codec, WM8978_CLOCK, reg | div);
		break;
	case WM8978_OPCLKDIV:
		reg = wm8978_read_reg_cache(codec, WM8978_GPIO) & 0x1cf;
		wm8978_write(codec, WM8978_GPIO, reg | div);
		break;
	case WM8978_DACOSR:
		reg = wm8978_read_reg_cache(codec, WM8978_DAC) & 0x1f7;
		wm8978_write(codec, WM8978_DAC, reg | div);
		break;
	case WM8978_ADCOSR:
		reg = wm8978_read_reg_cache(codec, WM8978_ADC) & 0x1f7;
		wm8978_write(codec, WM8978_ADC, reg | div);
		break;
	case WM8978_MCLKSEL:
		reg = wm8978_read_reg_cache(codec, WM8978_CLOCK) & 0x0ff;
		wm8978_write(codec, WM8978_CLOCK, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wm8978_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = wm8978_read_reg_cache(codec, WM8978_DAC) & 0xffbf;

	if(mute)
		wm8978_write(codec, WM8978_DAC, mute_reg | 0x40);
	else
		wm8978_write(codec, WM8978_DAC, mute_reg);

	return 0;
}

/* TODO: liam need to make this lower power with dapm */
static int wm8978_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{

	switch (level) {
	case SND_SOC_BIAS_ON:
		wm8978_write(codec, WM8978_POWER1, 0x1ff);
		wm8978_write(codec, WM8978_POWER2, 0x1ff);
		wm8978_write(codec, WM8978_POWER3, 0x1ff);
		break;
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
		wm8978_write(codec, WM8978_POWER1, 0x0);
		wm8978_write(codec, WM8978_POWER2, 0x0);
		wm8978_write(codec, WM8978_POWER3, 0x0);
		break;
	}
	codec->bias_level = level;
	return 0;
}

#define WM8978_RATES SNDRV_PCM_RATE_8000_48000

#define WM8978_FORMATS \
	(SNDRV_PCM_FORMAT_S16_LE | SNDRV_PCM_FORMAT_S20_3LE | \
	SNDRV_PCM_FORMAT_S24_3LE | SNDRV_PCM_FORMAT_S24_LE)

struct snd_soc_dai wm8978_dai = {
	.name = "WM8978 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8978_RATES,
		.formats = WM8978_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8978_RATES,
		.formats = WM8978_FORMATS,},
	.ops = {
		.hw_params = wm8978_hw_params,
		.digital_mute = wm8978_mute,
		.set_fmt = wm8978_set_dai_fmt,
		.set_clkdiv = wm8978_set_dai_clkdiv,
		.set_pll = wm8978_set_dai_pll,
	},
};
EXPORT_SYMBOL_GPL(wm8978_dai);

static int wm8978_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	wm8978_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int wm8978_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8978_reg); i++) {
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->hw_write(codec->control_data, data, 2);
	}
	wm8978_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	wm8978_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}

/*
 * initialise the WM8978 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int wm8978_init(struct snd_soc_device* socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;

	codec->name = "WM8978";
	codec->owner = THIS_MODULE;
	codec->read = wm8978_read_reg_cache;
	codec->write = wm8978_write;
	codec->set_bias_level = wm8978_set_bias_level;
	codec->dai = &wm8978_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(wm8978_reg);
	codec->reg_cache = kmemdup(wm8978_reg, sizeof(wm8978_reg), GFP_KERNEL);

	if (codec->reg_cache == NULL)
		return -ENOMEM;

	wm8978_reset(codec);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if(ret < 0) {
		printk(KERN_ERR "wm8978: failed to create pcms\n");
		goto pcm_err;
	}

	/* power on device */
	wm8978_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	wm8978_add_controls(codec);
	wm8978_add_widgets(codec);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "wm8978: failed to register card\n");
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

static struct snd_soc_device *wm8978_socdev;

#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)

/*
 * WM8978 2 wire address is 0x1a
 */
#define I2C_DRIVERID_WM8978 0xfefe /* liam -  need a proper id */

static unsigned short normal_i2c[] = { 0, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver wm8978_i2c_driver;
static struct i2c_client client_template;

/* If the i2c layer weren't so broken, we could pass this kind of data
   around */

static int wm8978_codec_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_device *socdev = wm8978_socdev;
	struct wm8978_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *i2c;
	int ret;

	if (addr != setup->i2c_address)
		return -ENODEV;

	client_template.adapter = adap;
	client_template.addr = addr;

	i2c = kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL){
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

	ret = wm8978_init(socdev);
	if (ret < 0) {
		pr_err("failed to initialise WM8978\n");
		goto err;
	}
	return ret;

err:
	kfree(codec);
	kfree(i2c);
	return ret;

}

static int wm8978_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	i2c_detach_client(client);
	kfree(codec->reg_cache);
	kfree(client);
	return 0;
}

static int wm8978_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, wm8978_codec_probe);
}

/* corgi i2c codec control layer */
static struct i2c_driver wm8978_i2c_driver = {
	.driver = {
		.name = "WM8978 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_WM8978,
	.attach_adapter = wm8978_i2c_attach,
	.detach_client =  wm8978_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "WM8978",
	.driver = &wm8978_i2c_driver,
};
#endif

static int wm8978_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct wm8978_setup_data *setup;
	struct snd_soc_codec *codec;
	int ret = 0;

	pr_info("WM8978 Audio Codec %s", WM8978_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	wm8978_socdev = socdev;
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		normal_i2c[0] = setup->i2c_address;
		codec->hw_write = (hw_write_t)i2c_master_send;
		ret = i2c_add_driver(&wm8978_i2c_driver);
		if (ret != 0)
			printk(KERN_ERR "can't add i2c driver");
	}
#else
	/* Add other interfaces here */
#endif
	return ret;
}

/* power down chip */
static int wm8978_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		wm8978_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined (CONFIG_I2C) || defined (CONFIG_I2C_MODULE)
	i2c_del_driver(&wm8978_i2c_driver);
#endif
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wm8978 = {
	.probe = 	wm8978_probe,
	.remove = 	wm8978_remove,
	.suspend = 	wm8978_suspend,
	.resume =	wm8978_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_wm8978);

static int __init wm8978_modinit(void)
{
	return snd_soc_register_dai(&wm8978_dai);
}
module_init(wm8978_modinit);

static void __exit wm8978_exit(void)
{
	snd_soc_unregister_dai(&wm8978_dai);
}
module_exit(wm8978_exit);

MODULE_DESCRIPTION("ASoC WM8978 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
