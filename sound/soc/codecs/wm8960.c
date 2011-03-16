/*
 * wm8960.c  --  WM8960 ALSA SoC Audio driver
 *
 * Author: Liam Girdwood
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
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

#include "wm8960.h"

#define AUDIO_NAME "wm8960"
#define WM8960_VERSION "0.2"


struct snd_soc_codec_device soc_codec_dev_wm8960;

/*
 * wm8960 register cache
 * We can't read the WM8960 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8960_reg[WM8960_CACHEREGNUM] = {
	0x0097, 0x0097, 0x0000, 0x0000,
	0x0000, 0x0008, 0x0000, 0x000a,
	0x01c0, 0x0000, 0x00ff, 0x00ff,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x007b, 0x0100, 0x0032,
	0x0000, 0x00c3, 0x00c3, 0x01c0,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0100, 0x0100, 0x0050, 0x0050,
	0x0050, 0x0050, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0040, 0x0000,
	0x0000, 0x0050, 0x0050, 0x0000,
	0x0002, 0x0037, 0x004d, 0x0080,
	0x0008, 0x0031, 0x0026, 0x00e9,
};

/*
 * read wm8960 register cache
 */
static inline unsigned int wm8960_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == WM8960_RESET)
		return 0;
	if (reg >= WM8960_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write wm8960 register cache
 */
static inline void wm8960_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WM8960_CACHEREGNUM)
		return;
	cache[reg] = value;
}

static inline unsigned int wm8960_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	return wm8960_read_reg_cache(codec, reg);
}

/*
 * write to the WM8960 register space
 */
static int wm8960_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	/* data is
	 *   D15..D9 WM8960 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	wm8960_write_reg_cache(codec, reg, value);
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -EIO;
}

#define wm8960_reset(c)	wm8960_write(c, WM8960_RESET, 0)

/* enumerated controls */
static const char *wm8960_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static const char *wm8960_polarity[] = {"No Inversion", "Left Inverted",
	"Right Inverted", "Stereo Inversion"};
static const char *wm8960_3d_upper_cutoff[] = {"High", "Low"};
static const char *wm8960_3d_lower_cutoff[] = {"Low", "High"};
static const char *wm8960_alcfunc[] = {"Off", "Right", "Left", "Stereo"};
static const char *wm8960_alcmode[] = {"ALC", "Limiter"};

static const struct soc_enum wm8960_enum[] = {
	SOC_ENUM_SINGLE(WM8960_DACCTL1, 1, 4, wm8960_deemph),
	SOC_ENUM_SINGLE(WM8960_DACCTL1, 5, 4, wm8960_polarity),
	SOC_ENUM_SINGLE(WM8960_DACCTL2, 5, 4, wm8960_polarity),
	SOC_ENUM_SINGLE(WM8960_3D, 6, 2, wm8960_3d_upper_cutoff),
	SOC_ENUM_SINGLE(WM8960_3D, 5, 2, wm8960_3d_lower_cutoff),
	SOC_ENUM_SINGLE(WM8960_ALC1, 7, 4, wm8960_alcfunc),
	SOC_ENUM_SINGLE(WM8960_ALC3, 8, 2, wm8960_alcmode),
};

/* to complete */
static const struct snd_kcontrol_new wm8960_snd_controls[] = {
SOC_DOUBLE_R("Capture Volume", WM8960_LINVOL, WM8960_RINVOL,
	0, 63, 0),
SOC_DOUBLE_R("Capture Volume ZC Switch", WM8960_LINVOL, WM8960_RINVOL,
	6, 1, 0),
SOC_DOUBLE_R("Capture Switch", WM8960_LINVOL, WM8960_RINVOL,
	7, 1, 0),
SOC_DOUBLE_R("Headphone Playback Volume", WM8960_LOUT1, WM8960_ROUT1,
	0, 127, 0),
SOC_DOUBLE_R("Headphone Playback ZC Switch", WM8960_LOUT1, WM8960_ROUT1,
	7, 1, 0),
SOC_SINGLE("PCM Playback -6dB Switch", WM8960_DACCTL1, 7, 1, 0),
SOC_ENUM("ADC Polarity", wm8960_enum[1]),
SOC_ENUM("Playback De-emphasis", wm8960_enum[0]),
SOC_SINGLE("ADC High Pass Filter Switch", WM8960_DACCTL1, 0, 1, 0),

SOC_ENUM("DAC Polarity", wm8960_enum[2]),

SOC_DOUBLE_R("PCM Volume", WM8960_LDAC, WM8960_RDAC,
	0, 127, 0),

SOC_ENUM("3D Filter Upper Cut-Off", wm8960_enum[3]),
SOC_ENUM("3D Filter Lower Cut-Off", wm8960_enum[4]),
SOC_SINGLE("3D Volume", WM8960_3D, 1, 15, 0),
SOC_SINGLE("3D Switch", WM8960_3D, 0, 1, 0),

SOC_ENUM("ALC Function", wm8960_enum[5]),
SOC_SINGLE("ALC Max Gain", WM8960_ALC1, 4, 7, 0),
SOC_SINGLE("ALC Target", WM8960_ALC1, 0, 15, 1),
SOC_SINGLE("ALC Min Gain", WM8960_ALC2, 4, 7, 0),
SOC_SINGLE("ALC Hold Time", WM8960_ALC2, 0, 15, 0),
SOC_ENUM("ALC Mode", wm8960_enum[6]),
SOC_SINGLE("ALC Decay", WM8960_ALC3, 4, 15, 0),
SOC_SINGLE("ALC Attack", WM8960_ALC3, 0, 15, 0),

SOC_SINGLE("Noise Gate Threshold", WM8960_NOISEG, 3, 31, 0),
SOC_SINGLE("Noise Gate Switch", WM8960_NOISEG, 0, 1, 0),

SOC_DOUBLE_R("ADC PCM Capture Volume", WM8960_LINPATH, WM8960_RINPATH,
	0, 127, 0),
};

/* add non dapm controls */
static int wm8960_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8960_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&wm8960_snd_controls[i],
					       codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

/* Left Output Mixer */
static const struct snd_kcontrol_new wm8960_loutput_mixer_controls[] = {
SOC_DAPM_SINGLE("Left PCM Playback Switch", WM8960_LOUTMIX1, 8, 1, 0),
};

/* Right Output Mixer */
static const struct snd_kcontrol_new wm8960_routput_mixer_controls[] = {
SOC_DAPM_SINGLE("Right PCM Playback Switch", WM8960_ROUTMIX2, 8, 1, 0),
};

static const struct snd_soc_dapm_widget wm8960_dapm_widgets[] = {
SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
	&wm8960_loutput_mixer_controls[0],
	ARRAY_SIZE(wm8960_loutput_mixer_controls)),
SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
	&wm8960_loutput_mixer_controls[0],
	ARRAY_SIZE(wm8960_routput_mixer_controls)),
};

static int wm8960_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, wm8960_dapm_widgets,
				  ARRAY_SIZE(wm8960_dapm_widgets));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int wm8960_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	/* set iface */
	wm8960_write(codec, WM8960_IFACE1, iface);
	return 0;
}

static int wm8960_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u16 iface = wm8960_read(codec, WM8960_IFACE1) & 0xfff3;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0008;
		break;
	}

	/* set iface */
	wm8960_write(codec, WM8960_IFACE1, iface);
	return 0;
}

static int wm8960_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = wm8960_read(codec, WM8960_DACCTL1) & 0xfff7;

	if (mute)
		wm8960_write(codec, WM8960_DACCTL1, mute_reg | 0x8);
	else
		wm8960_write(codec, WM8960_DACCTL1, mute_reg);
	return 0;
}

static int wm8960_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	u16 power1 = wm8960_read(codec, WM8960_POWER1);
	u16 power2 = wm8960_read(codec, WM8960_POWER2);

	switch (level) {
	case SND_SOC_BIAS_ON:
		power1 &= ~0x180;
		power1 |= 0xc0;
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		power1 &= ~0x180;
		power1 |= 0x140;
		wm8960_write(codec, WM8960_ADDCTL1,
			     wm8960_read(codec, WM8960_ADDCTL1) | 1);
		break;

	case SND_SOC_BIAS_OFF:
		wm8960_write(codec, WM8960_APOP1, 0x94);
		wm8960_write(codec, WM8960_POWER1, power1 & ~0x1c0);
		mdelay(600);  /* Ensure HP output has discharged */
		wm8960_write(codec, WM8960_ADDCTL1,
			     wm8960_read(codec, WM8960_ADDCTL1) & ~1);
		return 0;
	}

	wm8960_write(codec, WM8960_POWER1, power1 | 0x03e);
	wm8960_write(codec, WM8960_POWER2, power2 | 0x1fe);
	wm8960_write(codec, WM8960_POWER3, 0xffff);
	codec->bias_level = level;

	return 0;
}

/* PLL divisors */
struct _pll_div {
	u32 pre_div:1;
	u32 n:4;
	u32 k:24;
};

/* The size in bits of the pll divide multiplied by 10
 * to allow rounding later */
#define FIXED_PLL_SIZE ((1 << 24) * 10)

static int pll_factors(unsigned int source, unsigned int target,
		       struct _pll_div *pll_div)
{
	unsigned long long Kpart;
	unsigned int K, Ndiv, Nmod;

	pr_debug("WM8960 PLL: setting %dHz->%dHz\n", source, target);

	/* Scale up target to PLL operating frequency */
	target *= 4;

	Ndiv = target / source;
	if (Ndiv < 6) {
		source >>= 1;
		pll_div->pre_div = 1;
		Ndiv = target / source;
	} else
		pll_div->pre_div = 0;

	if ((Ndiv < 6) || (Ndiv > 12)) {
		pr_err("WM8960 PLL: Unsupported N=%d\n", Ndiv);
		return -EINVAL;
	}

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

	pr_debug("WM8960 PLL: N=%x K=%x pre_div=%d\n",
		 pll_div->n, pll_div->k, pll_div->pre_div);

	return 0;
}

static int wm8960_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;
	static struct _pll_div pll_div;
	int ret;

	if (freq_out) {
		ret = pll_factors(freq_in, freq_out, &pll_div);
		if (ret != 0)
			return ret;
	}

	/* Disable the PLL: even if we are changing the frequency the
	 * PLL needs to be disabled while we do so. */
	wm8960_write(codec, WM8960_CLOCK1,
		     wm8960_read(codec, WM8960_CLOCK1) & ~1);
	wm8960_write(codec, WM8960_POWER2,
		     wm8960_read(codec, WM8960_POWER2) & ~1);

	reg = wm8960_read(codec, WM8960_PLL1) & ~0x3f;
	reg |= pll_div.pre_div << 4;
	reg |= pll_div.n;

	if (pll_div.k) {
		reg |= 0x20;

		wm8960_write(codec, WM8960_PLL2, (pll_div.k >> 16) & 0x7f);
		wm8960_write(codec, WM8960_PLL3, (pll_div.k >> 8) & 0xff);
		wm8960_write(codec, WM8960_PLL4, pll_div.k & 0xff);
	}
	wm8960_write(codec, WM8960_PLL1, reg);

	/* Turn it on */
	wm8960_write(codec, WM8960_POWER2,
		     wm8960_read(codec, WM8960_POWER2) | 1);
	msleep(250);
	wm8960_write(codec, WM8960_CLOCK1,
		     wm8960_read(codec, WM8960_CLOCK1) | 1);

	return 0;
}

static int wm8960_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	switch (div_id) {
	case WM8960_SYSCLKSEL:
		reg = wm8960_read(codec, WM8960_CLOCK1) & 0x1fe;
		wm8960_write(codec, WM8960_CLOCK1, reg | div);
		break;
	case WM8960_SYSCLKDIV:
		reg = wm8960_read(codec, WM8960_CLOCK1) & 0x1f9;
		wm8960_write(codec, WM8960_CLOCK1, reg | div);
		break;
	case WM8960_DACDIV:
		reg = wm8960_read(codec, WM8960_CLOCK1) & 0x1c7;
		wm8960_write(codec, WM8960_CLOCK1, reg | div);
		break;
	case WM8960_OPCLKDIV:
		reg = wm8960_read(codec, WM8960_PLL1) & 0x03f;
		wm8960_write(codec, WM8960_PLL1, reg | div);
		break;
	case WM8960_DCLKDIV:
		reg = wm8960_read(codec, WM8960_CLOCK2) & 0x03f;
		wm8960_write(codec, WM8960_CLOCK2, reg | div);
		break;
	case WM8960_TOCLKSEL:
		reg = wm8960_read(codec, WM8960_ADDCTL1) & 0x1fd;
		wm8960_write(codec, WM8960_ADDCTL1, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define WM8960_RATES SNDRV_PCM_RATE_8000_48000

#define WM8960_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

struct snd_soc_dai wm8960_dai = {
	.name = "WM8960",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8960_RATES,
		.formats = WM8960_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8960_RATES,
		.formats = WM8960_FORMATS,},
	.ops = {
		.hw_params = wm8960_hw_params,
		.digital_mute = wm8960_mute,
		.set_fmt = wm8960_set_dai_fmt,
		.set_clkdiv = wm8960_set_dai_clkdiv,
		.set_pll = wm8960_set_dai_pll,
	},
};
EXPORT_SYMBOL_GPL(wm8960_dai);


/* To complete PM */
static int wm8960_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	wm8960_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int wm8960_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8960_reg); i++) {
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->hw_write(codec->control_data, data, 2);
	}

	wm8960_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	wm8960_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}

/*
 * initialise the WM8960 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int wm8960_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int reg, ret = 0;

	codec->name = "WM8960";
	codec->owner = THIS_MODULE;
	codec->read = wm8960_read;
	codec->write = wm8960_write;
	codec->set_bias_level = wm8960_set_bias_level;
	codec->dai = &wm8960_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = ARRAY_SIZE(wm8960_reg);
	codec->reg_cache = kmemdup(wm8960_reg, sizeof(wm8960_reg), GFP_KERNEL);

	if (codec->reg_cache == NULL)
		return -ENOMEM;

	wm8960_reset(codec);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "wm8960: failed to create pcms\n");
		goto pcm_err;
	}

	/* Initial power on sequence */
	wm8960_write(codec, WM8960_APOP1, 0x94);
	wm8960_write(codec, WM8960_APOP2, 0x40);
	msleep(400);   /* HP output discharge */
	wm8960_write(codec, WM8960_POWER2, 0x60);
	wm8960_write(codec, WM8960_APOP2, 0);
	wm8960_write(codec, WM8960_POWER1, 0x80);
	msleep(400);   /* VMID initial charge */
	wm8960_write(codec, WM8960_POWER1, 0x140);
	wm8960_write(codec, WM8960_APOP1, 0);

	wm8960_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/*  set the update bits */
	reg = wm8960_read(codec, WM8960_LOUT1);
	wm8960_write(codec, WM8960_LOUT1, reg | 0x0100);
	reg = wm8960_read(codec, WM8960_ROUT1);
	wm8960_write(codec, WM8960_ROUT1, reg | 0x0100);

	wm8960_add_controls(codec);
	wm8960_add_widgets(codec);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "wm8960: failed to register card\n");
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

static struct snd_soc_device *wm8960_socdev;

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)

/*
 * WM8960 2 wire address is 0x1a
 */
static unsigned short normal_i2c[] = { 0, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver wm8960_i2c_driver;
static struct i2c_client client_template;

/* If the i2c layer weren't so broken, we could pass this kind of data
   around */

static int wm8960_codec_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_device *socdev = wm8960_socdev;
	struct wm8960_setup_data *setup = socdev->codec_data;
	struct snd_soc_codec *codec = socdev->codec;
	struct i2c_client *i2c;
	int ret;

	if (addr != setup->i2c_address)
		return -ENODEV;

	client_template.adapter = adap;
	client_template.addr = addr;

	i2c = kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
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

	ret = wm8960_init(socdev);
	if (ret < 0) {
		pr_err("failed to initialise WM8960\n");
		goto err;
	}
	return ret;

err:
	kfree(codec);
	kfree(i2c);
	return ret;
}

static int wm8960_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	i2c_detach_client(client);
	kfree(codec->reg_cache);
	kfree(client);
	return 0;
}

static int wm8960_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, wm8960_codec_probe);
}

/* corgi i2c codec control layer */
static struct i2c_driver wm8960_i2c_driver = {
	.driver = {
		.name = "WM8960 I2C Codec",
		.owner = THIS_MODULE,
	},
	.attach_adapter = wm8960_i2c_attach,
	.detach_client =  wm8960_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "WM8960",
	.driver = &wm8960_i2c_driver,
};
#endif

static int wm8960_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct wm8960_setup_data *setup;
	struct snd_soc_codec *codec;
	int ret = 0;

	pr_info("WM8960 Audio Codec %s", WM8960_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	wm8960_socdev = socdev;
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	if (setup->i2c_address) {
		normal_i2c[0] = setup->i2c_address;
		codec->hw_write = (hw_write_t)i2c_master_send;
		ret = i2c_add_driver(&wm8960_i2c_driver);
		if (ret != 0)
			printk(KERN_ERR "can't add i2c driver");
	}
#else
	/* Add other interfaces here */
#endif
	return ret;
}

/* power down chip */
static int wm8960_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		wm8960_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&wm8960_i2c_driver);
#endif
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wm8960 = {
	.probe = 	wm8960_probe,
	.remove = 	wm8960_remove,
	.suspend = 	wm8960_suspend,
	.resume =	wm8960_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_wm8960);

static int __init wm8960_modinit(void)
{
	return snd_soc_register_dai(&wm8960_dai);
}
module_init(wm8960_modinit);

static void __exit wm8960_exit(void)
{
	snd_soc_unregister_dai(&wm8960_dai);
}
module_exit(wm8960_exit);

MODULE_DESCRIPTION("ASoC WM8960 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
