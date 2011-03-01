/*
 * neo1973_gta02_wm8753.c  --  SoC audio for Neo1973
 *
 * Copyright 2007 OpenMoko Inc
 * Author: Graeme Gregory <graeme@openmoko.org>
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory <linux@wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    06th Nov 2007   Changed from GTA01 to GTA02
 *    20th Jan 2007   Initial version.
 *    05th Feb 2007   Rename all to Neo1973
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>
#include <asm/plat-s3c24xx/regs-iis.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/hardware.h>
#include <asm/arch/audio.h>
#include <linux/io.h>
#include <asm/arch/spi-gpio.h>
#include <asm/arch/regs-gpioj.h>
#include <asm/arch/gta02.h>
#include "../codecs/wm8753.h"
#include "s3c24xx-pcm.h"
#include "s3c24xx-i2s.h"

/* define the scenarios */
#define NEO_AUDIO_OFF			0
#define NEO_GSM_CALL_AUDIO_HANDSET	1
#define NEO_GSM_CALL_AUDIO_HEADSET	2
#define NEO_GSM_CALL_AUDIO_BLUETOOTH	3
#define NEO_STEREO_TO_SPEAKERS		4
#define NEO_STEREO_TO_HEADPHONES	5
#define NEO_CAPTURE_HANDSET		6
#define NEO_CAPTURE_HEADSET		7
#define NEO_CAPTURE_BLUETOOTH		8
#define NEO_STEREO_TO_HANDSET_SPK	9

static struct snd_soc_machine neo1973_gta02;

static int neo1973_gta02_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int pll_out = 0, bclk = 0;
	int ret = 0;
	unsigned long iis_clkrate;

	iis_clkrate = s3c24xx_i2s_get_clockrate();

	switch (params_rate(params)) {
	case 8000:
	case 16000:
		pll_out = 12288000;
		break;
	case 48000:
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 12288000;
		break;
	case 96000:
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 12288000;
		break;
	case 11025:
		bclk = WM8753_BCLK_DIV_16;
		pll_out = 11289600;
		break;
	case 22050:
		bclk = WM8753_BCLK_DIV_8;
		pll_out = 11289600;
		break;
	case 44100:
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 11289600;
		break;
	case 88200:
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 11289600;
		break;
	}

	/* set codec DAI configuration */
	ret = codec_dai->dai_ops.set_fmt(codec_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->dai_ops.set_fmt(cpu_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->dai_ops.set_sysclk(codec_dai, WM8753_MCLK, pll_out,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set MCLK division for sample rate */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C24XX_DIV_MCLK,
		S3C2410_IISMOD_32FS);
	if (ret < 0)
		return ret;

	/* set codec BCLK division for sample rate */
	ret = codec_dai->dai_ops.set_clkdiv(codec_dai,
					WM8753_BCLKDIV, bclk);
	if (ret < 0)
		return ret;

	/* set prescaler division for sample rate */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C24XX_DIV_PRESCALER,
		S3C24XX_PRESCALE(4, 4));
	if (ret < 0)
		return ret;

	/* codec PLL input is PCLK/4 */
	ret = codec_dai->dai_ops.set_pll(codec_dai, WM8753_PLL1,
		iis_clkrate / 4, pll_out);
	if (ret < 0)
		return ret;

	return 0;
}

static int neo1973_gta02_hifi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;

	/* disable the PLL */
	return codec_dai->dai_ops.set_pll(codec_dai, WM8753_PLL1, 0, 0);
}

/*
 * Neo1973 WM8753 HiFi DAI opserations.
 */
static struct snd_soc_ops neo1973_gta02_hifi_ops = {
	.hw_params = neo1973_gta02_hifi_hw_params,
	.hw_free = neo1973_gta02_hifi_hw_free,
};

static int neo1973_gta02_voice_hw_params(
	struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	unsigned int pcmdiv = 0;
	int ret = 0;
	unsigned long iis_clkrate;

	iis_clkrate = s3c24xx_i2s_get_clockrate();

	if (params_rate(params) != 8000)
		return -EINVAL;
	if (params_channels(params) != 1)
		return -EINVAL;

	pcmdiv = WM8753_PCM_DIV_6; /* 2.048 MHz */

	/* todo: gg check mode (DSP_B) against CSR datasheet */
	/* set codec DAI configuration */
	ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_B |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->dai_ops.set_sysclk(codec_dai, WM8753_PCMCLK,
		12288000, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec PCM division for sample rate */
	ret = codec_dai->dai_ops.set_clkdiv(codec_dai, WM8753_PCMDIV,
					pcmdiv);
	if (ret < 0)
		return ret;

	/* configue and enable PLL for 12.288MHz output */
	ret = codec_dai->dai_ops.set_pll(codec_dai, WM8753_PLL2,
		iis_clkrate / 4, 12288000);
	if (ret < 0)
		return ret;

	return 0;
}

static int neo1973_gta02_voice_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;

	/* disable the PLL */
	return codec_dai->dai_ops.set_pll(codec_dai, WM8753_PLL2, 0, 0);
}

static struct snd_soc_ops neo1973_gta02_voice_ops = {
	.hw_params = neo1973_gta02_voice_hw_params,
	.hw_free = neo1973_gta02_voice_hw_free,
};

#define LM4853_AMP 1
#define LM4853_SPK 2

static u8 lm4853_state;

static int lm4853_set_state(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int val = ucontrol->value.integer.value[0];

	if (val) {
		lm4853_state |= LM4853_AMP;
		s3c2410_gpio_setpin(GTA02_GPIO_AMP_SHUT, 0);
	} else {
		lm4853_state &= ~LM4853_AMP;
		s3c2410_gpio_setpin(GTA02_GPIO_AMP_SHUT, 1);
	}

	return 0;
}

static int lm4853_get_state(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = lm4853_state & LM4853_AMP;

	return 0;
}

static int lm4853_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int val = ucontrol->value.integer.value[0];

	if (val) {
		lm4853_state |= LM4853_SPK;
		s3c2410_gpio_setpin(GTA02_GPIO_HP_IN, 0);
	} else {
		lm4853_state &= ~LM4853_SPK;
		s3c2410_gpio_setpin(GTA02_GPIO_HP_IN, 1);
	}

	return 0;
}

static int lm4853_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = (lm4853_state & LM4853_SPK) >> 1;

	return 0;
}

static int neo1973_gta02_set_stereo_out(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_enable_pin(codec, "Stereo Out", val);

	snd_soc_dapm_sync(codec);

	return 0;
}

static int neo1973_gta02_get_stereo_out(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint_status(codec, "Stereo Out");

	return 0;
}


static int neo1973_gta02_set_gsm_out(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_enable_pin(codec, "GSM Line Out", val);

	snd_soc_dapm_sync(codec);

	return 0;
}

static int neo1973_gta02_get_gsm_out(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint_status(codec, "GSM Line Out");

	return 0;
}

static int neo1973_gta02_set_gsm_in(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_enable_pin(codec, "GSM Line In", val);

	snd_soc_dapm_sync(codec);

	return 0;
}

static int neo1973_gta02_get_gsm_in(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint_status(codec, "GSM Line In");

	return 0;
}

static int neo1973_gta02_set_headset_mic(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_enable_pin(codec, "Headset Mic", val);

	snd_soc_dapm_sync(codec);

	return 0;
}

static int neo1973_gta02_get_headset_mic(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint_status(codec, "Headset Mic");

	return 0;
}

static int neo1973_gta02_set_handset_mic(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_enable_pin(codec, "Handset Mic", val);

	snd_soc_dapm_sync(codec);

	return 0;
}

static int neo1973_gta02_get_handset_mic(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint_status(codec, "Handset Mic");

	return 0;
}

static int neo1973_gta02_set_handset_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int val = ucontrol->value.integer.value[0];

	snd_soc_dapm_enable_pin(codec, "Handset Spk", val);

	snd_soc_dapm_sync(codec);

	return 0;
}

static int neo1973_gta02_get_handset_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ucontrol->value.integer.value[0] =
		snd_soc_dapm_get_endpoint_status(codec, "Handset Spk");

	return 0;
}

static const struct snd_soc_dapm_widget wm8753_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Stereo Out", NULL),
	SND_SOC_DAPM_LINE("GSM Line Out", NULL),
	SND_SOC_DAPM_LINE("GSM Line In", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_SPK("Handset Spk", NULL),
};


/* example machine audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {

	/* Connections to the lm4853 amp */
	{"Stereo Out", NULL, "LOUT1"},
	{"Stereo Out", NULL, "ROUT1"},

	/* Connections to the GSM Module */
	{"GSM Line Out", NULL, "MONO1"},
	{"GSM Line Out", NULL, "MONO2"},
	{"RXP", NULL, "GSM Line In"},
	{"RXN", NULL, "GSM Line In"},

	/* Connections to Headset */
	{"MIC1", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Headset Mic"},

	/* Call Mic */
	{"MIC2", NULL, "Mic Bias"},
	{"MIC2N", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Handset Mic"},

	/* Call Speaker */
	{"Handset Spk", NULL, "LOUT2"},
	{"Handset Spk", NULL, "ROUT2"},

	/* Connect the ALC pins */
	{"ACIN", NULL, "ACOP"},
};

static const struct snd_kcontrol_new wm8753_neo1973_gta02_controls[] = {
	SOC_SINGLE_EXT("DAPM Stereo Out Switch", 0, 0, 1, 0,
		neo1973_gta02_get_stereo_out,
		neo1973_gta02_set_stereo_out),
	SOC_SINGLE_EXT("DAPM GSM Line Out Switch", 1, 0, 1, 0,
		neo1973_gta02_get_gsm_out,
		neo1973_gta02_set_gsm_out),
	SOC_SINGLE_EXT("DAPM GSM Line In Switch", 2, 0, 1, 0,
		neo1973_gta02_get_gsm_in,
		neo1973_gta02_set_gsm_in),
	SOC_SINGLE_EXT("DAPM Headset Mic Switch", 3, 0, 1, 0,
		neo1973_gta02_get_headset_mic,
		neo1973_gta02_set_headset_mic),
	SOC_SINGLE_EXT("DAPM Handset Mic Switch", 4, 0, 1, 0,
		neo1973_gta02_get_handset_mic,
		neo1973_gta02_set_handset_mic),
	SOC_SINGLE_EXT("DAPM Handset Spk Switch", 5, 0, 1, 0,
		neo1973_gta02_get_handset_spk,
		neo1973_gta02_set_handset_spk),
	SOC_SINGLE_EXT("Amp State Switch", 6, 0, 1, 0,
		lm4853_get_state,
		lm4853_set_state),
	SOC_SINGLE_EXT("Amp Spk Switch", 7, 0, 1, 0,
		lm4853_get_spk,
		lm4853_set_spk),
};

/*
 * This is an example machine initialisation for a wm8753 connected to a
 * neo1973 GTA02.
 */
static int neo1973_gta02_wm8753_init(struct snd_soc_codec *codec)
{
	int i, err;

	/* set up NC codec pins */
	snd_soc_dapm_enable_pin(codec, "OUT3",  0);
	snd_soc_dapm_enable_pin(codec, "OUT4",  0);
	snd_soc_dapm_enable_pin(codec, "LINE1", 0);
	snd_soc_dapm_enable_pin(codec, "LINE2", 0);

	/* Add neo1973 gta02 specific widgets */
	snd_soc_dapm_new_control(codec, wm8753_dapm_widgets,
				 ARRAY_SIZE(wm8753_dapm_widgets));

	/* add neo1973 gta02 specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8753_neo1973_gta02_controls); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&wm8753_neo1973_gta02_controls[i],
			codec, NULL));
		if (err < 0)
			return err;
	}

	/* set up neo1973 gta02 specific audio path audio_mapnects */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* set endpoints to default off mode */
	snd_soc_dapm_enable_pin(codec, "Stereo Out",   0);
	snd_soc_dapm_enable_pin(codec, "GSM Line Out", 0);
	snd_soc_dapm_enable_pin(codec, "GSM Line In",  0);
	snd_soc_dapm_enable_pin(codec, "Headset Mic",  0);
	snd_soc_dapm_enable_pin(codec, "Handset Mic",  0);
	snd_soc_dapm_enable_pin(codec, "Handset Spk",  0);

	snd_soc_dapm_sync(codec);
	return 0;
}

/*
 * BT Codec DAI
 */
static struct snd_soc_cpu_dai bt_dai = {
	.name = "Bluetooth",
	.id = 0,
	.type = SND_SOC_DAI_PCM,
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
};

static struct snd_soc_dai_link neo1973_gta02_dai[] = {
{ /* Hifi Playback - for similatious use with voice below */
	.name = "WM8753",
	.stream_name = "WM8753 HiFi",
	.cpu_dai = &s3c24xx_i2s_dai,
	.codec_dai = &wm8753_dai[WM8753_DAI_HIFI],
	.init = neo1973_gta02_wm8753_init,
	.ops = &neo1973_gta02_hifi_ops,
},
{ /* Voice via BT */
	.name = "Bluetooth",
	.stream_name = "Voice",
	.cpu_dai = &bt_dai,
	.codec_dai = &wm8753_dai[WM8753_DAI_VOICE],
	.ops = &neo1973_gta02_voice_ops,
},
};

#ifdef CONFIG_PM
int neo1973_gta02_suspend(struct platform_device *pdev, pm_message_t state)
{
	s3c2410_gpio_setpin(GTA02_GPIO_AMP_SHUT, 1);

	return 0;
}

int neo1973_gta02_resume(struct platform_device *pdev)
{
	if (lm4853_state & LM4853_AMP)
		s3c2410_gpio_setpin(GTA02_GPIO_AMP_SHUT, 0);

	return 0;
}
#else
#define neo1973_gta02_suspend NULL
#define neo1973_gta02_resume NULL
#endif

static struct snd_soc_machine neo1973_gta02 = {
	.name = "neo1973-gta02",
	.suspend_pre = neo1973_gta02_suspend,
	.resume_post = neo1973_gta02_resume,
	.dai_link = neo1973_gta02_dai,
	.num_links = ARRAY_SIZE(neo1973_gta02_dai),
};

static struct wm8753_setup_data neo1973_gta02_wm8753_setup = {
	.i2c_address = 0x1a,
};

static struct snd_soc_device neo1973_gta02_snd_devdata = {
	.machine = &neo1973_gta02,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8753,
	.codec_data = &neo1973_gta02_wm8753_setup,
};

static struct platform_device *neo1973_gta02_snd_device;

static int __init neo1973_gta02_init(void)
{
	int ret;

	if (!machine_is_neo1973_gta02()) {
		printk(KERN_INFO
		       "Only GTA02 hardware supported by ASoc driver\n");
		return -ENODEV;
	}

	neo1973_gta02_snd_device = platform_device_alloc("soc-audio", -1);
	if (!neo1973_gta02_snd_device)
		return -ENOMEM;

	platform_set_drvdata(neo1973_gta02_snd_device,
			&neo1973_gta02_snd_devdata);
	neo1973_gta02_snd_devdata.dev = &neo1973_gta02_snd_device->dev;
	ret = platform_device_add(neo1973_gta02_snd_device);

	if (ret)
		platform_device_put(neo1973_gta02_snd_device);

	/* Initialise GPIOs used by amp */
	s3c2410_gpio_cfgpin(GTA02_GPIO_HP_IN, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_cfgpin(GTA02_GPIO_AMP_SHUT, S3C2410_GPIO_OUTPUT);

	/* Amp off by default */
	s3c2410_gpio_setpin(GTA02_GPIO_AMP_SHUT, 1);

	/* Speaker off by default */
	s3c2410_gpio_setpin(GTA02_GPIO_HP_IN, 1);

	return ret;
}

static void __exit neo1973_gta02_exit(void)
{
	platform_device_unregister(neo1973_gta02_snd_device);
}

module_init(neo1973_gta02_init);
module_exit(neo1973_gta02_exit);

/* Module information */
MODULE_AUTHOR("Graeme Gregory, graeme@openmoko.org");
MODULE_DESCRIPTION("ALSA SoC WM8753 Neo1973 GTA02");
MODULE_LICENSE("GPL");

