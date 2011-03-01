/*
 * smdk2440_wm8991.c  --  SoC audio for smdk2440 with wm8991
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    10th Jul 2007   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>
#include <asm/arch/regs-iis.h>
#include <asm/arch/regs-clock.h>
#include <asm/arch/regs-gpio.h>
#include <asm/hardware.h>
#include <asm/arch/audio.h>
#include <asm/io.h>
#include <asm/arch/spi-gpio.h>
#include "../codecs/wm8991.h"
#include "s3c24xx-pcm.h"
#include "s3c24xx-i2s.h"

#define SMDK2440_DEBUG 0
#if SMDK2440_DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

static struct snd_soc_machine smdk2440;

static int smdk2440_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int pll_out = 0, bclk = 0, mclk = 0, dacclk = 0;
	int ret = 0;
	unsigned long iis_clkrate;

	iis_clkrate = s3c24xx_i2s_get_clockrate();

	printk("iis_clkrate %ld\n", iis_clkrate);

	switch (params_rate(params)) {
	case 96000:
		dacclk = WM8991_DAC_CLKDIV_1;
		bclk = WM8991_BCLK_DIV_4;
		mclk = WM8991_MCLK_DIV_1;
		pll_out = 12288000*2;
		break;
	case 48000:
		dacclk = WM8991_DAC_CLKDIV_1;
		bclk = WM8991_BCLK_DIV_4;
		mclk = WM8991_MCLK_DIV_2;
		pll_out = 12288000*2;
		break;
	case 16000:
		dacclk = WM8991_DAC_CLKDIV_1;
		bclk = WM8991_BCLK_DIV_12;
		mclk = WM8991_MCLK_DIV_2;
		pll_out = 12288000*2;
		break;
	case 8000:
		dacclk = WM8991_DAC_CLKDIV_1;
		bclk = WM8991_BCLK_DIV_24;
		mclk = WM8991_MCLK_DIV_2;
		pll_out = 12288000*2;
		break;
	case 88200:
		dacclk = WM8991_DAC_CLKDIV_1;
		bclk = WM8991_BCLK_DIV_4;
		mclk = WM8991_MCLK_DIV_1;
		pll_out = 11289600*2;
		break;
	case 44100:
		dacclk = WM8991_DAC_CLKDIV_1;
		bclk = WM8991_BCLK_DIV_4;
		mclk = WM8991_MCLK_DIV_2;
		pll_out = 11289600*2;
		break;
	case 22050:
		dacclk = WM8991_DAC_CLKDIV_1;
		bclk = WM8991_BCLK_DIV_8;
		mclk = WM8991_MCLK_DIV_2;
		pll_out = 11289600*2;
		break;
	case 11025:
		dacclk = WM8991_DAC_CLKDIV_1;
		bclk = WM8991_BCLK_DIV_16;
		mclk = WM8991_MCLK_DIV_2;
		pll_out = 11289600*2;
		break;

	}

	/* set codec DAI configuration */
	ret = codec_dai->dai_ops.set_fmt(codec_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->dai_ops.set_fmt(cpu_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set MCLK division for sample rate */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C24XX_DIV_MCLK,
		S3C2410_IISMOD_32FS );
	if (ret < 0)
		return ret;

	/* Setupd Codec Clocks */
	ret = codec_dai->dai_ops.set_clkdiv(codec_dai, WM8991_BCLK_DIV, bclk);
	if (ret < 0)
		return ret;

	ret = codec_dai->dai_ops.set_clkdiv(codec_dai, WM8991_MCLK_DIV, mclk);
	if (ret < 0)
		return ret;

	ret = codec_dai->dai_ops.set_clkdiv(codec_dai, WM8991_DACCLK_DIV, dacclk);
	if (ret < 0)
		return ret;

	/* set prescaler division for sample rate */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C24XX_DIV_PRESCALER,
		S3C24XX_PRESCALE(4,4));
	if (ret < 0)
		return ret;

	/* codec PLL input is PCLK/4 */

	ret = codec_dai->dai_ops.set_pll(codec_dai, 0, iis_clkrate/4,
		pll_out);
	if (ret < 0)
		return ret;

	return 0;
}

static int smdk2440_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;

	/* disable the PLL */
	return codec_dai->dai_ops.set_pll(codec_dai, 0, 0, 0);
}

/*
 * Hxd8 WM8976 HiFi DAI opserations.
 */
static struct snd_soc_ops smdk2440_ops = {
	.hw_params = smdk2440_hw_params,
	.hw_free = smdk2440_hw_free,
};

static const struct snd_soc_dapm_widget wm8976_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};


/* example machine audio_mapnections */
static const char* audio_map[][3] = {

	/* Connections to the lm4857 amp */
	{"Headphones", NULL, "LOUT"},
	{"Headphones", NULL, "ROUT"},

	{"LIN2", NULL, "Line In"},
	{"RIN2", NULL, "Line In"},

	{NULL, NULL, NULL},
};


/*
 * This is an example machine initialisation for a wm8991 connected to a
 * smdk2440.
 */

static int smdk2440_wm8991_init(struct snd_soc_codec *codec)
{
	int i;

	/* set up NC codec pins */
	snd_soc_dapm_enable_pin(codec, "Internal DAC Sink", 1);
	snd_soc_dapm_enable_pin(codec, "Internal ADC Source", 1);

	/* Add smdk2440 specific widgets */
	snd_soc_dapm_new_controls(codec, &wm8976_dapm_widgets,
				  ARRAY_SIZE(wm8976_dapm_widgets));

	/* set up smdk2440 specific audio path audio_map connects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(codec, audio_map[i][0], audio_map[i][1],
			audio_map[i][2]);
	}

	snd_soc_dapm_enable_pin(codec, "Headphones", 1);
	snd_soc_dapm_enable_pin(codec, "Line In", 1);

	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link smdk2440_dai[] = {
{ /* Hifi Playback */
	.name = "WM8991",
	.stream_name = "WM8991 Primary",
	.cpu_dai = &s3c24xx_i2s_dai,
	.codec_dai = &wm8991_dai,
	.init = smdk2440_wm8991_init,
	.ops = &smdk2440_ops,
},
};

static struct snd_soc_machine smdk2440 = {
	.name = "smdk2440",
	.dai_link = smdk2440_dai,
	.num_links = ARRAY_SIZE(smdk2440_dai),
};

/* spitz audio private data */
static struct wm8991_setup_data smdk2440_wm8991_setup = {
	.i2c_address = 0x34 >> 1,
};

static struct snd_soc_device smdk2440_snd_devdata = {
	.machine = &smdk2440,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8991,
	.codec_data = &smdk2440_wm8991_setup,
};

static struct platform_device *smdk2440_snd_device;

static int __init smdk2440_init(void)
{
	int ret;

	smdk2440_snd_device = platform_device_alloc("soc-audio", -1);
	if (!smdk2440_snd_device)
		return -ENOMEM;

	platform_set_drvdata(smdk2440_snd_device, &smdk2440_snd_devdata);
	smdk2440_snd_devdata.dev = &smdk2440_snd_device->dev;
	ret = platform_device_add(smdk2440_snd_device);

	if (ret)
		platform_device_put(smdk2440_snd_device);

	return ret;
}

static void __exit smdk2440_exit(void)
{
	platform_device_unregister(smdk2440_snd_device);
}

module_init(smdk2440_init);
module_exit(smdk2440_exit);

/* Module information */
MODULE_AUTHOR("Graeme Gregory, graeme.gregory@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("ALSA SoC WM8991 SMDK2440");
MODULE_LICENSE("GPL");
