/*
 * smdk2440.c  --  ALSA Soc Audio Layer
 *
 * (c) 2006 Wolfson Microelectronics PLC.
 * Graeme Gregory graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * (c) 2004-2005 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 * This module is a modified version of the s3c24xx I2S driver supplied by
 * Ben Dooks of Simtec and rejigged to the ASoC style at Wolfson Microelectronics
 *
 *  Revision history
 *    11th Dec 2006   Merged with Simtec driver
 *    10th Nov 2006   Initial version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
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
#include <asm/arch/hardware.h>
#include <asm/arch/audio.h>
#include <asm/io.h>
#include <asm/arch/spi-gpio.h>
#include "../codecs/wm8956.h"
#include "s3c24xx-pcm.h"
#include "s3c24xx-i2s.h"

#define SMDK2440_DEBUG 0
#if SMDK2440_DEBUG
#define DBG(x...) printk(KERN_DEBUG x)
#else
#define DBG(x...)
#endif

/* audio clock in Hz */
#define SMDK_CLOCK_SOURCE S3C24XX_CLKSRC_MPLL
#define SMDK_CRYSTAL_CLOCK 12000000

static int smdk2440_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->codec;

	DBG("Entered %s\n",__FUNCTION__);

	return 0;
}

static int smdk2440_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->codec;

	DBG("Entered %s\n",__FUNCTION__);

	return 0;
}

static int smdk2440_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	int bclk, mclk;
	int ret;
	int pll;
	int div=0,sysclkdiv=0;
	unsigned int rate = params_rate(params);

	DBG("Entered %s\n",__FUNCTION__);

	/* Work out the pll dividers */
	switch(rate)
	{
		case 8000:
		case 16000:
		case 32000:
		case 48000:
			pll=12288000;
			break;
		case 96000:
			pll=24576000;
			break;
		case 11025:
		case 22050:
		case 44100:
			pll=11289600;
			break;
		case 88200:
			pll=22579200;
			break;
		default:
			pll=12288000;
	}

	/* Work out the DAV Div */
	switch(rate)
	{
		 case 96000:
		 case 88200:
		 case 48000:
		 case 44100:
		 	div=0;
		 	break;
		 case 32000:
		 	div=1;
		 	break;
		 case 22050;
		 	div=2;
		 	break;
		 case 16000:
		 	div=1;
		 	sysclkdiv=2;
			break;
		case 11025:
			div=4;
			break;
		case 8000:
			div=6;
			break;
	}

	/* set codec DAI configuration */
	ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	ret = codec_dai->dai_ops.set_pll(codec_dai, 0, SMDK_CRYSTAL_CLOCK, pll);
	if (ret < 0)
		return ret;

	ret = codec_dai->dai_ops.set_clkdiv(codec_dai, WM8956_SYSCLKDIV, sysclkdiv);
	if (ret < 0)
		return ret;

	ret = codec_dai->dai_ops.set_clkdiv(codec_dai, WM8956_DACDIV, div);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->dai_ops.set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set the audio system clock for DAC and ADC */
	/* 12Mhz crystal for this example */
	ret = cpu_dai->dai_ops.set_sysclk(cpu_dai, S3C24XX_CLKSRC_MPLL,
		SMDK_CRYSTAL_CLOCK, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* set MCLK division for sample rate */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C24XX_DIV_MCLK, S3C2410_IISMOD_32FS );
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops smdk2440_ops = {
	.startup = smdk2440_startup,
	.shutdown = smdk2440_shutdown,
	.hw_params = smdk2440_hw_params,
};

/* smdk2440 machine dapm widgets */
static const struct snd_soc_dapm_widget smdk2440_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_MIC("Mic Jack", NULL),
SND_SOC_DAPM_LINE("Line Jack", NULL),
};

/* smdk2440 machine audio map (connections to the codec pins) */
static const char* audio_map[][3] = {
	/* headphone connected to  HPOUT */
	{"Headphone Jack", NULL, "HPOUT"},
	{"MICIN", NULL, "Mic Jack"},
	{"MICIN", NULL, "Line Jack"},

	{NULL, NULL, NULL},
};

/*
 * Logic for a wm8956 as attached to SMDK2440
 */
static int smdk2440_wm8956_init(struct snd_soc_codec *codec)
{
	int i, err;

	DBG("Entered %s\n",__FUNCTION__);


	/* Add smdk2440 specific widgets */
	snd_soc_dapm_new_controls(codec, &smdk2440_dapm_widgets,
				 ARRAY_SIZE(smdk2440_dapm_widgets));

	/* Set up smdk2440 specific audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(codec, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_sync(codec);

	return 0;
}

/* s3c24xx digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link s3c24xx_dai = {
	.name = "WM8731",
	.stream_name = "WM8731",
	.cpu_dai = &s3c24xx_i2s_dai,
	.codec_dai = &wm8956_dai,
	.init = smdk2440_wm8956_init,
	.ops = &smdk2440_ops,
};

/* smdk2440 audio machine driver */
static struct snd_soc_machine snd_soc_machine_smdk2440 = {
	.name = "SMDK2440",
	.dai_link = &s3c24xx_dai,
	.num_links = 1,
};

static struct wm8956_setup_data smdk2440_wm8956_setup = {
	.i2c_address = 0x00,
};

/* s3c24xx audio subsystem */
static struct snd_soc_device s3c24xx_snd_devdata = {
	.machine = &snd_soc_machine_smdk2440,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8956,
	.codec_data = &smdk2440_wm8956_setup,
};

static struct platform_device *s3c24xx_snd_device;

struct smdk2440_spi_device {
	struct device *dev;
};

static struct smdk2440_spi_device smdk2440_spi_devdata = {
};

struct s3c2410_spigpio_info smdk2440_spi_devinfo = {
	.pin_clk = S3C2410_GPF4,
	.pin_mosi = S3C2410_GPF5,
	.pin_miso = S3C2410_GPF6,
	//.board_size,
	//.board_info,
	.chip_select=NULL,
};

static struct platform_device *smdk2440_spi_device;

static int __init smdk2440_init(void)
{
	int ret;

	if (!machine_is_smdk2440() && !machine_is_s3c2440()) {
		DBG("%d\n",machine_arch_type);
		DBG("Not a SMDK2440\n");
		return -ENODEV;
	}

	s3c24xx_snd_device = platform_device_alloc("soc-audio", -1);
	if (!s3c24xx_snd_device) {
		DBG("platform_dev_alloc failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(s3c24xx_snd_device, &s3c24xx_snd_devdata);
	s3c24xx_snd_devdata.dev = &s3c24xx_snd_device->dev;
	ret = platform_device_add(s3c24xx_snd_device);

	if (ret)
		platform_device_put(s3c24xx_snd_device);

	// Create a bitbanged SPI device

	smdk2440_spi_device = platform_device_alloc("s3c24xx-spi-gpio",-1);
	if (!smdk2440_spi_device) {
		DBG("smdk2440_spi_device : platform_dev_alloc failed\n");
		return -ENOMEM;
	}
	DBG("Return Code %d\n",ret);

	platform_set_drvdata(smdk2440_spi_device, &smdk2440_spi_devdata);
	smdk2440_spi_devdata.dev = &smdk2440_spi_device->dev;
	smdk2440_spi_devdata.dev->platform_data = &smdk2440_spi_devinfo;
	ret = platform_device_add(smdk2440_spi_device);

	if (ret)
		platform_device_put(smdk2440_spi_device);

	return ret;
}

static void __exit smdk2440_exit(void)
{
	platform_device_unregister(s3c24xx_snd_device);
}

module_init(smdk2440_init);
module_exit(smdk2440_exit);

/* Module information */
MODULE_AUTHOR("Ben Dooks, <ben@simtec.co.uk>");
MODULE_DESCRIPTION("ALSA SoC SMDK2440");
MODULE_LICENSE("GPL");
