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
#include <asm/hardware.h>
#include <asm/arch/audio.h>
#include <asm/io.h>
#include <asm/arch/spi-gpio.h>
#include "../codecs/uda1380.h"
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
#define SMDK_CRYSTAL_CLOCK 16934400

static int smdk2440_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->codec;

	DBG("Entered smdk2440_startup\n");

	return 0;
}

static int smdk2440_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->socdev->codec;

	DBG("Entered smdk2440_shutdown\n");

	return 0;
}

static int smdk2440_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	unsigned long iis_clkrate;
	int div, div256, div384, diff256, diff384, bclk, mclk;
	int ret;
	unsigned int rate=params_rate(params);

	DBG("Entered %s\n",__FUNCTION__);

	iis_clkrate = s3c24xx_i2s_get_clockrate();

	/* Using PCLK doesnt seem to suit audio particularly well on these cpu's
	 */

	div256 = iis_clkrate / (rate * 256);
	div384 = iis_clkrate / (rate * 384);

	if (((iis_clkrate / div256) - (rate * 256)) <
		((rate * 256) - (iis_clkrate / (div256 + 1)))) {
		diff256 = (iis_clkrate / div256) - (rate * 256);
	} else {
		div256++;
		diff256 = (iis_clkrate / div256) - (rate * 256);
	}

	if (((iis_clkrate / div384) - (rate * 384)) <
		((rate * 384) - (iis_clkrate / (div384 + 1)))) {
		diff384 = (iis_clkrate / div384) - (rate * 384);
	} else {
		div384++;
		diff384 = (iis_clkrate / div384) - (rate * 384);
	}

	DBG("diff256 %d, diff384 %d\n", diff256, diff384);

	if (diff256<=diff384) {
		DBG("Selected 256FS\n");
		div = div256 - 1;
		bclk = S3C2410_IISMOD_256FS;
	} else {
		DBG("Selected 384FS\n");
		div = div384 - 1;
		bclk = S3C2410_IISMOD_384FS;
	}

	/* set codec DAI configuration */
	ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->dai_ops.set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the audio system clock for DAC and ADC */
	ret = cpu_dai->dai_ops.set_sysclk(cpu_dai, S3C24XX_CLKSRC_PCLK,
		rate, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* set MCLK division for sample rate */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C24XX_DIV_MCLK, S3C2410_IISMOD_32FS );
	if (ret < 0)
		return ret;

	/* set BCLK division for sample rate */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C24XX_DIV_BCLK, bclk);
	if (ret < 0)
		return ret;

	/* set prescaler division for sample rate */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C24XX_DIV_PRESCALER,
		S3C24XX_PRESCALE(div,div));
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

	/* mic is connected to MICIN (via right channel of headphone jack) */
	{"MICIN", NULL, "Mic Jack"},
	{"MICIN", NULL, "Line Jack"},

	{NULL, NULL, NULL},
};

/*
 * Logic for a UDA1341 as attached to SMDK2440
 */
static int smdk2440_uda1341_init(struct snd_soc_codec *codec)
{
	int i, err;

	DBG("Staring smdk2440 init\n");

	/* Add smdk2440 specific widgets */
	snd_soc_dapm_new_controls(codec, smdk2440_dapm_widgets,
				  ARRAY_SIZE(smdk2440_dapm_widgets));

	/* Set up smdk2440 specific audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(codec, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_sync(codec);

	DBG("Ending smdk2440 init\n");

	return 0;
}

/* s3c24xx digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link s3c24xx_dai = {
	.name = "WM8731",
	.stream_name = "WM8731",
	.cpu_dai = &s3c24xx_i2s_dai,
	.codec_dai = &uda1380_dai,
	.init = smdk2440_uda1341_init,
	.ops = &smdk2440_ops,
};

/* smdk2440 audio machine driver */
static struct snd_soc_machine snd_soc_machine_smdk2440 = {
	.name = "SMDK2440",
	.dai_link = &s3c24xx_dai,
	.num_links = 1,
};

static struct uda1380_setup_data smdk2440_uda1380_setup = {
	.i2c_address = 0x00,
};

/* s3c24xx audio subsystem */
static struct snd_soc_device s3c24xx_snd_devdata = {
	.machine = &snd_soc_machine_smdk2440,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_uda1380,
	.codec_data = &smdk2440_uda1380_setup,
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
