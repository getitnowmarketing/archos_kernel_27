/*
 * bf5xx_wm8750.c  --  SoC audio for bf5xx WM8750
 *
 * Copyright 2007 Analog Device Inc.
 * Copyright 2007 Wolfson Microelectronics PLC
 *
 * Based on bf548_ezkit.c by Roy Huang <roy.huang@analog.com>
 *          
 * Authors:  Liam Girdwood <lg@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    29th Aug 2007   Initial version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>

#include <asm/dma.h>
#include <asm/gpio.h>
#include <asm/portmux.h>
#include "../codecs/wm8750.h"
#include "bf5xx-sport.h"
#include "bf5xx-pcm.h"
#include "bf5xx-i2s.h"

#define BF53X_WM8750_DEBUG

#ifdef  BF53X_WM8750_DEBUG
#define printd(format, arg...) printk("bfin-wm8750: " format, ## arg)
#endif


#define PIN_REQ_SPORT_0 {P_SPORT0_TFS, P_SPORT0_DTPRI, P_SPORT0_TSCLK, P_SPORT0_RFS, \
		 P_SPORT0_DRPRI, P_SPORT0_RSCLK, 0}

#define PIN_REQ_SPORT_1 {P_SPORT1_TFS, P_SPORT1_DTPRI, P_SPORT1_TSCLK, P_SPORT1_RFS, \
		 P_SPORT1_DRPRI, P_SPORT1_RSCLK, 0}

#define PIN_REQ_SPORT_2 {P_SPORT2_TFS, P_SPORT2_DTPRI, P_SPORT2_TSCLK, P_SPORT2_RFS, \
		 P_SPORT2_DRPRI, P_SPORT2_RSCLK, 0}

#define PIN_REQ_SPORT_3 {P_SPORT3_TFS, P_SPORT3_DTPRI, P_SPORT3_TSCLK, P_SPORT3_RFS, \
		 P_SPORT3_DRPRI, P_SPORT3_RSCLK, 0}


static int sport_num = CONFIG_SND_BF5XX_SPORT_NUM;

static struct sport_param sport_params[4] = {
	{
		.dma_rx_chan	= CH_SPORT0_RX,
		.dma_tx_chan	= CH_SPORT0_TX,
		.err_irq	= IRQ_SPORT0_ERR,
		.regs		= (struct sport_register*)SPORT0_TCR1,
	},
	{
		.dma_rx_chan	= CH_SPORT1_RX,
		.dma_tx_chan	= CH_SPORT1_TX,
		.err_irq	= IRQ_SPORT1_ERR,
		.regs		= (struct sport_register*)SPORT1_TCR1,
	},
	{
		.dma_rx_chan	= CH_SPORT2_RX,
		.dma_tx_chan	= CH_SPORT2_TX,
		.err_irq	= IRQ_SPORT2_ERR,
		.regs		= (struct sport_register*)SPORT2_TCR1,
	},
	{
		.dma_rx_chan	= CH_SPORT3_RX,
		.dma_tx_chan	= CH_SPORT3_TX,
		.err_irq	= IRQ_SPORT3_ERR,
		.regs		= (struct sport_register*)SPORT3_TCR1,
	}
};

struct sport_device *sport_handle;

static struct snd_soc_machine bf5xx_wm8750;

static int bf5xx_wm8750_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	
	cpu_dai->private_data = sport_handle;		
	return 0;
}

static int bf5xx_wm8750_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int clk = 0;
	int ret = 0;

	printd("%s rate %d format %x\n", __func__, params_rate(params), 
		params_format(params));
	/*
	 * WARNING - TODO
	 * 
	 * This code assumes there is a variable clocksource for the WM8750.
	 * i.e. it supplies MCLK depending on rate.
	 * 
	 * If you are using a crystal source then modify the below case 
	 * statement with a static frequency.
	 * 
	 * If you are using the SPORT to generate clocking then this is 
	 * where to do it. 
	 */

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 48000:
	case 96000:
		clk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
		clk = 11289600;
		break;
	}

	/*
	 * CODEC is master for BCLK and LRC in this configuration.
	 */

	/* set codec DAI configuration */
	ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->dai_ops.set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->dai_ops.set_sysclk(codec_dai, WM8750_SYSCLK, clk,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops bf5xx_wm8750_ops = {
	.startup = bf5xx_wm8750_startup,
	.hw_params = bf5xx_wm8750_hw_params,
};

static int bf5xx_wm8750_init_dev(struct snd_soc_codec *codec)
{
	/* 
	 * NC codec pins -
	 * Add any NC codec pins as follows :-
	 * 
	 * snd_soc_dapm_disable_pin(codec, "RINPUT1");
	 */
	printd("%s\n", __func__);
	snd_soc_dapm_sync(codec);
	return 0;
}

static struct snd_soc_dai_link bf5xx_wm8750_dai = {
	.name = "wm8750",
	.stream_name = "WM8750",
	.cpu_dai = &bf5xx_i2s_dai[CONFIG_SND_BF5XX_SPORT_NUM],
	.codec_dai = &wm8750_dai,
	.init = bf5xx_wm8750_init_dev,
	.ops = &bf5xx_wm8750_ops,
};

static int bf5xx_probe(struct platform_device *pdev)
{

	u16 sport_req[][7] = {PIN_REQ_SPORT_0, PIN_REQ_SPORT_1,
				 PIN_REQ_SPORT_2, PIN_REQ_SPORT_3};
	
	printd("%s\n", __func__);
	if (peripheral_request_list(&sport_req[sport_num][0], "soc-audio")) {
		printk(KERN_ERR "Requesting Peripherals faild\n");
		return -EFAULT;
	}

	/* TODO: not sure of correct dummy size */
	sport_handle = sport_init(&sport_params[sport_num], 2, \
			10 * sizeof(u16), NULL);
	if (!sport_handle) {
		peripheral_free_list(&sport_req[sport_num][0]);
		return -ENODEV;
	}

	return 0;
}

static struct snd_soc_machine bf5xx_wm8750 = {
	.name = "bf5xx-wm8750",
	.probe = bf5xx_probe,
	.dai_link = &bf5xx_wm8750_dai,
	.num_links = 1,
};

static struct snd_soc_device bf5xx_wm8750_snd_devdata = {
	.machine = &bf5xx_wm8750,
	.platform = &bf5xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8750,
};

static struct platform_device *bf54xx_wm8750_snd_device;

static int __init bf5xx_wm8750_init(void)
{
	int ret;
	
	printd("%s\n", __func__);
	bf54xx_wm8750_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bf54xx_wm8750_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bf54xx_wm8750_snd_device, &bf5xx_wm8750_snd_devdata);
	bf5xx_wm8750_snd_devdata.dev = &bf54xx_wm8750_snd_device->dev;
	ret = platform_device_add(bf54xx_wm8750_snd_device);

	if (ret)
		platform_device_put(bf54xx_wm8750_snd_device);

	return ret;
}

static void __exit bf5xx_wm8750_exit(void)
{
	printd("%s\n", __func__);
	platform_device_unregister(bf54xx_wm8750_snd_device);
}

module_init(bf5xx_wm8750_init);
module_exit(bf5xx_wm8750_exit);

/* Module information */
MODULE_AUTHOR("Roy Huang");
MODULE_DESCRIPTION("ALSA SoC BF548-EZKIT");
MODULE_LICENSE("GPL");
