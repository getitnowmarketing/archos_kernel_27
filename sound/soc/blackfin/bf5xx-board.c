/*
 * bf5xx_board.c  --  SoC audio for Blackfin Processors
 *
 * Copyright 2007 Analog Device Inc.
 *
 * Authors: Roy Huang <roy.huang@analog.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    8th June 2007   Initial version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <asm/dma.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <asm/gpio.h>
#include <asm/portmux.h>
#include "../codecs/ad1980.h"
#include "bf5xx-sport.h"
#include "bf5xx-pcm.h"
#include "bf5xx-ac97.h"

#define PIN_REQ_SPORT_0 {P_SPORT0_TFS, P_SPORT0_DTPRI, P_SPORT0_TSCLK, P_SPORT0_RFS, \
		 P_SPORT0_DRPRI, P_SPORT0_RSCLK, 0}

#define PIN_REQ_SPORT_1 {P_SPORT1_TFS, P_SPORT1_DTPRI, P_SPORT1_TSCLK, P_SPORT1_RFS, \
		 P_SPORT1_DRPRI, P_SPORT1_RSCLK, 0}


static int sport_num = CONFIG_SND_BF5XX_SPORT_NUM;

static struct sport_param sport_params[2] = {
	{
		.dma_rx_chan	= CH_SPORT0_RX,
		.dma_tx_chan	= CH_SPORT0_TX,
		.err_irq	= IRQ_SPORT0_ERROR,
		.regs		= (struct sport_register *)SPORT0_TCR1,
	},
	{
		.dma_rx_chan	= CH_SPORT1_RX,
		.dma_tx_chan	= CH_SPORT1_TX,
		.err_irq	= IRQ_SPORT1_ERROR,
		.regs		= (struct sport_register *)SPORT1_TCR1,
	}
};

struct sport_device *sport_handle;

static struct snd_soc_machine bf5xx_board;

static int bf5xx_board_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_soc_ops bf5xx_board_ops = {
	.startup = bf5xx_board_startup,
};

static int bf5xx_board_ac97_init(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_dai_link bf5xx_board_dai = {
	.name = "AC97",
	.stream_name = "AC97 HiFi",
	.cpu_dai = &bfin_ac97_dai,
	.codec_dai = &ad1980_dai,
	.init = bf5xx_board_ac97_init,
	.ops = &bf5xx_board_ops,
};

static int bf5xx_probe(struct platform_device *pdev)
{

	u16 sport_req[][7] = {PIN_REQ_SPORT_0, PIN_REQ_SPORT_1};

	if (peripheral_request_list(&sport_req[sport_num][0], "soc-audio")) {
		printk(KERN_ERR "Requesting Peripherals faild\n");
		return -EFAULT;
		}

#ifdef CONFIG_SND_BF5XX_HAVE_COLD_RESET
	/* Request PB3 as reset pin */
	if (gpio_request(CONFIG_SND_BF5XX_RESET_GPIO_NUM, "SND_AD198x RESET")) {
		printk(KERN_ERR "Failed to request GPIO_%d for reset\n",
				CONFIG_SND_BF5XX_RESET_GPIO_NUM);
		peripheral_free_list(&sport_req[sport_num][0]);
		return -1;
	}
	gpio_direction_output(CONFIG_SND_BF5XX_RESET_GPIO_NUM);
	gpio_set_value(CONFIG_SND_BF5XX_RESET_GPIO_NUM, 1);
#endif
	sport_handle = sport_init(&sport_params[sport_num], 2, \
			10 * sizeof(struct ac97_frame), NULL);
	if (!sport_handle) {
		peripheral_free_list(&sport_req[sport_num][0]);
#ifdef CONFIG_SND_BF5XX_HAVE_COLD_RESET
		gpio_free(CONFIG_SND_BF5XX_RESET_GPIO_NUM);
#endif
		return -ENODEV;
	}

	sport_set_multichannel(sport_handle, 16, 0x1F, 1);
	sport_config_rx(sport_handle, IRFS, 0xF, 0, (16*16-1));
	sport_config_tx(sport_handle, ITFS, 0xF, 0, (16*16-1));

	return 0;
}

static struct snd_soc_machine bf5xx_board = {
	.name = "bf5xx-board",
	.probe = bf5xx_probe,
	.dai_link = &bf5xx_board_dai,
	.num_links = 1,
};

static struct snd_soc_device bf5xx_board_snd_devdata = {
	.machine = &bf5xx_board,
	.platform = &bf5xx_soc_platform,
	.codec_dev = &soc_codec_dev_ad1980,
};

static struct platform_device *bf5xx_board_snd_device;

static int __init bf5xx_board_init(void)
{
	int ret;

	bf5xx_board_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bf5xx_board_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bf5xx_board_snd_device, &bf5xx_board_snd_devdata);
	bf5xx_board_snd_devdata.dev = &bf5xx_board_snd_device->dev;
	ret = platform_device_add(bf5xx_board_snd_device);

	if (ret)
		platform_device_put(bf5xx_board_snd_device);

	return ret;
}

static void __exit bf5xx_board_exit(void)
{
	platform_device_unregister(bf5xx_board_snd_device);
}

module_init(bf5xx_board_init);
module_exit(bf5xx_board_exit);

/* Module information */
MODULE_AUTHOR("Roy Huang");
MODULE_DESCRIPTION("ALSA SoC BF5xx Board");
MODULE_LICENSE("GPL");
