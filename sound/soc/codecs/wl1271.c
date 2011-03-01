/*
 * wl1271.c  --  WL1271 (FM/Bluetooth) ALSA Soc Audio driver
 *
 * Copyright 2009 Archos SA.
 *
 * Authors:
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

#include "wl1271.h"

/*
 * There is no real interaction with the codec here since the codec
 * will be configured in userland using HCI stuff by the Android FM stack...
 * This file allows to register the codec with the SoC layer, that's all.
 */

#define AUDIO_NAME "wl1271"
#define WL1271_VERSION "0.1"

/* codec private data */
struct wl1271_priv {
	/* inclk is the input clock, (not the generated one when the WM8985 is Master) */
	unsigned int inclk;
};

static int wl1271_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	return 0;
}

static int wl1271_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	return 0;
}

static int wl1271_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

#define WL1271_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)

#define WL1271_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE)

struct snd_soc_dai wl1271_dai = {
	.name = "WL1271 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WL1271_RATES,
		.formats = WL1271_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WL1271_RATES,
		.formats = WL1271_FORMATS,},
	.ops = {
		.hw_params = wl1271_hw_params,
		.digital_mute = wl1271_mute,
		.set_fmt = wl1271_set_dai_fmt,
	},
};
EXPORT_SYMBOL_GPL(wl1271_dai);

static int wl1271_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
	default:
		break;
	}
	codec->bias_level = level;
	return 0;
}


static int wl1271_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	wl1271_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int wl1271_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	wl1271_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	wl1271_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}

/*
 * initialise the WL1271 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int wl1271_init(struct snd_soc_device* socdev)
{
	struct snd_soc_codec *codec = socdev->codec;
	int ret = 0;

	codec->name = "WL1271";
	codec->owner = THIS_MODULE;
	codec->set_bias_level = wl1271_set_bias_level;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->dai = &wl1271_dai;
	codec->num_dai = 1;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if(ret < 0) {
		printk(KERN_ERR "wl1271: failed to create pcms\n");
		goto pcm_err;
	}

	/* power on device */
	wl1271_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

//	wm8985_add_controls(codec);
//	wm8985_add_widgets(codec);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
      	printk(KERN_ERR "wl1271: failed to register card\n");
		goto card_err;
    }
	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	return ret;
}

static int wl1271_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct wl1271_setup_data *setup;
	struct snd_soc_codec *codec;
	struct wl1271_priv *wl1271;
	int ret = 0;

	pr_info("WL1271 (FM/Bluetooth) Audio Codec %s\n", WL1271_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	wl1271 = kzalloc(sizeof(struct wl1271_priv), GFP_KERNEL);
	if (wl1271 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	codec->private_data = wl1271;
	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

//	wl1271_socdev = socdev;
	wl1271_init(socdev);

	return ret;
}

/* power down chip */
static int wl1271_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec->control_data)
		wl1271_set_bias_level(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wl1271 = {
	.probe = 	wl1271_probe,
	.remove = 	wl1271_remove,
	.suspend = 	wl1271_suspend,
	.resume =	wl1271_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_wl1271);

static int __init wl1271_modinit(void)
{
	return snd_soc_register_dai(&wl1271_dai);
}
module_init(wl1271_modinit);

static void __exit wl1271_exit(void)
{
	snd_soc_unregister_dai(&wl1271_dai);
}
module_exit(wl1271_exit);

MODULE_DESCRIPTION("ASoC WL1271 (FM/BT) driver");
MODULE_AUTHOR("Jean-Christophe Rona");
MODULE_LICENSE("GPL");
