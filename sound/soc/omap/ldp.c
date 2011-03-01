/*
 * ldp.c  --  SoC audio for LDP
 *
 * Author: Misael Lopez Cruz <x0052729@ti.com>
 *
 * Based on:
 * Author: Steve Sakoman <steve@sakoman.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/twl4030.h"

static int ldp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops ldp_ops = {
	.hw_params = ldp_hw_params,
};

/* LDP machine DAPM */
static const struct snd_soc_dapm_widget ldp_twl4030_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Ext Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_HP("Headset Jack", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* External Mics: MAINMIC, SUBMIC with bias*/
	{"MAINMIC", NULL, "Mic Bias 1"},
	{"SUBMIC", NULL, "Mic Bias 2"},
	{"Mic Bias 1", NULL, "Ext Mic"},
	{"Mic Bias 2", NULL, "Ext Mic"},

	/* External Speakers: HFL, HFR */
	{"Ext Spk", NULL, "HFL"},
	{"Ext Spk", NULL, "HFR"},

	/* Headset: HSMIC (with bias), HSOL, HSOR */
	{"Headset Jack", NULL, "HSOL"},
	{"Headset Jack", NULL, "HSOR"},
	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headset Jack"},
};

static int ldp_twl4030_init(struct snd_soc_codec *codec)
{
	int ret;

	/* Add LDP specific widgets */
	ret = snd_soc_dapm_new_controls(codec, ldp_twl4030_dapm_widgets,
				ARRAY_SIZE(ldp_twl4030_dapm_widgets));
	if (ret)
		return ret;

	/* Set up LDP specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* LDP connected pins */
	snd_soc_dapm_enable_pin(codec, "Ext Mic");
	snd_soc_dapm_enable_pin(codec, "Ext Spk");
	snd_soc_dapm_enable_pin(codec, "Headset Jack");

	/* TWL4030 not connected pins */
	snd_soc_dapm_nc_pin(codec, "AUXL");
	snd_soc_dapm_nc_pin(codec, "AUXR");
	snd_soc_dapm_nc_pin(codec, "CARKITMIC");
	snd_soc_dapm_nc_pin(codec, "DIGIMIC0");
	snd_soc_dapm_nc_pin(codec, "DIGIMIC1");

	snd_soc_dapm_nc_pin(codec, "OUTL");
	snd_soc_dapm_nc_pin(codec, "OUTR");
	snd_soc_dapm_nc_pin(codec, "EARPIECE");
	snd_soc_dapm_nc_pin(codec, "PREDRIVEL");
	snd_soc_dapm_nc_pin(codec, "PREDRIVER");
	snd_soc_dapm_nc_pin(codec, "CARKITL");
	snd_soc_dapm_nc_pin(codec, "CARKITR");

	ret = snd_soc_dapm_sync(codec);

	return ret;
}

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link ldp_dai[] = {
	{
		.name = "TWL4030",
		.stream_name = "TWL4030",
		.cpu_dai = &omap_mcbsp_dai[0],
		.codec_dai = &twl4030_dai[0],
		.init = ldp_twl4030_init,
		.ops = &ldp_ops,
	},
#ifdef CONFIG_SND_SOC_TWL4030_FM
	{
		.name = "TWL4030_FM",
		.stream_name = "TWL4030_FM",
		.cpu_dai = &omap_mcbsp_dai[1],
		.codec_dai = &twl4030_dai[1],
		.ops = &ldp_ops,
	},
#endif
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_ldp = {
	.name = "LDP",
	.platform = &omap_soc_platform,
	.dai_link = ldp_dai,
	.num_links = ARRAY_SIZE(ldp_dai),
};

/* Audio subsystem */
static struct snd_soc_device ldp_snd_devdata = {
	.card = &snd_soc_ldp,
	.codec_dev = &soc_codec_dev_twl4030,
};

static struct platform_device *ldp_snd_device;

static int __init ldp_soc_init(void)
{
	int ret;

	if (!machine_is_omap_ldp()) {
		pr_debug("Not LDP!\n");
		return -ENODEV;
	}
	printk(KERN_INFO "LDP SoC init\n");

	ldp_snd_device = platform_device_alloc("soc-audio", -1);
	if (!ldp_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(ldp_snd_device, &ldp_snd_devdata);
	ldp_snd_devdata.dev = &ldp_snd_device->dev;

	*(unsigned int *)ldp_dai[0].cpu_dai->private_data = 1; /* McBSP2 */
#ifdef CONFIG_SND_SOC_TWL4030_FM
	*(unsigned int *)ldp_dai[1].cpu_dai->private_data = 3; /* McBSP4 */
#endif

	ret = platform_device_add(ldp_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(ldp_snd_device);

	return ret;
}
module_init(ldp_soc_init);

static void __exit ldp_soc_exit(void)
{
	platform_device_unregister(ldp_snd_device);
}
module_exit(ldp_soc_exit);

MODULE_AUTHOR("Misael Lopez Cruz <x0052729@ti.com>");
MODULE_DESCRIPTION("ALSA SoC LDP");
MODULE_LICENSE("GPL");

