/*
 * archos.c  --  SoC audio for Archos Board
 *
 * Author: Jean-Christophe Rona <rona@archos.com>
 * 
 *    Based on omap3beagle.c
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/switch.h>

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
#include "../codecs/wm8985.h"
#include "../codecs/wl1271.h"

#include <mach/archos-audio.h>
#include <mach/clock.h>

/* Define this if you want to enable DAPM (Dynamic Power Management) */
//#define DYNAMIC_POWER

static struct audio_device_config *pt_audio_device_io = NULL;

static int mclk;

static int archos_spk_func = 0;
static int archos_spdif_func = 0;
static int archos_hp_func = 0;
static int archos_out4_func = 0;
static int archos_input_func = 0;

static void _enable_spdif_ampli(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_spdif)
		pt_audio_device_io->set_spdif(PLAT_ON);
#endif
}

static void _disable_spdif_ampli(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_spdif)
		pt_audio_device_io->set_spdif(PLAT_OFF);
#endif
}

static void _enable_master_clock(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_codec_master_clk_state)
		pt_audio_device_io->set_codec_master_clk_state(PLAT_ON);
#endif
}

static void _disable_master_clock(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_codec_master_clk_state)
		pt_audio_device_io->set_codec_master_clk_state(PLAT_OFF);
#endif
}

static inline void _enable_speaker_ampli(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_speaker_state)
		pt_audio_device_io->set_speaker_state(PLAT_ON);
#endif
}

static inline void _disable_speaker_ampli(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_speaker_state)
		pt_audio_device_io->set_speaker_state(PLAT_OFF);
#endif
}

static int _get_headphone_state(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->get_headphone_plugged)
		return pt_audio_device_io->get_headphone_plugged();
#endif
	return 0;
}

static int _get_headphone_irq(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->get_headphone_irq)
		return pt_audio_device_io->get_headphone_irq();
#endif
	return 0;
}

struct sample_info_ {
        u32 sample_rate;
	int sr;
	int mclk_div;
	int bclk_div;
	int out_hz;
};

static struct sample_info_ analog_sample_info[] = {
	/* DAI bit and frame clocks needs to be as low as possible to save power */
        /*  SR,   sr, mclk, bclk,       sysclk  */
        {44100,    0,    2,    1,     11289600},
        {22050,    2,    4,    1,     11289600},
        {11025,    4,    6,    1,     11289600},

        {32000,    1,    3,    1,     12288000},
        {16000,    3,    5,    1,     12288000},
        { 8000,    5,    7,    1,     12288000},

        {48000,    0,    2,    1,     12288000},
        {24000,    2,    4,    1,     12288000},
        {12000,    4,    6,    1,     12288000},
	/* TODO: add more entries (especially for Slave mode)*/
};

int analog_sample_count = ARRAY_SIZE(analog_sample_info);

static struct sample_info_ spdif_sample_info[] = {
	/* DAI bit and frame clocks needs to be as low as possible to save power */
        /*  SR,   sr, mclk, bclk,       sysclk  */
        {48000,    0,    2,    1,     12288000},
        {44100,    0,    2,    1,     11289600},
        {32000,    1,    2,    1,      8192000},
 	/* TODO: add more entries (especially for Slave mode)*/
};

int spdif_sample_count = ARRAY_SIZE(spdif_sample_info);

static struct sample_info_ *sample_info;
int sample_count;

/* Headphone plug detection stuff ------------------------------------------ */

#define HEAPHONE_SW_NAME	"headphone_switch"

struct work_struct headphone_work;

static ssize_t print_headphone_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", HEAPHONE_SW_NAME);
}

static ssize_t print_headphone_state(struct switch_dev *sdev, char *buf)
{
	ssize_t buflen=0;

	switch(sdev->state)
	{
		case 0:
			buflen = sprintf(buf, "Not plugged\n");
			break;
		case 1:
			buflen = sprintf(buf, "Plugged\n");
			break;
		default:
			buflen = sprintf(buf, "Unknown\n");
			break;
	}
	return buflen;
}

static struct switch_dev headphone_sw_dev = {
	.name = HEAPHONE_SW_NAME,
	.print_name  = print_headphone_name,
	.print_state = print_headphone_state,
};

static irqreturn_t headphone_plug_isr(int irq, void *dev_id)
{
	schedule_work(&headphone_work);

	return IRQ_HANDLED;
}

static void headphone_plug_work_func(struct work_struct *work)
{
	int headphone_current_state;

	headphone_current_state = _get_headphone_state();
	//printk("HP IRQ Handler : %d\n", headphone_current_state);
	switch_set_state(&headphone_sw_dev, headphone_current_state);
}

static int headphone_plug_register(void)
{
	int irq, err;
	int headphone_current_state;

	irq = _get_headphone_irq();
	err = request_irq(irq, headphone_plug_isr, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
				"audio-headphone-int", &pt_audio_device_io);
	if(err < 0)
		return err;

	INIT_WORK(&headphone_work, headphone_plug_work_func);

	err = switch_dev_register(&headphone_sw_dev);
	if(err < 0)
		return err;

	headphone_current_state = _get_headphone_state();
	switch_set_state(&headphone_sw_dev, headphone_current_state);

	return 0;
}

static int headphone_plug_unregister(void)
{
	int irq;

	irq = _get_headphone_irq();
	free_irq(irq, &pt_audio_device_io);
	switch_dev_unregister(&headphone_sw_dev);

	return 0;
}

/* ------------------------------------------------------------------------- */

#ifdef DYNAMIC_POWER
/* Blocs will be powered dynamically, input routes could be set
	using WM8589 DAPM controls but we will activate them here to to be sure */
static void archos_ext_control(struct snd_soc_codec *codec)
{
	/* First Input stuff */
	switch (archos_input_func) {
		case 0:		/* None */
			wm8985_connect_pga(codec, OFF);
			wm8985_select_pga_inputs(codec, INPUT_LIP|INPUT_RIP|INPUT_LINE|INPUT_LIN|INPUT_RIN, OFF);
			snd_soc_dapm_disable_pin(codec, "Mic");
			snd_soc_dapm_disable_pin(codec, "Jack Mic");
			snd_soc_dapm_disable_pin(codec, "Line In");
			break;
		case 1:		/* Mic */
			wm8985_select_pga_inputs(codec, INPUT_RIP|INPUT_LINE, OFF);
			wm8985_select_pga_inputs(codec, INPUT_LIN|INPUT_RIN|INPUT_LIP, ON);
			wm8985_connect_pga(codec, ON);
			snd_soc_dapm_enable_pin(codec, "Mic");
			snd_soc_dapm_disable_pin(codec, "Jack Mic");
			snd_soc_dapm_disable_pin(codec, "Line In");
			break;
		case 2:		/* Jack Mic */
			wm8985_select_pga_inputs(codec, INPUT_LIP|INPUT_LINE, OFF);
			wm8985_select_pga_inputs(codec, INPUT_LIN|INPUT_RIN|INPUT_RIP, ON);
			wm8985_connect_pga(codec, ON);
			snd_soc_dapm_enable_pin(codec, "Jack Mic");
			snd_soc_dapm_disable_pin(codec, "Mic");
			snd_soc_dapm_disable_pin(codec, "Line In");
			break;
		case 3:		/* Line */
			wm8985_select_pga_inputs(codec, INPUT_LIP|INPUT_RIP, OFF);
			wm8985_select_pga_inputs(codec, INPUT_LIN|INPUT_RIN|INPUT_LINE, ON);
			wm8985_connect_pga(codec, ON);
			snd_soc_dapm_enable_pin(codec, "Line In");
			snd_soc_dapm_disable_pin(codec, "Mic");
			snd_soc_dapm_disable_pin(codec, "Jack Mic");
			break;
		default:
			break;
	}

	/* Then SPDIF */
	if (archos_spdif_func) {
		/* Use SPDIF sample rates */
		snd_soc_dapm_disable_pin(codec, "Speaker");
		snd_soc_dapm_disable_pin(codec, "Headphone");
		sample_info = spdif_sample_info;
		sample_count = spdif_sample_count;
		_enable_spdif_ampli();

		/* Nothing else has to be enabled if SPDIF is active */
		snd_soc_dapm_sync(codec);
		return;
	} else {
		/* Use analogic sample rates */
		sample_info = analog_sample_info;
		sample_count = analog_sample_count;
		_disable_spdif_ampli();
	}

	/* Finally, if SPDIF is OFF, Speaker, Headphone and Out4*/
	if (archos_spk_func)
		snd_soc_dapm_enable_pin(codec, "Speaker");
	else
		snd_soc_dapm_disable_pin(codec, "Speaker");

	if (archos_hp_func)
		snd_soc_dapm_enable_pin(codec, "Headphone");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone");

	if (archos_out4_func)
		snd_soc_dapm_enable_pin(codec, "HSDPA Mic");
	else
		snd_soc_dapm_disable_pin(codec, "HSDPA Mic");

	snd_soc_dapm_sync(codec);
}
#else /* DYNAMIC_POWER */
/* Blocs have to be powered manually, intput routes have to be set too */
static void archos_ext_control(struct snd_soc_codec *codec)
{
	/* First Input stuff */
	switch (archos_input_func) {
		case 0:		/* None */
			wm8985_power_up_inputs(codec, INP_BLOC_BOOST|INP_BLOC_PGA|INP_BLOC_MIC, OFF);
			wm8985_connect_pga(codec, OFF);
			wm8985_select_pga_inputs(codec, INPUT_LIP|INPUT_RIP|INPUT_LINE|INPUT_LIN|INPUT_RIN, OFF);
			break;
		case 1:		/* Mic */
			wm8985_select_pga_inputs(codec, INPUT_RIP|INPUT_LINE, OFF);
			wm8985_select_pga_inputs(codec, INPUT_LIN|INPUT_RIN|INPUT_LIP, ON);
			wm8985_connect_pga(codec, ON);
			wm8985_power_up_inputs(codec, INP_BLOC_BOOST|INP_BLOC_PGA|INP_BLOC_MIC, ON);
			break;
		case 2:		/* Jack Mic */
			wm8985_select_pga_inputs(codec, INPUT_LIP|INPUT_LINE, OFF);
			wm8985_select_pga_inputs(codec, INPUT_LIN|INPUT_RIN|INPUT_RIP, ON);
			wm8985_connect_pga(codec, ON);
			wm8985_power_up_inputs(codec, INP_BLOC_BOOST|INP_BLOC_PGA|INP_BLOC_MIC, ON);
			break;
		case 3:		/* Line */
			wm8985_select_pga_inputs(codec, INPUT_LIP|INPUT_RIP, OFF);
			wm8985_select_pga_inputs(codec, INPUT_LIN|INPUT_RIN|INPUT_LINE, ON);
			wm8985_connect_pga(codec, ON);
			wm8985_power_up_inputs(codec, INP_BLOC_MIC, OFF);
			wm8985_power_up_inputs(codec, INP_BLOC_BOOST|INP_BLOC_PGA, ON);
			break;
		default:
			break;
	}

	/* Then SPDIF */
	if (archos_spdif_func) {
		/* Use SPDIF sample rates */
		_disable_speaker_ampli();
		wm8985_select_outputs(codec, OUTPUT_SPK|OUTPUT_HP, OFF);
		sample_info = spdif_sample_info;
		sample_count = spdif_sample_count;
		_enable_spdif_ampli();

		/* Nothing else has to be enabled if SPDIF is active */
		return;
	} else {
		/* Use analogic sample rates */
		sample_info = analog_sample_info;
		sample_count = analog_sample_count;
		_disable_spdif_ampli();
	}

	/* Finally, if SPDIF is OFF, Speaker, Headphone and Out4*/
	if (archos_spk_func) {
		wm8985_select_outputs(codec, OUTPUT_SPK, ON);
		_enable_speaker_ampli();
	} else {
		_disable_speaker_ampli();
		wm8985_select_outputs(codec, OUTPUT_SPK, OFF);
	}

	if (archos_hp_func)
		wm8985_select_outputs(codec, OUTPUT_HP, ON);
	else
		wm8985_select_outputs(codec, OUTPUT_HP, OFF);

	if (archos_out4_func) {
		/* Out3 mixer is needed to route LIP (Internal MIC) to Out4 */
		wm8985_power_up_outn_mixers(codec, MIXER_OUT4 | MIXER_OUT3, ON);
		wm8985_select_outputs(codec, OUTPUT_OUT4, ON);
	} else {
		wm8985_select_outputs(codec, OUTPUT_OUT4, OFF);
		wm8985_power_up_outn_mixers(codec, MIXER_OUT4 | MIXER_OUT3, OFF);
	}
}
#endif /* DYNAMIC_POWER */

/* Archos board <--> WM8985 ops -------------------------------------------- */

static int archos_wm8985_startup(struct snd_pcm_substream *substream)
{
//	struct snd_pcm_runtime *runtime = substream->runtime;
//	struct snd_soc_pcm_runtime *rtd = substream->private_data;
//	struct snd_soc_codec *codec = rtd->socdev->codec;


//	return clk_enable(sys_clkout2);
	return 0;
}

static void archos_wm8985_shutdown(struct snd_pcm_substream *substream)
{
//	clk_disable(sys_clkout2);
}

static int archos_wm8985_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret,i;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_DSP_A |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI fmt configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	if (archos_spdif_func) {
		/* SPDIF Enabled, we need to configure MCBSP as a SPDIF output */
		ret = snd_soc_dai_set_fmt(cpu_dai,
					  SND_SOC_DAIFMT_SPDIF |
					  SND_SOC_DAIFMT_NB_IF |
					  SND_SOC_DAIFMT_CBM_CFM);
	} else {
		/* SPDIF Disable, we need to configure MCBSP as a PCM output */
		ret = snd_soc_dai_set_fmt(cpu_dai,
					  SND_SOC_DAIFMT_DSP_A |
					  SND_SOC_DAIFMT_NB_IF |
					  SND_SOC_DAIFMT_CBM_CFM);
	}

	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set PPL and divisor here and not in the codec,
		since it's our board which requires these settings */
	//printk(KERN_INFO "archos_wm8985_hw_params : freq = %d, channels = %d\n", params_rate(params), params_channels(params));
	for(i = 0; i < sample_count; i++) {
		if (params_rate(params) == sample_info[i].sample_rate) {
			//printk(KERN_INFO "archos_hw_params : selected id = %d\n", i);

			/* PPL (WM8985 is Master) */
			ret = snd_soc_dai_set_pll(codec_dai, 0, mclk,sample_info[i].out_hz);
			if (ret < 0) {
				printk(KERN_ERR "can't set codec DAI ppl configuration\n");
				return ret;
			}

			/* MCLK/BCLK stuff */
			ret = snd_soc_dai_set_clkdiv(codec_dai,WM8985_MCLKSEL,
				WM8985_MCLK_PLL);
			if (ret < 0) {
				printk(KERN_ERR "can't set codec DAI MCLKSEL configuration\n");
				return ret;
			}
			ret = snd_soc_dai_set_clkdiv(codec_dai,WM8985_MCLKDIV,
				sample_info[i].mclk_div << 5);
			if (ret < 0) {
				printk(KERN_ERR "can't set codec DAI MCLKDIV configuration\n");
				return ret;
			}
			ret = snd_soc_dai_set_clkdiv(codec_dai,WM8985_BCLKDIV,
				sample_info[i].bclk_div << 2);
			if (ret < 0) {
				printk(KERN_ERR "can't set codec DAI BCLKDIV configuration\n");
				return ret;
			}
		}
	}


	/* The codec may need to know the MCLK (Sample Rate for DAC and ADC are set by the codec) */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8985_ID_MCLK, mclk,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops archos_wm8985_ops = {
	.startup = archos_wm8985_startup,
	.hw_params = archos_wm8985_hw_params,
	.shutdown = archos_wm8985_shutdown,
};

/* ------------------------------------------------------------------------- */

/* Archos board <--> FM/BT ops --------------------------------------------- */

static int archos_wl1271_startup(struct snd_pcm_substream *substream)
{
//	struct snd_pcm_runtime *runtime = substream->runtime;
//	struct snd_soc_pcm_runtime *rtd = substream->private_data;
//	struct snd_soc_codec *codec = rtd->socdev->codec;

	return 0;
}

static void archos_wl1271_shutdown(struct snd_pcm_substream *substream)
{
}

static int archos_wl1271_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;

	//printk(KERN_INFO "archos_wl1271_hw_params : freq = %d, channels = %d\n", params_rate(params), params_channels(params));
	
	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI fmt configuration\n");
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
#if 0	/* Needed if the OMAP is Master */
	/* Set MCBSP sysclock to functional (96 Mhz) */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK, 0,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err(KERN_ERR "can't set cpu system clock\n");
		return ret;
	}

	/* Set Clock Divisor to 40 to output 2,4 Mhz */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_MCBSP_CLKGDV, 40);
	if (ret < 0) {
		pr_err(KERN_ERR "can't set SRG clock divider\n");
		return ret;
	}
#endif
	return 0;
}

static struct snd_soc_ops archos_wl1271_ops = {
	.startup = archos_wl1271_startup,
	.hw_params = archos_wl1271_hw_params,
	.shutdown = archos_wl1271_shutdown,
};

/* ------------------------------------------------------------------------- */

#if 0
/* Fake codec device allowing to handle more than one codec ---------------- */

struct soc_multiple_codec_dev {
	int num_codecs;
	struct snd_soc_codec_device **codecs;
};

static struct snd_soc_codec_device *archos_codec_list[] = {
	&soc_codec_dev_wm8985,
	&soc_codec_dev_wl1271,
};

static struct soc_multiple_codec_dev archos_codecs = {
	.codecs = archos_codec_list,
	.num_codecs = ARRAY_SIZE(archos_codec_list),
};

static int multiple_probe(struct platform_device *pdev)
{
	int i, ret;

	for (i = 0; i < archos_codecs.num_codecs; i++) {
		if (archos_codecs.codecs[i]->probe) {
			ret = archos_codecs.codecs[i]->probe(pdev);
			if (ret) return ret;
		}
	}

	return 0;
}

static int multiple_remove(struct platform_device *pdev)
{
	int i, ret;

	for (i = 0; i < archos_codecs.num_codecs; i++) {
		if (archos_codecs.codecs[i]->remove) {
			ret = archos_codecs.codecs[i]->remove(pdev);
			if (ret) return ret;
		}
	}

	return 0;
}

static int multiple_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i, ret;

	for (i = 0; i < archos_codecs.num_codecs; i++) {
		if (archos_codecs.codecs[i]->suspend) {
			ret = archos_codecs.codecs[i]->suspend(pdev, state);
			if (ret) return ret;
		}
	}

	return 0;
}

static int multiple_resume(struct platform_device *pdev)
{
	int i, ret;

	for (i = 0; i < archos_codecs.num_codecs; i++) {
		if (archos_codecs.codecs[i]->resume) {
			ret = archos_codecs.codecs[i]->resume(pdev);
			if (ret) return ret;
		}
	}

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_multiple = {
	.probe = 	multiple_probe,
	.remove = 	multiple_remove,
	.suspend = 	multiple_suspend,
	.resume =	multiple_resume,
};

/* ------------------------------------------------------------------------- */
#endif

static int archos_get_spk(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = archos_spk_func;

	return 0;
}

static int archos_set_spk(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (archos_spk_func == ucontrol->value.integer.value[0])
		return 0;

	archos_spk_func = ucontrol->value.integer.value[0];
	archos_ext_control(codec);

	return 1;
}

static int archos_get_spdif(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = archos_spdif_func;

	return 0;
}

static int archos_set_spdif(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (archos_spdif_func == ucontrol->value.integer.value[0])
		return 0;

	archos_spdif_func = ucontrol->value.integer.value[0];
	archos_ext_control(codec);

	return 1;
}

static int archos_get_hp(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = archos_hp_func;

	return 0;
}

static int archos_set_hp(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (archos_hp_func == ucontrol->value.integer.value[0])
		return 0;

	archos_hp_func = ucontrol->value.integer.value[0];
	archos_ext_control(codec);

	return 1;
}

static int archos_get_out4(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = archos_out4_func;

	return 0;
}

static int archos_set_out4(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (archos_out4_func == ucontrol->value.integer.value[0])
		return 0;

	archos_out4_func = ucontrol->value.integer.value[0];
	archos_ext_control(codec);

	return 1;
}

static int archos_get_input(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = archos_input_func;

	return 0;
}

static int archos_set_input(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (archos_input_func == ucontrol->value.integer.value[0])
		return 0;

	archos_input_func = ucontrol->value.integer.value[0];
	archos_ext_control(codec);

	return 1;
}

#ifdef DYNAMIC_POWER
static int speaker_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		_enable_speaker_ampli();
	} else if (SND_SOC_DAPM_EVENT_OFF(event)) {
		_disable_speaker_ampli();
	}

	return 0;
}

static const struct snd_soc_dapm_widget board_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_HP("HSDPA Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", speaker_event),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_INPUT("HSDPA Spk"),
	SND_SOC_DAPM_MIC("Jack Mic", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"Headphone", NULL, "LOUT1"},
	{"Headphone", NULL, "ROUT1"},

	{"Speaker", NULL, "LOUT2"},
	{"Speaker", NULL, "ROUT2"},

	{"HSDPA Mic", NULL, "OUT4"},

	{"LLINE", NULL, "Line In"},
	{"RLINE", NULL, "Line In"},

	{"LAUX", NULL, "HSDPA Spk"},
	{"RAUX", NULL, "HSDPA Spk"},

	{"RINP", NULL, "Jack Mic"},

	{"LINP", NULL, "Mic"},
};
#endif /* DYNAMIC_POWER */

static const char *spk_function[] = {"Off", "On"};
static const char *hp_function[] = {"Off", "On"};
static const char *out4_function[] = {"Off", "On"};
static const char *spdif_function[] = {"Off", "On"};
static const char *input_function[] = {"None", "Mic", "Jack Mic", "Line"};
static const struct soc_enum archos_controls_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spk_function), spk_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spdif_function), spdif_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(hp_function), hp_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(out4_function), out4_function),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(input_function), input_function),
};

static const struct snd_kcontrol_new board_controls[] = {
	SOC_ENUM_EXT("Speaker Function Playback Switch", archos_controls_enum[0],
		     archos_get_spk, archos_set_spk),
	SOC_ENUM_EXT("SPDIF Function Playback Switch", archos_controls_enum[1],
		     archos_get_spdif, archos_set_spdif),
	SOC_ENUM_EXT("Headphone Function Playback Switch", archos_controls_enum[2],
		     archos_get_hp, archos_set_hp),
	SOC_ENUM_EXT("Out4 Function Playback Switch", archos_controls_enum[3],
		     archos_get_out4, archos_set_out4),
	SOC_ENUM_EXT("Input Select",  archos_controls_enum[4],
		     archos_get_input, archos_set_input),
};

static int archos_wm8985_init(struct snd_soc_codec *codec)
{
	int i, err;

	/* Add archos G7 specific controls */
	for (i = 0; i < ARRAY_SIZE(board_controls); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&board_controls[i], codec, NULL));
		if (err < 0)
			return err;
	}

#ifdef DYNAMIC_POWER
	/* Not used (so it won't trigger any power up) */
	snd_soc_dapm_nc_pin(codec, "LINN");
	snd_soc_dapm_nc_pin(codec, "RINN");

	/* Add archos G7 specific widgets */
	snd_soc_dapm_new_controls(codec, board_dapm_widgets,
				  ARRAY_SIZE(board_dapm_widgets));

	/* Set up archos specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);
#endif /* DYNAMIC_POWER */

	/* Sync stuff with the external controls config */
	archos_ext_control(codec);

	return 0;
}

static int archos_wl1271_init(struct snd_soc_codec *codec)
{
	return 0;
}

#ifdef CONFIG_PM
int archos_wm8985_suspend_pre(struct platform_device *pdev, pm_message_t state)
{
	if (archos_spk_func) {
		msleep(100);
		_disable_speaker_ampli();
	}
	return 0;
}

int archos_wm8985_suspend_post(struct platform_device *pdev, pm_message_t state)
{
	_disable_master_clock();
	return 0;
}

int archos_wm8985_resume_pre(struct platform_device *pdev)
{
	_enable_master_clock();
	return 0;
}

int archos_wm8985_resume_post(struct platform_device *pdev)
{
	int headphone_current_state;

	if (archos_spk_func) {
		msleep(100);
		_enable_speaker_ampli();
	}

	headphone_current_state = _get_headphone_state();
	switch_set_state(&headphone_sw_dev, headphone_current_state);

	return 0;
}

int archos_wl1271_suspend_pre(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

int archos_wl1271_suspend_post(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

int archos_wl1271_resume_pre(struct platform_device *pdev)
{
	return 0;
}

int archos_wl1271_resume_post(struct platform_device *pdev)
{
	return 0;
}
#else

#define archos_wm8985_suspend_pre NULL
#define archos_wm8985_suspend_post NULL
#define archos_wm8985_resume_pre NULL
#define archos_wm8985_resume_post NULL
#define archos_wl1271_suspend_pre NULL
#define archos_wl1271_suspend_post NULL
#define archos_wl1271_resume_pre NULL
#define archos_wl1271_resume_post NULL

#endif

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link archos_dai[] = {
	{
	.name = "Wolfson",
	.stream_name = "WM8985",
	.cpu_dai = &omap_mcbsp_dai[0],
	.codec_dai = &wm8985_dai,
	.init = archos_wm8985_init,
	.ops = &archos_wm8985_ops,
	},{
	.name = "FM/BT",
	.stream_name = "WL1271",
	.cpu_dai = &omap_mcbsp_dai[1],
	.codec_dai = &wl1271_dai,
	.init = archos_wl1271_init,
	.ops = &archos_wl1271_ops,
	}
};

/* Audio machine driver (WM8985)*/
static struct snd_soc_card snd_soc_archos_wm8985 = {
	.name = "Wolfson",
	.platform = &omap_soc_platform,
	.dai_link = &archos_dai[0],
	.num_links = 1,
	.suspend_pre = &archos_wm8985_suspend_pre,
	.suspend_post = &archos_wm8985_suspend_post,
	.resume_pre = &archos_wm8985_resume_pre,
	.resume_post = &archos_wm8985_resume_post,
};

/* Audio machine driver (FM/BT)*/
static struct snd_soc_card snd_soc_archos_wl1271 = {
	.name = "FM/BT",
	.platform = &omap_soc_platform,
	.dai_link = &archos_dai[1],
	.num_links = 1,
	.suspend_pre = &archos_wl1271_suspend_pre,
	.suspend_post = &archos_wl1271_suspend_post,
	.resume_pre = &archos_wl1271_resume_pre,
	.resume_post = &archos_wl1271_resume_post,
};

/* Audio subsystem (WM8985)*/
static struct snd_soc_device archos_snd_devdata_wm8985 = {
	.card = &snd_soc_archos_wm8985,
	.codec_dev = &soc_codec_dev_wm8985,
};

/* Audio subsystem (FM/BT)*/
static struct snd_soc_device archos_snd_devdata_wl1271 = {
	.card = &snd_soc_archos_wl1271,
	.codec_dev = &soc_codec_dev_wl1271,
};

static struct platform_device *archos_snd_device_wm8985;
static struct platform_device *archos_snd_device_wl1271;

static int __init archos_soc_init(void)
{
	int ret;
	struct clk *clkout1;

	if (!machine_is_archos_g6h() && !machine_is_archos_g6s() &&
		!machine_is_archos_a5s() && !machine_is_archos_a5h() &&
		!machine_is_archos_a5sg() && !machine_is_archos_a5hg() &&
		!machine_is_archos_a5sgw() && !machine_is_archos_a5hgw() &&
		!machine_is_archos_a5gcam() && !machine_is_archos_a5sc() &&
		!machine_is_archos_a5st()) {
		pr_debug("Not an Archos Device!\n");
		return -ENODEV;
	}
	pr_info("Archos Device SoC init\n");

	/* Master clock has to be enable to feed the WM8985 */
	pt_audio_device_io = archos_audio_get_io();
	_enable_master_clock();

	clkout1 = clk_get(NULL, "sys_clkout1");
	if (IS_ERR(clkout1)) {
		printk(KERN_DEBUG "Archos ASoC: cannot get codec master clock\n");
		ret = -ENODEV;
		goto err1;
	}
	mclk = clk_get_rate(clkout1);
	clk_put(clkout1);

	printk(KERN_DEBUG "Archos ASoC: master clk is: %d\n", mclk);

	if(headphone_plug_register() < 0) {
		printk(KERN_ERR "archos_soc_init: Error registering Headphone device\n");	
		ret = -ENODEV;
		goto err2;
	}

	archos_snd_device_wm8985 = platform_device_alloc("soc-audio", -1);
	if (!archos_snd_device_wm8985) {
		printk(KERN_ERR "archos_soc_init: Platform device allocation failed for WM8985\n");
		ret = -ENOMEM;
		goto err3;
	}
	archos_snd_device_wl1271 = platform_device_alloc("soc-audio", -2);
	if (!archos_snd_device_wl1271) {
		printk(KERN_ERR "archos_soc_init: Platform device allocation failed for FM/BT\n");
		ret = -ENOMEM;
		goto err4;
	}

	platform_set_drvdata(archos_snd_device_wm8985, &archos_snd_devdata_wm8985);
	archos_snd_devdata_wm8985.dev = &archos_snd_device_wm8985->dev;
	platform_set_drvdata(archos_snd_device_wl1271, &archos_snd_devdata_wl1271);
	archos_snd_devdata_wl1271.dev = &archos_snd_device_wl1271->dev;
	*(unsigned int *)archos_dai[0].cpu_dai->private_data = 1; /* WM8985 <--> McBSP2 */
	*(unsigned int *)archos_dai[1].cpu_dai->private_data = 2; /* FM/BT <--> McBSP3 */

	ret = platform_device_add(archos_snd_device_wm8985);
	if (ret)
		goto err5;
	ret = platform_device_add(archos_snd_device_wl1271);
	if (ret)
		goto err6;

	msleep(2);

	return 0;

err6:
	platform_device_unregister(archos_snd_device_wm8985);
err5:
	platform_device_put(archos_snd_device_wl1271);
err4:
	platform_device_put(archos_snd_device_wm8985);
err3:
	headphone_plug_unregister();
err2:
err1:
	_disable_master_clock();

	return ret;
}

static void __exit archos_soc_exit(void)
{
#if defined (CONFIG_MACH_ARCHOS)
	_disable_master_clock();
#endif
	headphone_plug_unregister();
	platform_device_unregister(archos_snd_device_wm8985);
	platform_device_unregister(archos_snd_device_wl1271);
}

module_init(archos_soc_init);
module_exit(archos_soc_exit);

MODULE_AUTHOR("Jean-Christophe Rona <rona@archos.com>");
MODULE_DESCRIPTION("ALSA SoC Archos Device");
MODULE_LICENSE("GPL");
