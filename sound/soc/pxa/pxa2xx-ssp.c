/*
 * pxa2xx-ssp.c  --  ALSA Soc Audio Layer
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    12th Aug 2005   Initial version.
 *
 * TODO:
 *  o Test network mode for > 16bit sample size
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/regs-ssp.h>
#include <asm/arch/audio.h>
#include <asm/arch/ssp.h>

#include "pxa2xx-pcm.h"
#include "pxa2xx-ssp.h"

/*
 * SSP audio private data
 */
struct ssp_priv {
	unsigned int sysclk;
};

static struct ssp_priv ssp_clk[3];
static struct ssp_dev ssp[3];
#ifdef CONFIG_PM
static struct ssp_state ssp_state[3];
#endif

#define PXA2xx_SSP1_BASE	0x41000000
#define PXA27x_SSP2_BASE	0x41700000
#define PXA27x_SSP3_BASE	0x41900000

static struct pxa2xx_pcm_dma_params pxa2xx_ssp1_pcm_mono_out = {
	.name			= "SSP1 PCM Mono out",
	.dev_addr		= PXA2xx_SSP1_BASE + SSDR,
	.drcmr			= &DRCMRTXSSDR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ssp1_pcm_mono_in = {
	.name			= "SSP1 PCM Mono in",
	.dev_addr		= PXA2xx_SSP1_BASE + SSDR,
	.drcmr			= &DRCMRRXSSDR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ssp1_pcm_stereo_out = {
	.name			= "SSP1 PCM Stereo out",
	.dev_addr		= PXA2xx_SSP1_BASE + SSDR,
	.drcmr			= &DRCMRTXSSDR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH4,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ssp1_pcm_stereo_in = {
	.name			= "SSP1 PCM Stereo in",
	.dev_addr		= PXA2xx_SSP1_BASE + SSDR,
	.drcmr			= &DRCMRRXSSDR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH4,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ssp2_pcm_mono_out = {
	.name			= "SSP2 PCM Mono out",
	.dev_addr		= PXA27x_SSP2_BASE + SSDR,
	.drcmr			= &DRCMRTXSS2DR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ssp2_pcm_mono_in = {
	.name			= "SSP2 PCM Mono in",
	.dev_addr		= PXA27x_SSP2_BASE + SSDR,
	.drcmr			= &DRCMRRXSS2DR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ssp2_pcm_stereo_out = {
	.name			= "SSP2 PCM Stereo out",
	.dev_addr		= PXA27x_SSP2_BASE + SSDR,
	.drcmr			= &DRCMRTXSS2DR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH4,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ssp2_pcm_stereo_in = {
	.name			= "SSP2 PCM Stereo in",
	.dev_addr		= PXA27x_SSP2_BASE + SSDR,
	.drcmr			= &DRCMRRXSS2DR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH4,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ssp3_pcm_mono_out = {
	.name			= "SSP3 PCM Mono out",
	.dev_addr		= PXA27x_SSP3_BASE + SSDR,
	.drcmr			= &DRCMRTXSS3DR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ssp3_pcm_mono_in = {
	.name			= "SSP3 PCM Mono in",
	.dev_addr		= PXA27x_SSP3_BASE + SSDR,
	.drcmr			= &DRCMRRXSS3DR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ssp3_pcm_stereo_out = {
	.name			= "SSP3 PCM Stereo out",
	.dev_addr		= PXA27x_SSP3_BASE + SSDR,
	.drcmr			= &DRCMRTXSS3DR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH4,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ssp3_pcm_stereo_in = {
	.name			= "SSP3 PCM Stereo in",
	.dev_addr		= PXA27x_SSP3_BASE + SSDR,
	.drcmr			= &DRCMRRXSS3DR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH4,
};

static struct pxa2xx_pcm_dma_params *ssp_dma_params[3][4] = {
	{&pxa2xx_ssp1_pcm_mono_out, &pxa2xx_ssp1_pcm_mono_in,
	&pxa2xx_ssp1_pcm_stereo_out, &pxa2xx_ssp1_pcm_stereo_in,},
	{&pxa2xx_ssp2_pcm_mono_out, &pxa2xx_ssp2_pcm_mono_in,
	&pxa2xx_ssp2_pcm_stereo_out, &pxa2xx_ssp2_pcm_stereo_in,},
	{&pxa2xx_ssp3_pcm_mono_out, &pxa2xx_ssp3_pcm_mono_in,
	&pxa2xx_ssp3_pcm_stereo_out, &pxa2xx_ssp3_pcm_stereo_in,},
};

static int pxa2xx_ssp_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;

	if (!rtd->dai->cpu_dai->active) {
		ret = ssp_init(&ssp[cpu_dai->id], cpu_dai->id + 1,
			SSP_NO_IRQ);
		if (ret < 0)
			return ret;
		ssp_disable(&ssp[cpu_dai->id]);
	}
	return ret;
}

static void pxa2xx_ssp_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;

	if (!cpu_dai->active) {
		ssp_disable(&ssp[cpu_dai->id]);
		ssp_exit(&ssp[cpu_dai->id]);
	}
}

#ifdef CONFIG_PM

static int pxa2xx_ssp_suspend(struct platform_device *pdev,
	struct snd_soc_cpu_dai *dai)
{
	if (!dai->active)
		return 0;

	ssp_save_state(&ssp[dai->id], &ssp_state[dai->id]);
	clk_disable(ssp[dai->id].ssp->clk);
	return 0;
}

static int pxa2xx_ssp_resume(struct platform_device *pdev,
	struct snd_soc_cpu_dai *dai)
{
	if (!dai->active)
		return 0;

	clk_enable(ssp[dai->id].ssp->clk);
	ssp_restore_state(&ssp[dai->id], &ssp_state[dai->id]);
	ssp_enable(&ssp[dai->id]);

	return 0;
}

#else
#define pxa2xx_ssp_suspend	NULL
#define pxa2xx_ssp_resume	NULL
#endif

/**
 * ssp_set_clkdiv - set SSP clock divider
 * @div: serial clock rate divider
 */
void ssp_set_scr(struct ssp_dev *dev, u32 div)
{
	struct ssp_device *ssp = dev->ssp;
	u32 sscr0 = __raw_readl(ssp->mmio_base + SSCR0) & ~SSCR0_SCR;

	__raw_writel(sscr0 | SSCR0_SerClkDiv(div), ssp->mmio_base + SSCR0);
}

/*
 * Set the SSP ports SYSCLK.
 */
static int pxa2xx_ssp_set_dai_sysclk(struct snd_soc_cpu_dai *cpu_dai,
	int clk_id, unsigned int freq, int dir)
{
	void __iomem *mmio_base = ssp[cpu_dai->id].ssp->mmio_base;
	int val;

	u32 sscr0 = __raw_readl(mmio_base + SSCR0) &
		~(SSCR0_ECS |  SSCR0_NCS | SSCR0_MOD | SSCR0_ADC);

	pr_debug("pxa2xx_ssp_set_dai_sysclk id: %d, clk_id %d, freq %d",
		cpu_dai->id, clk_id, freq);

	switch (clk_id) {
	case PXA2XX_SSP_CLK_NET_PLL:
		sscr0 |= SSCR0_MOD;
	case PXA2XX_SSP_CLK_PLL:
		/* Internal PLL is fixed on pxa25x and pxa27x */
		if (cpu_is_pxa25x())
			ssp_clk[cpu_dai->id].sysclk = 1843200;
		else
			ssp_clk[cpu_dai->id].sysclk = 13000000;
		break;
	case PXA2XX_SSP_CLK_EXT:
		ssp_clk[cpu_dai->id].sysclk = freq;
		sscr0 |= SSCR0_ECS;
		break;
	case PXA2XX_SSP_CLK_NET:
		ssp_clk[cpu_dai->id].sysclk = freq;
		sscr0 |= SSCR0_NCS | SSCR0_MOD;
		break;
	case PXA2XX_SSP_CLK_AUDIO:
		ssp_clk[cpu_dai->id].sysclk = 0;
		ssp_set_scr(&ssp[cpu_dai->id], 1);
		sscr0 |= SSCR0_ADC;
		break;
	default:
		return -ENODEV;
	}

	/* the SSP CKEN clock must be disabled when changing SSP clock mode */
	clk_disable(ssp[cpu_dai->id].ssp->clk);
	val = __raw_readl(mmio_base + SSCR0) | sscr0;
	__raw_writel(val, mmio_base + SSCR0);
	clk_enable(ssp[cpu_dai->id].ssp->clk);
	return 0;
}

/*
 * Set the SSP clock dividers.
 */
static int pxa2xx_ssp_set_dai_clkdiv(struct snd_soc_cpu_dai *cpu_dai,
	int div_id, int div)
{
	void __iomem *mmio_base = ssp[cpu_dai->id].ssp->mmio_base;
	int val;

	switch (div_id) {
	case PXA2XX_SSP_AUDIO_DIV_ACDS:
		val = (__raw_readl(mmio_base + SSACD) & ~0x7) | SSACD_ACDS(div);
		__raw_writel(val, mmio_base + SSACD);
		break;
	case PXA2XX_SSP_AUDIO_DIV_SCDB:
		val = __raw_readl(mmio_base + SSACD) & ~0x8;
		if (div == PXA2XX_SSP_CLK_SCDB_1)
			val |= SSACD_SCDB;
		__raw_writel(val, mmio_base + SSACD);
		break;
	case PXA2XX_SSP_DIV_SCR:
		ssp_set_scr(&ssp[cpu_dai->id], div);
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

/*
 * Configure the PLL frequency pxa27x and (afaik - pxa320 only)
 */
static int pxa2xx_ssp_set_dai_pll(struct snd_soc_cpu_dai *cpu_dai,
	int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	void __iomem *mmio_base = ssp[cpu_dai->id].ssp->mmio_base;
	u32 ssacd;

	ssacd = __raw_readl(mmio_base + SSACD) & ~0x70;
	switch (freq_out) {
	case 5622000:
		break;
	case 11345000:
		ssacd |= (0x1 << 4);
		break;
	case 12235000:
		ssacd |= (0x2 << 4);
		break;
	case 14857000:
		ssacd |= (0x3 << 4);
		break;
	case 32842000:
		ssacd |= (0x4 << 4);
		break;
	case 48000000:
		ssacd |= (0x5 << 4);
		break;
	}
	__raw_writel(ssacd, mmio_base + SSACD);
	return 0;
}

/*
 * Set the active slots in TDM/Network mode
 */
static int pxa2xx_ssp_set_dai_tdm_slot(struct snd_soc_cpu_dai *cpu_dai,
	unsigned int mask, int slots)
{
	void __iomem *mmio_base = ssp[cpu_dai->id].ssp->mmio_base;
	u32 sscr0;

	sscr0 = __raw_readl(mmio_base + SSCR0) & ~SSCR0_SlotsPerFrm(7);

	/* set number of active slots */
	sscr0 |= SSCR0_SlotsPerFrm(slots);
	__raw_writel(sscr0, mmio_base + SSCR0);

	/* set active slot mask */
	__raw_writel(mask, mmio_base + SSTSA);
	__raw_writel(mask, mmio_base + SSRSA);
	return 0;
}

/*
 * Tristate the SSP DAI lines
 */
static int pxa2xx_ssp_set_dai_tristate(struct snd_soc_cpu_dai *cpu_dai,
	int tristate)
{
	void __iomem *mmio_base = ssp[cpu_dai->id].ssp->mmio_base;
	u32 sscr1;

	sscr1 = __raw_readl(mmio_base + SSCR1);
	if (tristate)
		sscr1 &= ~SSCR1_TTE;
	else
		sscr1 |= SSCR1_TTE;
	__raw_writel(sscr1, mmio_base + SSCR1);

	return 0;
}

/*
 * Set up the SSP DAI format.
 * The SSP Port must be inactive before calling this function as the
 * physical interface format is changed.
 */
static int pxa2xx_ssp_set_dai_fmt(struct snd_soc_cpu_dai *cpu_dai,
		unsigned int fmt)
{
	void __iomem *mmio_base = ssp[cpu_dai->id].ssp->mmio_base;
	int val;

	/* reset port settings */
	__raw_writel(0, mmio_base + SSCR0);
	__raw_writel(0, mmio_base + SSCR1);
	__raw_writel(0, mmio_base + SSPSP);

	/* NOTE: I2S emulation is still very much work in progress here */

	/* FIXME: this is what wince uses for msb */
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_MSB) {
		__raw_writel(SSCR0_EDSS | SSCR0_TISSP | SSCR0_DataSize(16),
				mmio_base + SSCR0);
		goto master;
	}

	/* check for I2S emulation mode - handle it separately  */
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_I2S) {
		/* 8.4.11 */

		/* Only SSCR0[NCS] or SSCR0[ECS] bit fields settings are optional */
		__raw_writel(SSCR0_EDSS | SSCR0_PSP | SSCR0_DataSize(16),
				mmio_base + SSCR0);

		/* set FIFO thresholds */
		__raw_writel(SSCR1_RxTresh(14) | SSCR1_TxTresh(1),
				mmio_base + SSCR1);

		/* normal: */
		/* all bit fields must be cleared except: FSRT = 1 and
		 * SFRMWDTH = 16, DMYSTART=0,1) */
		__raw_writel(SSPSP_FSRT | SSPSP_SFRMWDTH(16) | SSPSP_DMYSTRT(0),
				mmio_base + SSPSP);
		goto master;
	}

	val = __raw_readl(mmio_base + SSCR0) | SSCR0_PSP;
	__raw_writel(val, mmio_base + SSCR0);
	__raw_writel(SSCR1_RxTresh(14) | SSCR1_TxTresh(1) |
			SSCR1_TRAIL | SSCR1_RWOT,
			mmio_base + SSCR1);

master:
	val = __raw_readl(mmio_base + SSCR1);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		val |= (SSCR1_SCLKDIR | SSCR1_SFRMDIR);
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		val |= SSCR1_SCLKDIR;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		val |= SSCR1_SFRMDIR;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}
	__raw_writel(val, mmio_base + SSCR1);

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		val = __raw_readl(mmio_base + SSPSP) | SSPSP_SFRMP | SSPSP_FSRT;
		__raw_writel(val, mmio_base + SSPSP);
		break;
	case SND_SOC_DAIFMT_IB_IF:
		break;
	default:
		return -EINVAL;
	}

	val = __raw_readl(mmio_base + SSPSP);
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		val |= SSPSP_DMYSTRT(1);
	case SND_SOC_DAIFMT_DSP_B:
		val |= SSPSP_SCMODE(2);
		break;
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_MSB:
		/* handled above */
		break;
	default:
		return -EINVAL;
	}
	__raw_writel(val, mmio_base + SSPSP);

	return 0;
}

/*
 * Set the SSP audio DMA parameters and sample size.
 * Can be called multiple times by oss emulation.
 */
static int pxa2xx_ssp_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	int dma = 0, chn = params_channels(params);
	void __iomem *mmio_base = ssp[cpu_dai->id].ssp->mmio_base;
	u32 sscr0;

	/* select correct DMA params */
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		dma = 1; /* capture DMA offset is 1,3 */
	if (chn == 2)
		dma += 2; /* stereo DMA offset is 2, mono is 0 */
	cpu_dai->dma_data = ssp_dma_params[cpu_dai->id][dma];

	pr_debug("pxa2xx_ssp_hw_params: dma %d", dma);

	/* we can only change the settings if the port is not in use */
	if (__raw_readl(mmio_base + SSCR0) & SSCR0_SSE)
		return 0;

	/* clear selected SSP bits */
	sscr0 = __raw_readl(mmio_base + SSCR0) & ~(SSCR0_DSS | SSCR0_EDSS);
	__raw_writel(sscr0, mmio_base + SSCR0);

	/* bit size */
	sscr0 = __raw_readl(mmio_base + SSCR0);
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sscr0 |= SSCR0_DataSize(16);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		sscr0 |= (SSCR0_EDSS | SSCR0_DataSize(8));
		/* we must be in network mode (2 slots) for 24 bit stereo */
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sscr0 |= (SSCR0_EDSS | SSCR0_DataSize(16));
		/* we must be in network mode (2 slots) for 32 bit stereo */
		break;
	}
	__raw_writel(sscr0, mmio_base + SSCR0);

	pr_debug("SSCR0 0x%08x SSCR1 0x%08x SSTO 0x%08x SSPSP 0x%08x SSSR 0x%08x SSACD 0x%08x",
		__raw_readl(mmio_base + SSCR0), __raw_readl(mmio_base + SSCR1),
		__raw_readl(mmio_base + SSTO), __raw_readl(mmio_base + SSPSP),
		__raw_readl(mmio_base + SSSR), __raw_readl(mmio_base + SSACD));

	return 0;
}

static int pxa2xx_ssp_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;
	void __iomem *mmio_base = ssp[cpu_dai->id].ssp->mmio_base;
	int val;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
		ssp_enable(&ssp[cpu_dai->id]);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		val = __raw_readl(mmio_base + SSCR1);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val |= SSCR1_TSRE;
		else
			val |= SSCR1_RSRE;
		__raw_writel(val, mmio_base + SSCR1);
		/* SSSR_P(port) |= SSSR_P(port); */
		val = __raw_readl(mmio_base + SSSR);
		__raw_writel(val, mmio_base + SSSR);
		break;
	case SNDRV_PCM_TRIGGER_START:
		val = __raw_readl(mmio_base + SSCR1);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val |= SSCR1_TSRE;
		else
			val |= SSCR1_RSRE;
		__raw_writel(val, mmio_base + SSCR1);
		ssp_enable(&ssp[cpu_dai->id]);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		val = __raw_readl(mmio_base + SSCR1);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val &= ~SSCR1_TSRE;
		else
			val &= ~SSCR1_RSRE;
		__raw_writel(val, mmio_base + SSCR1);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		ssp_disable(&ssp[cpu_dai->id]);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		val = __raw_readl(mmio_base + SSCR1);
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			val &= ~SSCR1_TSRE;
		else
			val &= ~SSCR1_RSRE;
		__raw_writel(val, mmio_base + SSCR1);
		break;

	default:
		ret = -EINVAL;
	}

	pr_debug("SSCR0 0x%08x SSCR1 0x%08x SSTO 0x%08x SSPSP 0x%08x SSSR 0x%08x",
		__raw_readl(mmio_base + SSCR0), __raw_readl(mmio_base + SSCR1),
		__raw_readl(mmio_base + SSTO), __raw_readl(mmio_base + SSPSP),
		__raw_readl(mmio_base + SSSR));

	return ret;
}

#define PXA2XX_SSP_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define PXA2XX_SSP_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

struct snd_soc_cpu_dai pxa_ssp_dai[] = {
	{	.name = "pxa2xx-ssp1",
		.id = 0,
		.type = SND_SOC_DAI_PCM,
		.suspend = pxa2xx_ssp_suspend,
		.resume = pxa2xx_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA2XX_SSP_RATES,
			.formats = PXA2XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA2XX_SSP_RATES,
			.formats = PXA2XX_SSP_FORMATS,},
		.ops = {
			.startup = pxa2xx_ssp_startup,
			.shutdown = pxa2xx_ssp_shutdown,
			.trigger = pxa2xx_ssp_trigger,
			.hw_params = pxa2xx_ssp_hw_params,},
		.dai_ops = {
			.set_sysclk = pxa2xx_ssp_set_dai_sysclk,
			.set_clkdiv = pxa2xx_ssp_set_dai_clkdiv,
			.set_pll = pxa2xx_ssp_set_dai_pll,
			.set_fmt = pxa2xx_ssp_set_dai_fmt,
			.set_tdm_slot = pxa2xx_ssp_set_dai_tdm_slot,
			.set_tristate = pxa2xx_ssp_set_dai_tristate,
		},
	},
	{	.name = "pxa2xx-ssp2",
		.id = 1,
		.type = SND_SOC_DAI_PCM,
		.suspend = pxa2xx_ssp_suspend,
		.resume = pxa2xx_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA2XX_SSP_RATES,
			.formats = PXA2XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA2XX_SSP_RATES,
			.formats = PXA2XX_SSP_FORMATS,},
		.ops = {
			.startup = pxa2xx_ssp_startup,
			.shutdown = pxa2xx_ssp_shutdown,
			.trigger = pxa2xx_ssp_trigger,
			.hw_params = pxa2xx_ssp_hw_params,},
		.dai_ops = {
			.set_sysclk = pxa2xx_ssp_set_dai_sysclk,
			.set_clkdiv = pxa2xx_ssp_set_dai_clkdiv,
			.set_pll = pxa2xx_ssp_set_dai_pll,
			.set_fmt = pxa2xx_ssp_set_dai_fmt,
			.set_tdm_slot = pxa2xx_ssp_set_dai_tdm_slot,
			.set_tristate = pxa2xx_ssp_set_dai_tristate,
		},
	},
	{	.name = "pxa2xx-ssp3",
		.id = 2,
		.type = SND_SOC_DAI_PCM,
		.suspend = pxa2xx_ssp_suspend,
		.resume = pxa2xx_ssp_resume,
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA2XX_SSP_RATES,
			.formats = PXA2XX_SSP_FORMATS,},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = PXA2XX_SSP_RATES,
			.formats = PXA2XX_SSP_FORMATS,},
		.ops = {
			.startup = pxa2xx_ssp_startup,
			.shutdown = pxa2xx_ssp_shutdown,
			.trigger = pxa2xx_ssp_trigger,
			.hw_params = pxa2xx_ssp_hw_params,},
		.dai_ops = {
			.set_sysclk = pxa2xx_ssp_set_dai_sysclk,
			.set_clkdiv = pxa2xx_ssp_set_dai_clkdiv,
			.set_pll = pxa2xx_ssp_set_dai_pll,
			.set_fmt = pxa2xx_ssp_set_dai_fmt,
			.set_tdm_slot = pxa2xx_ssp_set_dai_tdm_slot,
			.set_tristate = pxa2xx_ssp_set_dai_tristate,
		},
	},
};
EXPORT_SYMBOL_GPL(pxa_ssp_dai);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("pxa2xx SSP/PCM SoC Interface");
MODULE_LICENSE("GPL");
